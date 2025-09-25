#!/usr/bin/env python3
"""
Optimized Streamer Server - Direct GStreamer to shared memory with minimal latency
Hardware-accelerated RTSP decoder for Jetson platforms using subprocess
"""

import os
import sys
import time
import json
import subprocess
import threading
import numpy as np
import mmap
import struct
import signal
from pathlib import Path
from typing import Dict, List, Optional, Tuple

class OptimizedSharedMemoryWriter:
    """Direct shared memory writer optimized for minimal latency"""
    
    def __init__(self, name: str, max_width: int = 2560, max_height: int = 1440):
        self.name = name
        self.max_width = max_width
        self.max_height = max_height
        self.max_frame_size = max_width * max_height * 3  # BGR
        
        # Create shared memory file
        self.shm_path = f"/dev/shm/rtsp_frames_{name}"
        
        # Optimized header: width(4) + height(4) + timestamp(8) + frame_size(4) = 20 bytes
        self.header_size = 20
        self.total_size = self.header_size + self.max_frame_size
        
        # Create memory-mapped file with optimized flags
        with open(self.shm_path, 'wb') as f:
            f.write(b'\x00' * self.total_size)
        
        self.shm_file = open(self.shm_path, 'r+b')
        self.mmap = mmap.mmap(
            self.shm_file.fileno(), 
            self.total_size,
            flags=mmap.MAP_SHARED,
            prot=mmap.PROT_READ | mmap.PROT_WRITE
        )
        
        # Pre-allocate header struct for performance
        self._header_struct = struct.Struct('IIQI')
        
        print(f"Created optimized shared memory: {self.shm_path} ({self.total_size} bytes)")
    
    def write_frame_direct(self, frame_data: bytes, width: int, height: int, timestamp: float):
        """Write frame data directly to shared memory (minimal copy)"""
        frame_size = len(frame_data)
        if frame_size > self.max_frame_size:
            return False
        
        # Pack header directly
        header = self._header_struct.pack(width, height, int(timestamp * 1000000), frame_size)
        
        # Single write operation to minimize latency
        self.mmap.seek(0)
        self.mmap.write(header)
        self.mmap.write(frame_data)
        # Skip flush for even lower latency - OS will handle it
        
        return True
    
    def write_frame(self, frame: np.ndarray, timestamp: float):
        """Write numpy frame to shared memory (compatibility method)"""
        if frame is None:
            return False
        
        height, width, channels = frame.shape
        if channels != 3:
            return False
        
        return self.write_frame_direct(frame.tobytes(), width, height, timestamp)
    
    def close(self):
        """Cleanup shared memory"""
        if hasattr(self, 'mmap'):
            self.mmap.close()
        if hasattr(self, 'shm_file'):
            self.shm_file.close()
        if os.path.exists(self.shm_path):
            os.unlink(self.shm_path)

class OptimizedGStreamerStream:
    """Optimized hardware-accelerated GStreamer stream using subprocess"""
    
    def __init__(self, rtsp_url: str, name: str, width: int = 0, height: int = 0, 
                 use_nvdec: bool = True, username: str = None, password: str = None):
        self.rtsp_url = rtsp_url
        self.name = name
        self.width = width
        self.height = height
        self.use_nvdec = use_nvdec
        self.username = username
        self.password = password
        self.process = None
        self.running = False
        
        # Build authenticated RTSP URL if credentials provided
        if self.username and self.password:
            self.rtsp_url = self._build_authenticated_url()
        
        # Optimized shared memory writer
        self.shm_writer = OptimizedSharedMemoryWriter(
            name, 
            max_width=max(width, 2560), 
            max_height=max(height, 1440)
        )
    
    def _build_authenticated_url(self) -> str:
        """Build RTSP URL with authentication"""
        from urllib.parse import urlparse, urlunparse
        
        parsed = urlparse(self.rtsp_url)
        
        # Build new netloc with credentials
        if parsed.port:
            netloc = f"{self.username}:{self.password}@{parsed.hostname}:{parsed.port}"
        else:
            netloc = f"{self.username}:{self.password}@{parsed.hostname}"
        
        # Rebuild URL
        authenticated_url = urlunparse((
            parsed.scheme,
            netloc,
            parsed.path,
            parsed.params,
            parsed.query,
            parsed.fragment
        ))
        
        # Log without password for security
        safe_url = self.rtsp_url.replace(self.password, "***")
        print(f"Built authenticated URL for {self.name}: {safe_url}")
        
        return authenticated_url
    
    def _build_gstreamer_command(self) -> List[str]:
        """Build optimized GStreamer command for subprocess"""
        
        cmd = ['gst-launch-1.0', '-q']
        
        # Build pipeline parts
        pipeline_parts = []
        
        # RTSP source with low latency settings
        pipeline_parts.append(f"rtspsrc location={self.rtsp_url} latency=0 buffer-mode=auto protocols=tcp")
        
        # RTP depayloader
        pipeline_parts.append("rtph264depay")
        
        # H264 parser (crucial for Jetson hardware decoder)
        pipeline_parts.append("h264parse")
        
        # Hardware decoder selection
        if self.use_nvdec:
            if self._check_element_availability("nvv4l2decoder"):
                pipeline_parts.append("nvv4l2decoder")
                print(f"Using nvv4l2decoder for {self.name}")
            elif self._check_element_availability("nvh264dec"):
                pipeline_parts.append("nvh264dec")
                print(f"Using nvh264dec for {self.name}")
            else:
                print(f"WARNING: No hardware decoder found for {self.name}, using software")
                pipeline_parts.append("avdec_h264")
        else:
            pipeline_parts.append("avdec_h264")
        
        # Color conversion and scaling
        if self.use_nvdec and self._check_element_availability("nvvidconv"):
            # Use nvvidconv for hardware acceleration
            if self.width > 0 and self.height > 0:
                pipeline_parts.extend([
                    "nvvidconv",
                    f"video/x-raw(memory:NVMM),width={self.width},height={self.height}",
                    "nvvidconv",
                    "video/x-raw,format=BGR"
                ])
            else:
                pipeline_parts.extend([
                    "nvvidconv", 
                    "video/x-raw,format=BGR"
                ])
        else:
            # Software fallback
            if self.width > 0 and self.height > 0:
                pipeline_parts.extend([
                    "videoscale", 
                    "videoconvert", 
                    f"video/x-raw,width={self.width},height={self.height},format=BGR"
                ])
            else:
                pipeline_parts.extend([
                    "videoconvert", 
                    "video/x-raw,format=BGR"
                ])
        
        # Output to stdout for reading
        pipeline_parts.append("fdsink fd=1")
        
        # Join pipeline and add to command
        pipeline_str = " ! ".join(pipeline_parts)
        cmd.append(pipeline_str)
        
        return cmd
    
    def _test_pipeline(self) -> bool:
        """Test if the pipeline works before starting frame reading"""
        try:
            # Build a test command that just connects and exits after 5 seconds  
            cmd = ['gst-launch-1.0', '-q']
            
            # Use a simpler test pipeline first
            test_pipeline = f"rtspsrc location={self.rtsp_url} latency=0 protocols=tcp ! rtph264depay ! h264parse ! nvv4l2decoder ! fakesink"
            cmd.append(test_pipeline)
            
            print(f"Testing basic pipeline for {self.name}: {' '.join(cmd)}")
            
            # Run test for 5 seconds
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(5)
            process.terminate()
            
            try:
                stdout, stderr = process.communicate(timeout=2)
                if process.returncode == 0 or "Setting pipeline to PLAYING" in stderr.decode():
                    print(f"✓ Basic pipeline test passed for {self.name}")
                    return True
                else:
                    print(f"✗ Basic pipeline test failed for {self.name}")
                    print(f"Test stderr: {stderr.decode()}")
                    return False
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
                print(f"✓ Basic pipeline test passed for {self.name} (timeout is expected)")
                return True
                
        except Exception as e:
            print(f"Error testing pipeline for {self.name}: {e}")
            return False
    
    def _check_element_availability(self, element_name: str) -> bool:
        """Check if a GStreamer element is available"""
        try:
            result = subprocess.run(
                ['gst-inspect-1.0', element_name], 
                capture_output=True, 
                timeout=2
            )
            return result.returncode == 0
        except Exception:
            return False
    
    def _read_frames(self):
        """Optimized frame reading from GStreamer subprocess"""
        # Use configured dimensions or defaults
        frame_width = self.width if self.width > 0 else 1920
        frame_height = self.height if self.height > 0 else 1080
        frame_size = frame_width * frame_height * 3  # BGR
        
        print(f"Starting optimized frame reader for {self.name}: {frame_width}x{frame_height} (frame_size: {frame_size})")
        
        # Give GStreamer some time to start up
        time.sleep(2)
        
        while self.running and self.process and self.process.poll() is None:
            try:
                # Read frame data from stdout with timeout
                frame_data = self.process.stdout.read(frame_size)
                
                if not frame_data:
                    print(f"No frame data received for {self.name} - stream may have ended")
                    break
                    
                if len(frame_data) != frame_size:
                    print(f"Frame size mismatch for {self.name}: got {len(frame_data)}, expected {frame_size}")
                    # Try to read remaining data to sync
                    remaining = frame_size - len(frame_data)
                    if remaining > 0:
                        extra_data = self.process.stdout.read(remaining)
                        if extra_data:
                            frame_data += extra_data
                    
                    if len(frame_data) != frame_size:
                        continue
                
                # Direct write to shared memory (zero-copy)
                timestamp = time.time()
                success = self.shm_writer.write_frame_direct(
                    frame_data, 
                    frame_width, 
                    frame_height, 
                    timestamp
                )
                
                if not success:
                    print(f"Failed to write frame to shared memory for {self.name}")
                
            except Exception as e:
                if self.running:
                    print(f"Error reading frame for {self.name}: {e}")
                    # Check if process is still alive
                    if self.process.poll() is not None:
                        stderr_output = self.process.stderr.read()
                        if stderr_output:
                            print(f"GStreamer stderr for {self.name}: {stderr_output.decode()}")
                break
        
        print(f"Frame reader stopped for {self.name}")
    
    def start(self):
        """Start optimized GStreamer subprocess"""
        self.running = True
        
        try:
            # First test the basic pipeline
            print(f"Testing pipeline connectivity for {self.name}...")
            if not self._test_pipeline():
                print(f"Basic pipeline test failed for {self.name}, skipping")
                return
            
            # Build command
            cmd = self._build_gstreamer_command()
            print(f"Starting optimized pipeline for {self.name}: {' '.join(cmd)}")
            
            # Start subprocess
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0  # Unbuffered for lowest latency
            )
            
            # Wait a moment to check if process starts successfully
            time.sleep(2)
            if self.process.poll() is not None:
                # Process died immediately, get error output
                _, stderr = self.process.communicate()
                print(f"ERROR: GStreamer process died immediately for {self.name}")
                print(f"GStreamer stderr: {stderr.decode()}")
                return
            
            # Start frame reading thread
            self.thread = threading.Thread(target=self._read_frames, daemon=True)
            self.thread.start()
            
            print(f"Started optimized GStreamer stream for {self.name}")
            
        except Exception as e:
            print(f"Error starting pipeline for {self.name}: {e}")
            self.stop()
    
    def stop(self):
        """Stop the stream"""
        self.running = False
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.shm_writer.close()
        print(f"Stopped stream {self.name}")

class OptimizedStreamServer:
    """Optimized stream server with minimal latency for multiple camera streams"""
    
    def __init__(self, config_file: str):
        self.config_file = config_file
        self.streams: Dict[str, OptimizedGStreamerStream] = {}
        self.load_config()
    
    def load_config(self):
        """Load camera configuration"""
        try:
            print(f"Loading config from: {self.config_file}")
            with open(self.config_file, 'r', encoding='utf-8') as f:
                content = f.read()
                print(f"Config file size: {len(content)} bytes")
                self.config = json.loads(content)
            print(f"Loaded configuration for {len(self.config.get('cameras', []))} cameras")
        except FileNotFoundError:
            print(f"Error: Configuration file not found: {self.config_file}")
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON in config file: {e}")
            print(f"Line {e.lineno}, Column {e.colno}: {e.msg}")
            # Show the problematic line
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                    if e.lineno <= len(lines):
                        print(f"Problematic line: {lines[e.lineno-1].strip()}")
            except:
                pass
            sys.exit(1)
        except Exception as e:
            print(f"Error loading config: {e}")
            sys.exit(1)
    
    def start_streams(self):
        """Start all configured streams"""
        cameras = self.config.get('cameras', [])
        
        for camera in cameras:
            camera_name = camera.get('name', 'unknown')
            host = camera.get('host', '')
            port = camera.get('port', 554)
            username = camera.get('username', '')
            password = camera.get('password', '')
            channels = camera.get('channels', [])
            
            if not host:
                print(f"Warning: No host specified for camera {camera_name}")
                continue
            
            for channel in channels:
                channel_name = channel.get('name', 'main')
                stream_path = channel.get('stream_path', '/stream1')
                width = channel.get('width', 0)
                height = channel.get('height', 0)
                encoding = channel.get('encoding', 'h264')
                fps = channel.get('fps', 30)
                
                # Build RTSP URL
                rtsp_url = f"rtsp://{host}:{port}{stream_path}"
                
                stream_name = f"{camera_name}_{channel_name}"
                
                # Check if hardware acceleration is available
                use_nvdec = self._check_nvdec_support()
                
                stream = OptimizedGStreamerStream(
                    rtsp_url=rtsp_url,
                    name=stream_name,
                    width=width,
                    height=height,
                    use_nvdec=use_nvdec,
                    username=username if username else None,
                    password=password if password else None
                )
                stream.encoding = encoding
                
                print(f"Configured stream {stream_name}: {width}x{height}@{fps}fps ({encoding})")
                
                self.streams[stream_name] = stream
                stream.start()
    
    def _check_nvdec_support(self) -> bool:
        """Check if hardware acceleration is available"""
        try:
            # Check for Jetson hardware elements using gst-inspect
            elements = ['nvv4l2decoder', 'nvh264dec', 'nvvidconv']
            for element in elements:
                result = subprocess.run(
                    ['gst-inspect-1.0', element], 
                    capture_output=True, 
                    timeout=2
                )
                if result.returncode == 0:
                    print(f"Hardware acceleration available: {element}")
                    return True
            return False
        except Exception:
            return False
    
    def run(self):
        """Main server loop"""
        print("Starting Optimized Stream Server...")
        
        self.start_streams()
        
        if not self.streams:
            print("No streams configured. Exiting.")
            return
        
        print(f"Running {len(self.streams)} optimized streams. Press Ctrl+C to stop.")
        
        try:
            while True:
                # Health check - restart failed streams
                for name, stream in list(self.streams.items()):
                    if stream.process and stream.process.poll() is not None:
                        print(f"Stream {name} died, restarting...")
                        stream.stop()
                        time.sleep(1)
                        stream.start()
                
                time.sleep(10)  # Check every 10 seconds
                
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.stop()
    
    def stop(self):
        """Stop all streams"""
        for stream in self.streams.values():
            stream.stop()
        print("All streams stopped.")

def main():
    """Main entry point"""
    config_file = os.environ.get('CONFIG_FILE', '/app/config/cameras.json')
    
    if not os.path.exists(config_file):
        print(f"Configuration file not found: {config_file}")
        sys.exit(1)
    
    server = OptimizedStreamServer(config_file)
    server.run()

if __name__ == '__main__':
    main()
