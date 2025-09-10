#!/usr/bin/env python3
"""
🔍 Photo Capture URL Discovery for Jidetech Camera

This script tests various HTTP snapshot URLs to find the working endpoint
for photo capture, helping to fix the 0-byte image issue.
"""

import requests
import os
import time
from requests.auth import HTTPBasicAuth
from datetime import datetime
from typing import List, Tuple, Optional

class PhotoCaptureDiscovery:
    """Discover working photo capture URLs for Jidetech camera."""
    
    def __init__(self, ip: str = "192.168.0.18", username: str = "admin", password: str = "admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        self.session.auth = HTTPBasicAuth(username, password)
        self.timeout = 10
        
        # Create test directory
        self.test_dir = "/tmp/jidetech_photo_test"
        os.makedirs(self.test_dir, exist_ok=True)
    
    def test_snapshot_urls(self) -> List[Tuple[str, bool, int, str]]:
        """
        Test various snapshot URLs to find working ones.
        
        Returns:
            List of (url, success, content_length, message) tuples
        """
        print("🔍 Testing Photo Capture URLs")
        print("="*60)
        
        # Comprehensive list of possible snapshot URLs
        snapshot_urls = [
            # Jidetech/Dahua style URLs
            "/cgi-bin/snapshot.cgi",
            "/snapshot.cgi", 
            "/cgi-bin/snapshot.cgi?channel=0",
            "/cgi-bin/snapshot.cgi?channel=1",
            "/form/getSnapshot",
            "/form/getSnapshot.cgi",
            
            # Common snapshot endpoints
            "/jpg/image.jpg",
            "/tmpfs/auto.jpg", 
            "/image.jpg",
            "/snapshot.jpg",
            "/picture.jpg",
            "/capture.jpg",
            
            # CGI variations
            "/cgi-bin/hi3510/snap.cgi",
            "/cgi-bin/hi3510/snapshot.cgi",
            "/webcapture.jpg",
            "/axis-cgi/jpg/image.cgi",
            "/cgi-bin/jpeg/snap.cgi",
            "/cgi-bin/mjpg/video.cgi",
            
            # Alternative formats
            "/video.jpg",
            "/live.jpg",
            "/current.jpg",
            "/snap.jpg",
            "/frame.jpg",
            
            # Hikvision style
            "/ISAPI/Streaming/channels/1/picture",
            "/ISAPI/Streaming/channels/101/picture",
            
            # Generic formats
            "/api/snapshot",
            "/api/image",
            "/stream/snapshot",
            "/v1/snapshot",
        ]
        
        results = []
        
        for endpoint in snapshot_urls:
            url = f"http://{self.ip}{endpoint}"
            print(f"\n🔍 Testing: {url}")
            
            # Test with different parameter combinations
            param_sets = [
                {},  # No parameters
                {"channel": "0"},
                {"channel": "1"}, 
                {"quality": "100"},
                {"width": "1920", "height": "1080"},
                {"channel": "0", "quality": "100"},
                {"channel": "0", "width": "1920", "height": "1080", "quality": "95"},
            ]
            
            best_result = None
            
            for i, params in enumerate(param_sets):
                try:
                    if params:
                        print(f"   📋 Params: {params}")
                    
                    response = self.session.get(url, params=params, timeout=self.timeout)
                    content_length = len(response.content)
                    
                    print(f"   📊 Status: {response.status_code}, Size: {content_length} bytes")
                    
                    if response.status_code == 200 and content_length > 1000:
                        # This looks like a valid image
                        timestamp = datetime.now().strftime("%H%M%S")
                        test_filename = f"test_{endpoint.replace('/', '_')}_{i}_{timestamp}.jpg"
                        test_path = os.path.join(self.test_dir, test_filename)
                        
                        # Save test image
                        with open(test_path, 'wb') as f:
                            f.write(response.content)
                        
                        # Verify it's a valid image by checking JPEG header
                        is_valid_jpeg = self._is_valid_jpeg(test_path)
                        
                        if is_valid_jpeg:
                            message = f"✅ SUCCESS! Valid JPEG saved: {test_path}"
                            print(f"   {message}")
                            
                            if not best_result or content_length > best_result[2]:
                                best_result = (url, True, content_length, message, params)
                        else:
                            message = f"⚠️  File saved but not valid JPEG"
                            print(f"   {message}")
                    
                    elif response.status_code == 200:
                        message = f"⚠️  HTTP 200 but small content ({content_length} bytes)"
                        print(f"   {message}")
                    
                    else:
                        message = f"❌ HTTP {response.status_code}"
                        print(f"   {message}")
                        
                except Exception as e:
                    message = f"❌ Error: {e}"
                    print(f"   {message}")
            
            # Add best result for this URL
            if best_result:
                results.append((best_result[0], best_result[1], best_result[2], best_result[3]))
            else:
                results.append((url, False, 0, "No working parameters found"))
        
        return results
    
    def _is_valid_jpeg(self, filepath: str) -> bool:
        """Check if file is a valid JPEG image."""
        try:
            with open(filepath, 'rb') as f:
                # Read first few bytes
                header = f.read(10)
                
                # Check JPEG magic bytes
                if len(header) >= 2 and header[:2] == b'\xff\xd8':
                    # Check for JFIF or EXIF markers
                    if b'JFIF' in header or b'Exif' in header:
                        return True
                    
                    # Basic JPEG check - starts with FFD8
                    return True
                
                return False
        except:
            return False
    
    def test_rtsp_snapshot(self) -> Tuple[bool, str]:
        """Test RTSP snapshot capture as fallback."""
        print("\n🎥 Testing RTSP Snapshot Capture")
        print("="*40)
        
        try:
            import cv2
            
            # RTSP URL
            rtsp_url = f"rtsp://{self.username}:{self.password}@{self.ip}:554/h264/ch1/main/av_stream"
            print(f"📡 RTSP URL: {rtsp_url}")
            
            # Open RTSP stream
            cap = cv2.VideoCapture(rtsp_url)
            
            if not cap.isOpened():
                return False, "Failed to open RTSP stream"
            
            # Read frame
            ret, frame = cap.read()
            
            if ret:
                # Save frame
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                rtsp_path = os.path.join(self.test_dir, f"rtsp_snapshot_{timestamp}.jpg")
                
                cv2.imwrite(rtsp_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                cap.release()
                
                if os.path.exists(rtsp_path) and os.path.getsize(rtsp_path) > 1000:
                    return True, f"✅ RTSP snapshot saved: {rtsp_path}"
                else:
                    return False, "RTSP snapshot file is too small"
            else:
                cap.release()
                return False, "Failed to read frame from RTSP stream"
                
        except ImportError:
            return False, "OpenCV not available for RTSP testing"
        except Exception as e:
            return False, f"RTSP snapshot error: {e}"
    
    def generate_curl_commands(self, working_urls: List[Tuple[str, bool, int, str]]):
        """Generate curl commands for working URLs."""
        print("\n🔧 Working cURL Commands")
        print("="*40)
        
        working = [result for result in working_urls if result[1]]
        
        if working:
            print("Copy these commands to test manually:")
            print()
            
            for url, success, size, message in working:
                print(f"# {message}")
                print(f'curl -u admin:admin "{url}" -o test_photo.jpg')
                print(f"# Expected size: ~{size} bytes")
                print()
        else:
            print("❌ No working HTTP snapshot URLs found")
            print("   Try the RTSP method or check camera documentation")

def main():
    """Main discovery function."""
    print("🔍 Jidetech Camera - Photo Capture URL Discovery")
    print("="*60)
    print("Goal: Find working HTTP snapshot URLs to fix 0-byte image issue")
    print("="*60)
    
    discovery = PhotoCaptureDiscovery()
    
    try:
        # Test HTTP snapshot URLs
        results = discovery.test_snapshot_urls()
        
        # Test RTSP as fallback
        rtsp_success, rtsp_message = discovery.test_rtsp_snapshot()
        
        # Summary
        print("\n" + "="*60)
        print("🎯 DISCOVERY SUMMARY")
        print("="*60)
        
        working_urls = [r for r in results if r[1]]
        
        if working_urls:
            print(f"✅ Found {len(working_urls)} working HTTP snapshot URLs:")
            for url, success, size, message in working_urls:
                print(f"   • {url} ({size} bytes)")
            
            print(f"\n📁 Test images saved in: {discovery.test_dir}")
            print("   Check these files to verify image quality")
            
            # Generate curl commands
            discovery.generate_curl_commands(results)
            
        else:
            print("❌ No working HTTP snapshot URLs found")
        
        if rtsp_success:
            print(f"\n✅ RTSP Fallback: {rtsp_message}")
        else:
            print(f"\n❌ RTSP Fallback: {rtsp_message}")
        
        print(f"\n💡 Recommendations:")
        if working_urls:
            best_url = max(working_urls, key=lambda x: x[2])  # Largest file
            print(f"   • Use this URL in photo service: {best_url[0]}")
            print(f"   • Expected image size: ~{best_url[2]} bytes")
        else:
            print("   • Check camera web interface for snapshot settings")
            print("   • Verify camera supports HTTP snapshot capture")
            print("   • Use RTSP method as fallback")
        
    except KeyboardInterrupt:
        print("\n🛑 Discovery interrupted by user")
    except Exception as e:
        print(f"\n❌ Discovery failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 