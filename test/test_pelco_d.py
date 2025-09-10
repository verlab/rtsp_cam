#!/usr/bin/env python3
"""
Pelco-D Protocol Test for Jidetech P1-$X-2mp
Based on camera configuration: Address 1, Pelco-D, 9600 baud
"""

import requests
import time
from requests.auth import HTTPBasicAuth, HTTPDigestAuth

class PelcoDTester:
    """Test Pelco-D protocol commands for Jidetech cameras."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        
    def test_pelco_d_commands(self):
        """Test Pelco-D specific commands."""
        print("🔍 Testing Pelco-D Protocol Commands")
        print("=" * 50)
        print("Camera Config: Address 1, Pelco-D, 9600 baud")
        print()
        
        # Pelco-D specific commands with address 1
        commands = [
            {
                "name": "Pan Right (Pelco-D)",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "1", "arg2": "20", "arg3": "0"}
            },
            {
                "name": "Pan Left (Pelco-D)",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi", 
                "params": {"action": "start", "channel": "0", "code": "Left", "arg1": "1", "arg2": "20", "arg3": "0"}
            },
            {
                "name": "Tilt Up (Pelco-D)",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Up", "arg1": "1", "arg2": "20", "arg3": "0"}
            },
            {
                "name": "Tilt Down (Pelco-D)",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Down", "arg1": "1", "arg2": "20", "arg3": "0"}
            },
            {
                "name": "Zoom In (Pelco-D)",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "ZoomTele", "arg1": "1", "arg2": "20", "arg3": "0"}
            },
            {
                "name": "Zoom Out (Pelco-D)",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi", 
                "params": {"action": "start", "channel": "0", "code": "ZoomWide", "arg1": "1", "arg2": "20", "arg3": "0"}
            }
        ]
        
        for cmd in commands:
            print(f"\nTesting: {cmd['name']}")
            success = self._send_cgi_command(cmd)
            if success:
                print("   ⏱️  Moving for 3 seconds...")
                time.sleep(3)
                # Send stop command
                stop_cmd = {
                    "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                    "params": {"action": "stop", "channel": "0", "code": cmd['params']['code'], "arg1": "1"}
                }
                print("   🛑 Sending stop command...")
                self._send_cgi_command(stop_cmd)
            time.sleep(1)
    
    def test_different_speeds(self):
        """Test different speed values."""
        print("\n🔍 Testing Different Speed Values")
        print("=" * 50)
        
        speeds = [10, 20, 30, 40, 50]
        
        for speed in speeds:
            print(f"\nTesting Pan Right at speed {speed}")
            cmd = {
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "1", "arg2": str(speed), "arg3": "0"}
            }
            
            success = self._send_cgi_command(cmd)
            if success:
                print(f"   ⏱️  Moving at speed {speed} for 2 seconds...")
                time.sleep(2)
                # Stop
                stop_cmd = {
                    "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                    "params": {"action": "stop", "channel": "0", "code": "Right", "arg1": "1"}
                }
                self._send_cgi_command(stop_cmd)
                time.sleep(1)
    
    def test_alternative_formats(self):
        """Test alternative Pelco-D command formats."""
        print("\n🔍 Testing Alternative Pelco-D Formats")
        print("=" * 50)
        
        formats = [
            {
                "name": "Format 1 - Standard Pelco-D",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "1", "arg2": "20", "arg3": "0"}
            },
            {
                "name": "Format 2 - Address in Channel",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "1", "code": "Right", "arg1": "0", "arg2": "20", "arg3": "0"}
            },
            {
                "name": "Format 3 - Pelco-D with Protocol",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "protocol": "Pelco-D", "address": "1", "code": "Right", "speed": "20"}
            },
            {
                "name": "Format 4 - Simple Pelco-D",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"cmd": "ptz", "address": "1", "code": "Right", "speed": "20"}
            },
            {
                "name": "Format 5 - Direct Pelco",
                "url": f"http://{self.ip}/cgi-bin/pelco.cgi",
                "params": {"address": "1", "command": "Right", "speed": "20"}
            }
        ]
        
        for fmt in formats:
            print(f"\nTesting: {fmt['name']}")
            success = self._send_cgi_command(fmt)
            if success:
                time.sleep(2)
                print("   🛑 Stopping...")
                # Try to stop
                stop_cmd = {
                    "url": fmt["url"],
                    "params": dict(fmt["params"])
                }
                stop_cmd["params"]["action"] = "stop"
                self._send_cgi_command(stop_cmd)
            time.sleep(1)
    
    def test_preset_positions(self):
        """Test preset position commands."""
        print("\n🔍 Testing Preset Positions")
        print("=" * 50)
        
        presets = [
            {"name": "Go to Preset 1", "preset": 1},
            {"name": "Go to Preset 2", "preset": 2}, 
            {"name": "Go to Preset 3", "preset": 3},
            {"name": "Disable Auto-Track (93)", "preset": 93},
            {"name": "Enable Auto-Track (99)", "preset": 99},
            {"name": "Home Position (0)", "preset": 0}
        ]
        
        for preset in presets:
            print(f"\nTesting: {preset['name']}")
            
            # Try different preset command formats
            formats = [
                {
                    "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                    "params": {"action": "start", "channel": "0", "code": "GotoPreset", "arg1": "1", "arg2": str(preset['preset']), "arg3": "0"}
                },
                {
                    "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                    "params": {"action": "start", "channel": "1", "code": "GotoPreset", "arg1": "0", "arg2": str(preset['preset']), "arg3": "0"}
                },
                {
                    "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                    "params": {"cmd": "preset", "address": "1", "preset": str(preset['preset'])}
                }
            ]
            
            for i, fmt in enumerate(formats, 1):
                print(f"   Format {i}: ", end="")
                success = self._send_cgi_command(fmt)
                if success:
                    print(" ✅ SUCCESS!")
                    break
            
            time.sleep(2)
    
    def _send_cgi_command(self, cmd):
        """Send CGI command with authentication."""
        auth_methods = [
            ("Basic", HTTPBasicAuth(self.username, self.password)),
            ("Digest", HTTPDigestAuth(self.username, self.password)),
            ("None", None)
        ]
        
        for auth_name, auth in auth_methods:
            try:
                response = requests.get(cmd["url"], params=cmd["params"], auth=auth, timeout=5)
                if response.status_code == 200:
                    print(f"✅ {auth_name} Auth: Status {response.status_code}")
                    if len(response.text) < 100 and response.text.strip():
                        print(f"   Response: {response.text}")
                    return True
                elif response.status_code == 401:
                    continue  # Try next auth method
                else:
                    print(f"⚠️  {auth_name} Auth: Status {response.status_code}")
                    
            except requests.exceptions.ConnectTimeout:
                print(f"⏱️  {auth_name} Auth: Timeout")
            except requests.exceptions.ConnectionError:
                print(f"🔌 {auth_name} Auth: Connection error")
            except Exception as e:
                print(f"❌ {auth_name} Auth: {str(e)[:30]}...")
        
        print("❌ All auth methods failed")
        return False

def main():
    """Main test function."""
    print("🚀 Pelco-D Protocol PTZ Tester")
    print("=" * 60)
    print("Camera Configuration:")
    print("  Address: 1")
    print("  Protocol: Pelco-D") 
    print("  Baud Rate: 9600")
    print("  3D Protocol: Pelco-DH")
    print("  User Mode: HK-TST")
    print()
    print("This test uses the correct Pelco-D format based on your camera settings.")
    print("Watch your camera for movement!")
    print()
    
    input("Press Enter to start testing...")
    
    tester = PelcoDTester()
    
    try:
        # Test basic Pelco-D commands
        tester.test_pelco_d_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test different speeds...")
        tester.test_different_speeds()
        
        print("\n" + "="*60)
        input("Press Enter to test alternative formats...")
        tester.test_alternative_formats()
        
        print("\n" + "="*60)
        input("Press Enter to test preset positions...")
        tester.test_preset_positions()
        
        print("\n🎯 Pelco-D testing completed!")
        print("\nKey findings to look for:")
        print("1. Commands that cause visible camera movement")
        print("2. Successful preset commands (especially 93/99 for auto-tracking)")
        print("3. Working speed values and command formats")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 