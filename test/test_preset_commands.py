#!/usr/bin/env python3
"""
Preset Commands Test for Jidetech P1-$X-2mp
Based on findings from GitHub and forum discussions.
"""

import requests
import time
from requests.auth import HTTPBasicAuth, HTTPDigestAuth

class JidetechPresetTester:
    """Test preset commands for Jidetech cameras."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        
    def test_preset_commands(self):
        """Test various preset commands."""
        print("🔍 Testing Preset Commands")
        print("=" * 50)
        
        # Based on forum findings
        presets = [
            {"name": "Disable Auto-Track", "preset": 93, "description": "Should disable auto-tracking if present"},
            {"name": "Enable Auto-Track", "preset": 99, "description": "Should enable auto-tracking if present"},
            {"name": "Preset 1", "preset": 1, "description": "Standard preset position"},
            {"name": "Preset 2", "preset": 2, "description": "Standard preset position"},
            {"name": "Preset 3", "preset": 3, "description": "Standard preset position"},
            {"name": "Home Position", "preset": 0, "description": "Return to home position"},
        ]
        
        for preset in presets:
            print(f"\nTesting: {preset['name']} (Preset {preset['preset']})")
            print(f"Description: {preset['description']}")
            self._send_preset_command(preset['preset'])
            time.sleep(3)
    
    def test_cgi_ptz_commands(self):
        """Test CGI-based PTZ commands."""
        print("\n🔍 Testing CGI PTZ Commands")
        print("=" * 50)
        
        # Based on HX IP Camera CGI instruction findings
        commands = [
            {
                "name": "Pan Right",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Pan Left", 
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Left", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Tilt Up",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Up", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Tilt Down",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi", 
                "params": {"action": "start", "channel": "0", "code": "Down", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Zoom In",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "ZoomTele", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Zoom Out",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "ZoomWide", "arg1": "0", "arg2": "5", "arg3": "0"}
            }
        ]
        
        for cmd in commands:
            print(f"\nTesting: {cmd['name']}")
            success = self._send_cgi_command(cmd)
            if success:
                time.sleep(2)
                # Send stop command
                stop_cmd = {
                    "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                    "params": {"action": "stop", "channel": "0", "code": cmd['params']['code']}
                }
                print("   Sending stop command...")
                self._send_cgi_command(stop_cmd)
            time.sleep(1)
    
    def test_alternative_cgi_formats(self):
        """Test alternative CGI command formats."""
        print("\n🔍 Testing Alternative CGI Formats")
        print("=" * 50)
        
        # Different CGI formats found in research
        formats = [
            {
                "name": "Format 1 - Simple",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "code": "Right", "arg1": "5", "arg2": "5"}
            },
            {
                "name": "Format 2 - No Channel",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "code": "Right", "arg1": "0", "arg2": "5"}
            },
            {
                "name": "Format 3 - HX Style",
                "url": f"http://{self.ip}/cgi-bin/hi3510/ptzctrl.cgi",
                "params": {"action": "start", "code": "Right", "arg1": "0", "arg2": "5"}
            },
            {
                "name": "Format 4 - Direct PTZ",
                "url": f"http://{self.ip}/ptz.cgi",
                "params": {"action": "start", "code": "Right", "speed": "5"}
            }
        ]
        
        for fmt in formats:
            print(f"\nTesting: {fmt['name']}")
            self._send_cgi_command(fmt)
            time.sleep(2)
    
    def _send_preset_command(self, preset_num):
        """Send preset command via CGI."""
        commands_to_try = [
            # Standard CGI preset format
            {
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "GotoPreset", "arg1": "0", "arg2": str(preset_num), "arg3": "0"}
            },
            # Alternative preset format
            {
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "code": "GotoPreset", "arg1": str(preset_num)}
            },
            # Simple preset format
            {
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"preset": str(preset_num)}
            },
            # HX style preset
            {
                "url": f"http://{self.ip}/cgi-bin/hi3510/ptzctrl.cgi",
                "params": {"action": "start", "code": "GotoPreset", "arg1": str(preset_num)}
            }
        ]
        
        for i, cmd in enumerate(commands_to_try, 1):
            print(f"   Format {i}: ", end="")
            success = self._send_cgi_command(cmd)
            if success:
                print(" ✅ SUCCESS!")
                return True
        
        return False
    
    def _send_cgi_command(self, cmd):
        """Send CGI command with different authentication methods."""
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
    print("🚀 Jidetech Preset & CGI Command Tester")
    print("=" * 60)
    print("Based on GitHub repository and forum findings:")
    print("- Preset 93: Disable auto-tracking")
    print("- Preset 99: Enable auto-tracking") 
    print("- CGI commands for PTZ control")
    print()
    print("Watch your camera for movement!")
    print()
    
    input("Press Enter to start testing...")
    
    tester = JidetechPresetTester()
    
    try:
        # Test preset commands first (most likely to work)
        tester.test_preset_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test CGI PTZ commands...")
        tester.test_cgi_ptz_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test alternative CGI formats...")
        tester.test_alternative_cgi_formats()
        
        print("\n🎯 Preset and CGI testing completed!")
        print("\nKey findings to look for:")
        print("1. Any commands that returned HTTP 200")
        print("2. Visible camera movement")
        print("3. Changes in camera behavior (auto-tracking on/off)")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 