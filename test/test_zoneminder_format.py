#!/usr/bin/env python3
"""
Test ZoneMinder Command Format for Jidetech Camera
Based on the working Perl module: /form/setPTZCfg?command=X
"""

import requests
import time
from requests.auth import HTTPBasicAuth

class ZoneMinderPTZTester:
    """Test PTZ using ZoneMinder command format."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        self.session.auth = HTTPBasicAuth(username, password)
    
    def test_zoneminder_commands(self):
        """Test the exact ZoneMinder command format."""
        print("🎯 Testing ZoneMinder Command Format")
        print("=" * 50)
        print("Format: /form/setPTZCfg?command=X")
        print("Commands: 0=Stop, 1=Up, 2=Down, 3=Left, 4=Right")
        print()
        
        # Test basic movements with explicit stops
        movements = [
            {"name": "Move Up", "command": "1"},
            {"name": "Move Down", "command": "2"},
            {"name": "Move Left", "command": "3"},
            {"name": "Move Right", "command": "4"},
            {"name": "Move Up-Left", "command": "6"},
            {"name": "Move Up-Right", "command": "7"},
            {"name": "Move Down-Left", "command": "5"},
            {"name": "Move Down-Right", "command": "8"}
        ]
        
        for move in movements:
            print(f"\n🎯 Testing: {move['name']}")
            
            # 1. Send movement command
            print(f"   1. Sending command={move['command']}...")
            move_response = self._send_command(move['command'])
            print(f"      Status: {move_response.status_code}")
            
            if move_response.status_code == 200:
                print("   2. Moving for 2 seconds...")
                time.sleep(2)
                
                # 2. Send STOP command (command=0)
                print("   3. Sending STOP (command=0)...")
                stop_response = self._send_command("0")
                print(f"      Status: {stop_response.status_code}")
                
                print("   4. Waiting 1 second before next command...")
                time.sleep(1)
            
            print(f"   ✅ {move['name']} completed")
    
    def test_zoom_focus_commands(self):
        """Test zoom and focus commands."""
        print("\n🔍 Testing Zoom and Focus Commands")
        print("=" * 50)
        
        zoom_focus = [
            {"name": "Focus Near", "command": "12"},
            {"name": "Focus Far", "command": "11"},
            {"name": "Zoom In (Tele)", "command": "13"},
            {"name": "Zoom Out (Wide)", "command": "14"},
            {"name": "Iris Open", "command": "9"},
            {"name": "Iris Close", "command": "10"}
        ]
        
        for zf in zoom_focus:
            print(f"\n🎯 Testing: {zf['name']}")
            
            # Send command
            print(f"   Sending command={zf['command']}...")
            response = self._send_command(zf['command'])
            print(f"   Status: {response.status_code}")
            
            if response.status_code == 200:
                print("   Operating for 1 second...")
                time.sleep(1)
                
                # Stop
                print("   Sending STOP...")
                self._send_command("0")
                time.sleep(0.5)
    
    def test_preset_commands(self):
        """Test preset commands."""
        print("\n📍 Testing Preset Commands")
        print("=" * 50)
        
        # Test setting and recalling presets
        for preset in [1, 2, 3]:
            print(f"\n🎯 Testing Preset {preset}")
            
            # Set preset
            print(f"   Setting preset {preset}...")
            set_url = f"http://{self.ip}/form/presetSet?existFlag=1&flag=3&language=en&presetNum={preset}"
            set_response = self.session.get(set_url, timeout=5)
            print(f"   Set Status: {set_response.status_code}")
            
            time.sleep(1)
            
            # Recall preset
            print(f"   Recalling preset {preset}...")
            recall_url = f"http://{self.ip}/form/presetSet?existFlag=1&flag=4&language=en&presetNum={preset}"
            recall_response = self.session.get(recall_url, timeout=5)
            print(f"   Recall Status: {recall_response.status_code}")
            
            time.sleep(2)
    
    def test_patrol_commands(self):
        """Test patrol commands."""
        print("\n🚁 Testing Patrol Commands")
        print("=" * 50)
        
        print("🎯 Starting Horizontal Patrol...")
        patrol_url = f"http://{self.ip}/form/setSpecialFunc?command=1&flag=2"
        patrol_response = self.session.get(patrol_url, timeout=5)
        print(f"   Status: {patrol_response.status_code}")
        
        if patrol_response.status_code == 200:
            print("   Patrol running for 10 seconds...")
            time.sleep(10)
            
            print("   Stopping patrol...")
            self._send_command("0")
    
    def _send_command(self, command):
        """Send a single PTZ command using ZoneMinder format."""
        url = f"http://{self.ip}/form/setPTZCfg"
        params = {"command": command}
        return self.session.get(url, params=params, timeout=5)
    
    def manual_test(self):
        """Manual testing with ZoneMinder commands."""
        print("\n🎮 Manual ZoneMinder Command Testing")
        print("=" * 50)
        print("Commands:")
        print("  1=Up     2=Down   3=Left   4=Right")
        print("  5=DL     6=UL     7=UR     8=DR")
        print("  9=Iris+  10=Iris- 11=Focus+ 12=Focus-")
        print("  13=Zoom+ 14=Zoom- 0=STOP   q=Quit")
        print()
        
        while True:
            cmd = input("Enter command (0-14, q): ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd.isdigit() and 0 <= int(cmd) <= 14:
                print(f"Sending command={cmd}...")
                response = self._send_command(cmd)
                print(f"Status: {response.status_code}")
                
                # Auto-stop after 2 seconds for movement commands
                if int(cmd) in [1, 2, 3, 4, 5, 6, 7, 8] and int(cmd) != 0:
                    print("Auto-stopping in 2 seconds...")
                    time.sleep(2)
                    self._send_command("0")
                    print("Stopped.")
            else:
                print("Invalid command")
    
    def generate_curl_commands(self):
        """Generate working curl commands."""
        print("\n🔧 Working cURL Commands (ZoneMinder Format)")
        print("=" * 50)
        
        commands = [
            ("Move Up", "1"),
            ("Move Down", "2"), 
            ("Move Left", "3"),
            ("Move Right", "4"),
            ("STOP", "0")
        ]
        
        for name, cmd in commands:
            print(f"{name}:")
            print(f'  curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command={cmd}"')
        
        print("\nExample sequence:")
        print(f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=4"  # Move Right')
        print("sleep 2")
        print(f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=0"  # Stop')

def main():
    """Main test function."""
    print("🚀 ZoneMinder PTZ Command Tester")
    print("=" * 60)
    print("Testing the WORKING command format from ZoneMinder Perl module")
    print("Format: /form/setPTZCfg?command=X")
    print("Key: Always send command=0 (STOP) between movements!")
    print()
    
    tester = ZoneMinderPTZTester()
    
    try:
        # Test basic movements
        tester.test_zoneminder_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test zoom/focus commands...")
        tester.test_zoom_focus_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test preset commands...")
        tester.test_preset_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test patrol commands...")
        tester.test_patrol_commands()
        
        # Show curl commands
        tester.generate_curl_commands()
        
        # Manual testing
        print("\n" + "="*60)
        manual_test = input("Want to try manual testing? (y/n): ")
        if manual_test.lower() == 'y':
            tester.manual_test()
        
        print("\n🎯 ZoneMinder format testing completed!")
        print("This format should work reliably with explicit STOP commands!")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 