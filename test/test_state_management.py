#!/usr/bin/env python3
"""
Test PTZ State Management Issues
Debug why camera only moves on first command then stops responding
"""

import requests
import time
from requests.auth import HTTPBasicAuth

class PTZStateDebugger:
    """Debug PTZ state management issues."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        self.session.auth = HTTPBasicAuth(username, password)
    
    def test_explicit_stop_between_commands(self):
        """Test if explicit stop commands are needed between movements."""
        print("🧪 Testing: Explicit STOP between commands")
        print("=" * 50)
        
        commands = [
            {"name": "Pan Right", "panSpeed": "8", "tiltSpeed": "0"},
            {"name": "Pan Left", "panSpeed": "-8", "tiltSpeed": "0"}, 
            {"name": "Tilt Up", "panSpeed": "0", "tiltSpeed": "8"},
            {"name": "Tilt Down", "panSpeed": "0", "tiltSpeed": "-8"}
        ]
        
        for i, cmd in enumerate(commands, 1):
            print(f"\n🎯 Test {i}: {cmd['name']}")
            
            # 1. Send START command
            print("   1. Sending START command...")
            start_response = self._send_command("0", cmd["panSpeed"], cmd["tiltSpeed"])
            print(f"      Status: {start_response.status_code}")
            
            if start_response.status_code == 200:
                print("   2. Moving for 2 seconds...")
                time.sleep(2)
                
                # 2. Send EXPLICIT STOP command
                print("   3. Sending EXPLICIT STOP command...")
                stop_response = self._send_command("3", cmd["panSpeed"], cmd["tiltSpeed"])
                print(f"      Status: {stop_response.status_code}")
                
                # 3. Wait before next command
                print("   4. Waiting 2 seconds before next command...")
                time.sleep(2)
            
            print(f"   ✅ Test {i} completed")
    
    def test_timing_delays(self):
        """Test different timing delays between commands."""
        print("\n⏱️  Testing: Different timing delays")
        print("=" * 50)
        
        delays = [0.5, 1.0, 2.0, 3.0]
        
        for delay in delays:
            print(f"\n🎯 Testing with {delay}s delay:")
            
            # Pan Right
            print("   Pan Right...")
            self._send_command("0", "8", "0")
            time.sleep(1)
            self._send_command("3", "8", "0")
            
            print(f"   Waiting {delay} seconds...")
            time.sleep(delay)
            
            # Pan Left
            print("   Pan Left...")
            self._send_command("0", "-8", "0")
            time.sleep(1)
            self._send_command("3", "-8", "0")
            
            time.sleep(1)
    
    def test_session_refresh(self):
        """Test if session refresh helps."""
        print("\n🔄 Testing: Session refresh between commands")
        print("=" * 50)
        
        commands = [
            {"name": "Pan Right", "panSpeed": "8", "tiltSpeed": "0"},
            {"name": "Pan Left", "panSpeed": "-8", "tiltSpeed": "0"},
            {"name": "Tilt Up", "panSpeed": "0", "tiltSpeed": "8"}
        ]
        
        for i, cmd in enumerate(commands, 1):
            print(f"\n🎯 Test {i}: {cmd['name']} (Fresh session)")
            
            # Create fresh session for each command
            fresh_session = requests.Session()
            fresh_session.auth = HTTPBasicAuth(self.username, self.password)
            
            # Send command with fresh session
            start_response = fresh_session.get(
                f"http://{self.ip}/form/setPTZCfg",
                params={
                    "command": "0",
                    "ZFSpeed": "0",
                    "PTSpeed": "0",
                    "panSpeed": cmd["panSpeed"],
                    "tiltSpeed": cmd["tiltSpeed"]
                },
                timeout=5
            )
            
            print(f"   Start Status: {start_response.status_code}")
            
            if start_response.status_code == 200:
                time.sleep(2)
                
                # Stop with same fresh session
                stop_response = fresh_session.get(
                    f"http://{self.ip}/form/setPTZCfg",
                    params={
                        "command": "3",
                        "ZFSpeed": "0",
                        "PTSpeed": "0",
                        "panSpeed": cmd["panSpeed"],
                        "tiltSpeed": cmd["tiltSpeed"]
                    },
                    timeout=5
                )
                print(f"   Stop Status: {stop_response.status_code}")
            
            time.sleep(2)
    
    def test_zero_reset_between_commands(self):
        """Test sending zero speeds to reset state."""
        print("\n🔄 Testing: Zero speed reset between commands")
        print("=" * 50)
        
        commands = [
            {"name": "Pan Right", "panSpeed": "8", "tiltSpeed": "0"},
            {"name": "Pan Left", "panSpeed": "-8", "tiltSpeed": "0"},
            {"name": "Tilt Up", "panSpeed": "0", "tiltSpeed": "8"}
        ]
        
        for i, cmd in enumerate(commands, 1):
            print(f"\n🎯 Test {i}: {cmd['name']} (with zero reset)")
            
            # 1. Send movement command
            print("   1. Sending movement command...")
            start_response = self._send_command("0", cmd["panSpeed"], cmd["tiltSpeed"])
            print(f"      Status: {start_response.status_code}")
            
            if start_response.status_code == 200:
                time.sleep(2)
                
                # 2. Send stop command
                print("   2. Sending stop command...")
                self._send_command("3", cmd["panSpeed"], cmd["tiltSpeed"])
                
                # 3. Send zero reset command
                print("   3. Sending zero reset...")
                zero_response = self._send_command("0", "0", "0")
                print(f"      Zero Status: {zero_response.status_code}")
                time.sleep(0.5)
                
                # 4. Send stop for zero command
                self._send_command("3", "0", "0")
                
                time.sleep(1)
    
    def test_single_command_repeatedly(self):
        """Test the same command repeatedly to isolate the issue."""
        print("\n🔁 Testing: Same command repeatedly")
        print("=" * 50)
        
        for i in range(5):
            print(f"\n🎯 Attempt {i+1}: Pan Right")
            
            # Start
            start_response = self._send_command("0", "8", "0")
            print(f"   Start Status: {start_response.status_code}")
            
            if start_response.status_code == 200:
                print("   Moving for 1 second...")
                time.sleep(1)
                
                # Stop
                stop_response = self._send_command("3", "8", "0")
                print(f"   Stop Status: {stop_response.status_code}")
                
                # Wait
                print("   Waiting 3 seconds...")
                time.sleep(3)
            
            # Ask user for observation
            if i == 0:
                input("   👀 Did the camera move? Press Enter to continue...")
            else:
                user_input = input("   👀 Did the camera move? (y/n/Enter to continue): ")
                if user_input.lower() == 'n':
                    print("   ❌ Camera did not move - issue confirmed!")
                    break
                elif user_input.lower() == 'y':
                    print("   ✅ Camera moved - continuing...")
    
    def _send_command(self, command, pan_speed, tilt_speed):
        """Send a single PTZ command."""
        return self.session.get(
            f"http://{self.ip}/form/setPTZCfg",
            params={
                "command": command,
                "ZFSpeed": "0",
                "PTSpeed": "0", 
                "panSpeed": pan_speed,
                "tiltSpeed": tilt_speed
            },
            timeout=5
        )
    
    def manual_test_commands(self):
        """Manual testing with user input."""
        print("\n🎮 Manual Testing Mode")
        print("=" * 50)
        print("Commands:")
        print("  1 = Pan Right    2 = Pan Left")
        print("  3 = Tilt Up      4 = Tilt Down")
        print("  s = Stop         q = Quit")
        print()
        
        while True:
            cmd = input("Enter command (1-4, s, q): ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 's':
                print("Sending STOP...")
                response = self._send_command("3", "0", "0")
                print(f"Status: {response.status_code}")
            elif cmd == '1':
                print("Pan Right...")
                response = self._send_command("0", "8", "0")
                print(f"Status: {response.status_code}")
            elif cmd == '2':
                print("Pan Left...")
                response = self._send_command("0", "-8", "0")
                print(f"Status: {response.status_code}")
            elif cmd == '3':
                print("Tilt Up...")
                response = self._send_command("0", "0", "8")
                print(f"Status: {response.status_code}")
            elif cmd == '4':
                print("Tilt Down...")
                response = self._send_command("0", "0", "-8")
                print(f"Status: {response.status_code}")
            else:
                print("Invalid command")

def main():
    """Main test function."""
    print("🔍 PTZ State Management Debugger")
    print("=" * 60)
    print("This tool debugs why camera only moves on first command")
    print()
    
    debugger = PTZStateDebugger()
    
    try:
        print("Running diagnostic tests...")
        print("Watch your camera carefully during each test!")
        print()
        
        # Test 1: Explicit stops
        debugger.test_explicit_stop_between_commands()
        input("\nPress Enter to continue to next test...")
        
        # Test 2: Timing delays
        debugger.test_timing_delays()
        input("\nPress Enter to continue to next test...")
        
        # Test 3: Session refresh
        debugger.test_session_refresh()
        input("\nPress Enter to continue to next test...")
        
        # Test 4: Zero reset
        debugger.test_zero_reset_between_commands()
        input("\nPress Enter to continue to next test...")
        
        # Test 5: Same command repeatedly
        debugger.test_single_command_repeatedly()
        
        # Manual testing
        print("\n" + "="*60)
        manual_test = input("Want to try manual testing? (y/n): ")
        if manual_test.lower() == 'y':
            debugger.manual_test_commands()
        
        print("\n🎯 State management testing completed!")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 