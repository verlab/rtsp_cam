#!/usr/bin/env python3
"""
Test the WORKING PTZ commands discovered from browser network debug
URL: /form/setPTZCfg?command=X&ZFSpeed=0&PTSpeed=0&panSpeed=8&tiltSpeed=8
"""

import requests
import time
from requests.auth import HTTPBasicAuth

class WorkingPTZTester:
    """Test the actual working PTZ commands from browser debug."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        self.session.auth = HTTPBasicAuth(username, password)
    
    def test_discovered_format(self):
        """Test the exact format discovered from browser network debug."""
        print("🎯 Testing DISCOVERED PTZ Format from Browser Debug")
        print("=" * 60)
        print("URL Pattern: /form/setPTZCfg?command=X&ZFSpeed=0&PTSpeed=0&panSpeed=8&tiltSpeed=8")
        print("Commands: 0=Start, 3=Stop")
        print()
        
        # Test the exact commands from browser debug
        print("🧪 Testing Pan Right Movement:")
        print("1. Starting movement (command=0)...")
        
        start_response = self.session.get(
            f"http://{self.ip}/form/setPTZCfg",
            params={
                "command": "0",
                "ZFSpeed": "0", 
                "PTSpeed": "0",
                "panSpeed": "8",
                "tiltSpeed": "8"
            },
            timeout=5
        )
        
        print(f"   Status: {start_response.status_code}")
        if start_response.text.strip():
            print(f"   Response: {start_response.text[:100]}")
        
        if start_response.status_code == 200:
            print("   ⏱️  Camera should be moving now! Waiting 3 seconds...")
            time.sleep(3)
            
            print("2. Stopping movement (command=3)...")
            stop_response = self.session.get(
                f"http://{self.ip}/form/setPTZCfg",
                params={
                    "command": "3",
                    "ZFSpeed": "0",
                    "PTSpeed": "0", 
                    "panSpeed": "8",
                    "tiltSpeed": "8"
                },
                timeout=5
            )
            
            print(f"   Status: {stop_response.status_code}")
            if stop_response.text.strip():
                print(f"   Response: {stop_response.text[:100]}")
            
            print("   🛑 Movement should have stopped!")
        
        return start_response.status_code == 200
    
    def test_all_directions(self):
        """Test all movement directions with the working format."""
        print("\n🧭 Testing All Movement Directions")
        print("=" * 50)
        
        # We need to figure out how to specify direction
        # Let's try different parameter combinations
        
        direction_tests = [
            {
                "name": "Pan Right (Speed 8)",
                "params": {"command": "0", "ZFSpeed": "0", "PTSpeed": "0", "panSpeed": "8", "tiltSpeed": "0"}
            },
            {
                "name": "Pan Left (Speed -8)", 
                "params": {"command": "0", "ZFSpeed": "0", "PTSpeed": "0", "panSpeed": "-8", "tiltSpeed": "0"}
            },
            {
                "name": "Tilt Up (Speed 8)",
                "params": {"command": "0", "ZFSpeed": "0", "PTSpeed": "0", "panSpeed": "0", "tiltSpeed": "8"}
            },
            {
                "name": "Tilt Down (Speed -8)",
                "params": {"command": "0", "ZFSpeed": "0", "PTSpeed": "0", "panSpeed": "0", "tiltSpeed": "-8"}
            },
            {
                "name": "Pan Right + Tilt Up",
                "params": {"command": "0", "ZFSpeed": "0", "PTSpeed": "0", "panSpeed": "8", "tiltSpeed": "8"}
            },
            {
                "name": "Pan Left + Tilt Down", 
                "params": {"command": "0", "ZFSpeed": "0", "PTSpeed": "0", "panSpeed": "-8", "tiltSpeed": "-8"}
            }
        ]
        
        for test in direction_tests:
            print(f"\n🎯 Testing: {test['name']}")
            
            # Start movement
            start_response = self.session.get(
                f"http://{self.ip}/form/setPTZCfg",
                params=test["params"],
                timeout=5
            )
            
            print(f"   Start Status: {start_response.status_code}")
            
            if start_response.status_code == 200:
                print("   ⏱️  Moving for 2 seconds...")
                time.sleep(2)
                
                # Stop movement
                stop_params = dict(test["params"])
                stop_params["command"] = "3"
                
                stop_response = self.session.get(
                    f"http://{self.ip}/form/setPTZCfg",
                    params=stop_params,
                    timeout=5
                )
                
                print(f"   Stop Status: {stop_response.status_code}")
                print("   🛑 Stopped")
            
            time.sleep(1)
    
    def test_different_speeds(self):
        """Test different speed values."""
        print("\n⚡ Testing Different Speed Values")
        print("=" * 50)
        
        speeds = [1, 2, 4, 8, 12, 16, 20]
        
        for speed in speeds:
            print(f"\n🎯 Testing Pan Right at speed {speed}")
            
            # Start movement
            start_response = self.session.get(
                f"http://{self.ip}/form/setPTZCfg",
                params={
                    "command": "0",
                    "ZFSpeed": "0",
                    "PTSpeed": "0", 
                    "panSpeed": str(speed),
                    "tiltSpeed": "0"
                },
                timeout=5
            )
            
            print(f"   Status: {start_response.status_code}")
            
            if start_response.status_code == 200:
                print(f"   ⏱️  Moving at speed {speed} for 1.5 seconds...")
                time.sleep(1.5)
                
                # Stop
                stop_response = self.session.get(
                    f"http://{self.ip}/form/setPTZCfg",
                    params={
                        "command": "3",
                        "ZFSpeed": "0",
                        "PTSpeed": "0",
                        "panSpeed": str(speed), 
                        "tiltSpeed": "0"
                    },
                    timeout=5
                )
                print("   🛑 Stopped")
            
            time.sleep(0.5)
    
    def generate_curl_commands(self):
        """Generate working curl commands for manual testing."""
        print("\n🔧 Working cURL Commands")
        print("=" * 50)
        print("These should make your camera move!")
        print()
        
        commands = [
            {
                "name": "Pan Right",
                "start": f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=0&ZFSpeed=0&PTSpeed=0&panSpeed=8&tiltSpeed=0"',
                "stop": f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=3&ZFSpeed=0&PTSpeed=0&panSpeed=8&tiltSpeed=0"'
            },
            {
                "name": "Pan Left", 
                "start": f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=0&ZFSpeed=0&PTSpeed=0&panSpeed=-8&tiltSpeed=0"',
                "stop": f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=3&ZFSpeed=0&PTSpeed=0&panSpeed=-8&tiltSpeed=0"'
            },
            {
                "name": "Tilt Up",
                "start": f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=0&ZFSpeed=0&PTSpeed=0&panSpeed=0&tiltSpeed=8"',
                "stop": f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=3&ZFSpeed=0&PTSpeed=0&panSpeed=0&tiltSpeed=8"'
            },
            {
                "name": "Tilt Down",
                "start": f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=0&ZFSpeed=0&PTSpeed=0&panSpeed=0&tiltSpeed=-8"', 
                "stop": f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=3&ZFSpeed=0&PTSpeed=0&panSpeed=0&tiltSpeed=-8"'
            }
        ]
        
        for cmd in commands:
            print(f"📍 {cmd['name']}:")
            print(f"   Start: {cmd['start']}")
            print(f"   Stop:  {cmd['stop']}")
            print()

def main():
    """Main test function."""
    print("🚀 Testing WORKING PTZ Commands from Browser Debug")
    print("=" * 70)
    print("Format discovered: /form/setPTZCfg?command=X&...")
    print("This should finally make the camera move!")
    print()
    
    tester = WorkingPTZTester()
    
    try:
        # Test the exact discovered format
        success = tester.test_discovered_format()
        
        if success:
            print("\n" + "="*70)
            input("✅ Press Enter to test all directions...")
            tester.test_all_directions()
            
            print("\n" + "="*70) 
            input("✅ Press Enter to test different speeds...")
            tester.test_different_speeds()
        
        # Always show curl commands
        tester.generate_curl_commands()
        
        print("\n🎯 Testing completed!")
        print("\nIf the camera moved, we found the solution! 🎉")
        print("We can now update the ROS 2 PTZ controller with this format.")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 