#!/usr/bin/env python3
"""
Test Position Control and Feedback for Jidetech Camera
Discover capabilities for:
1. Getting current pan/tilt position
2. Setting absolute position
3. Position feedback
"""

import requests
import time
import json
import re
from requests.auth import HTTPBasicAuth

class PositionControlTester:
    """Test position control and feedback capabilities."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        self.session.auth = HTTPBasicAuth(username, password)
    
    def test_position_query_endpoints(self):
        """Test various endpoints that might provide position information."""
        print("🔍 Testing Position Query Endpoints")
        print("=" * 50)
        
        # Common position query endpoints
        endpoints = [
            "/form/getPTZCfg",
            "/cgi-bin/ptz.cgi?action=query",
            "/cgi-bin/ptz.cgi?action=getStatus", 
            "/form/getPTZStatus",
            "/form/getPTZPosition",
            "/cgi-bin/configManager.cgi?action=getConfig&name=PTZ",
            "/cgi-bin/magicBox.cgi?action=getDeviceType",
            "/cgi-bin/encode.cgi?action=getConfigCaps",
            "/form/getStatus",
            "/api/ptz/status",
            "/web/ptz/status",
            "/status.cgi",
            "/form/getDeviceInfo",
            "/cgi-bin/deviceInfo.cgi"
        ]
        
        working_endpoints = []
        
        for endpoint in endpoints:
            try:
                print(f"\n🎯 Testing: {endpoint}")
                response = self.session.get(f"http://{self.ip}{endpoint}", timeout=5)
                
                print(f"   Status: {response.status_code}")
                
                if response.status_code == 200:
                    content = response.text.strip()
                    if content and len(content) < 500:  # Show short responses
                        print(f"   Response: {content}")
                    elif content:
                        print(f"   Response: {content[:100]}... (truncated)")
                        
                        # Look for position-related keywords
                        keywords = ['pan', 'tilt', 'zoom', 'position', 'angle', 'degree', 'x=', 'y=', 'z=']
                        found_keywords = [kw for kw in keywords if kw.lower() in content.lower()]
                        if found_keywords:
                            print(f"   🎯 Found keywords: {found_keywords}")
                            working_endpoints.append(endpoint)
                    
                    # Try to parse as JSON
                    try:
                        json_data = json.loads(content)
                        print(f"   📋 JSON structure: {list(json_data.keys()) if isinstance(json_data, dict) else type(json_data)}")
                        working_endpoints.append(endpoint)
                    except:
                        pass
                        
            except Exception as e:
                print(f"   ❌ Error: {str(e)[:30]}...")
        
        return working_endpoints
    
    def test_position_with_parameters(self):
        """Test position queries with various parameters."""
        print("\n🔧 Testing Position Queries with Parameters")
        print("=" * 50)
        
        base_endpoints = ["/form/getPTZCfg", "/cgi-bin/ptz.cgi"]
        
        param_combinations = [
            {"action": "getStatus"},
            {"action": "query"},
            {"action": "getPosition"},
            {"command": "getStatus"},
            {"command": "query"},
            {"cmd": "getStatus"},
            {"cmd": "query"},
            {"channel": "0"},
            {"channel": "1"},
            {"stream": "0"},
            {"stream": "1"},
            {"preset": "query"},
            {"status": "1"},
            {"info": "1"}
        ]
        
        for endpoint in base_endpoints:
            for params in param_combinations:
                try:
                    print(f"\n🎯 Testing: {endpoint} with {params}")
                    response = self.session.get(f"http://{self.ip}{endpoint}", params=params, timeout=5)
                    
                    if response.status_code == 200:
                        content = response.text.strip()
                        if content and 'error' not in content.lower():
                            print(f"   ✅ Status: {response.status_code}")
                            print(f"   Response: {content[:100]}...")
                            
                            # Look for numeric values that might be positions
                            numbers = re.findall(r'-?\d+\.?\d*', content)
                            if len(numbers) >= 2:
                                print(f"   🔢 Found numbers: {numbers[:5]}...")
                        
                except Exception as e:
                    pass  # Skip errors for brevity
    
    def test_absolute_position_commands(self):
        """Test setting absolute positions."""
        print("\n🎯 Testing Absolute Position Commands")
        print("=" * 50)
        
        # Test different absolute position formats
        position_formats = [
            {
                "name": "setPTZCfg with pan/tilt values",
                "endpoint": "/form/setPTZCfg",
                "params": {"command": "setPosition", "pan": "100", "tilt": "50"}
            },
            {
                "name": "setPTZCfg with absolute command",
                "endpoint": "/form/setPTZCfg", 
                "params": {"command": "15", "pan": "100", "tilt": "50"}  # Command 15 might be absolute
            },
            {
                "name": "PTZ CGI absolute",
                "endpoint": "/cgi-bin/ptz.cgi",
                "params": {"action": "start", "code": "PositionABS", "arg1": "100", "arg2": "50", "arg3": "0"}
            },
            {
                "name": "PTZ CGI goto",
                "endpoint": "/cgi-bin/ptz.cgi",
                "params": {"action": "start", "code": "Goto", "arg1": "100", "arg2": "50", "arg3": "0"}
            },
            {
                "name": "Direct position setting",
                "endpoint": "/form/setPTZPosition",
                "params": {"pan": "100", "tilt": "50", "zoom": "50"}
            },
            {
                "name": "Absolute move command",
                "endpoint": "/form/setPTZCfg",
                "params": {"command": "absoluteMove", "panPosition": "100", "tiltPosition": "50"}
            }
        ]
        
        for fmt in position_formats:
            try:
                print(f"\n🎯 Testing: {fmt['name']}")
                response = self.session.get(f"http://{self.ip}{fmt['endpoint']}", params=fmt['params'], timeout=5)
                
                print(f"   Status: {response.status_code}")
                if response.status_code == 200:
                    print(f"   Response: {response.text[:100]}...")
                    print("   ⏱️  Waiting 3 seconds to see if camera moves...")
                    time.sleep(3)
                    
            except Exception as e:
                print(f"   ❌ Error: {str(e)[:30]}...")
    
    def test_preset_position_info(self):
        """Test getting position info from presets."""
        print("\n📍 Testing Preset Position Information")
        print("=" * 50)
        
        # First, try to get preset information
        preset_endpoints = [
            "/form/getPresetInfo",
            "/form/getPresetList", 
            "/form/presetQuery",
            "/cgi-bin/ptz.cgi?action=getPresets",
            "/form/presetSet?flag=query",
            "/form/presetSet?flag=list"
        ]
        
        for endpoint in preset_endpoints:
            try:
                print(f"\n🎯 Testing: {endpoint}")
                response = self.session.get(f"http://{self.ip}{endpoint}", timeout=5)
                
                if response.status_code == 200:
                    content = response.text.strip()
                    print(f"   ✅ Status: {response.status_code}")
                    print(f"   Response: {content[:200]}...")
                    
            except Exception as e:
                print(f"   ❌ Error: {str(e)[:30]}...")
        
        # Test setting and querying a preset to see position info
        print(f"\n🎯 Testing preset position workflow:")
        
        try:
            # Set a preset at current position
            print("   1. Setting preset 1 at current position...")
            set_response = self.session.get(
                f"http://{self.ip}/form/presetSet?existFlag=1&flag=3&language=en&presetNum=1", 
                timeout=5
            )
            print(f"      Set Status: {set_response.status_code}")
            
            time.sleep(1)
            
            # Try to query preset info
            print("   2. Querying preset 1 info...")
            query_response = self.session.get(
                f"http://{self.ip}/form/presetSet?flag=query&presetNum=1",
                timeout=5
            )
            print(f"      Query Status: {query_response.status_code}")
            if query_response.status_code == 200:
                print(f"      Response: {query_response.text}")
                
        except Exception as e:
            print(f"   ❌ Error: {str(e)}")
    
    def test_position_range_discovery(self):
        """Test to discover position ranges and limits."""
        print("\n📏 Testing Position Range Discovery")
        print("=" * 50)
        
        # Try to discover position limits
        limit_tests = [
            {"name": "Min Pan", "params": {"command": "3", "duration": "5000"}},  # Move left for 5 seconds
            {"name": "Max Pan", "params": {"command": "4", "duration": "5000"}},  # Move right for 5 seconds
            {"name": "Min Tilt", "params": {"command": "2", "duration": "3000"}}, # Move down for 3 seconds
            {"name": "Max Tilt", "params": {"command": "1", "duration": "3000"}}, # Move up for 3 seconds
        ]
        
        print("⚠️  This test will move the camera to limits to discover range.")
        print("Make sure the camera can move freely!")
        
        proceed = input("Proceed with limit discovery? (y/n): ")
        if proceed.lower() != 'y':
            return
        
        for test in limit_tests:
            print(f"\n🎯 Testing: {test['name']}")
            
            # Move to limit
            print("   Moving to limit...")
            self.session.get(f"http://{self.ip}/form/setPTZCfg", params=test['params'], timeout=10)
            
            # Stop
            time.sleep(1)
            self.session.get(f"http://{self.ip}/form/setPTZCfg", params={"command": "0"}, timeout=5)
            
            # Try to get position
            print("   Trying to get position at limit...")
            # Add position query attempts here
            
            time.sleep(2)
    
    def generate_position_curl_commands(self):
        """Generate curl commands for position testing."""
        print("\n🔧 Position Control cURL Commands")
        print("=" * 50)
        
        print("# Query position:")
        print(f'curl -u admin:admin "http://{self.ip}/form/getPTZCfg"')
        print(f'curl -u admin:admin "http://{self.ip}/cgi-bin/ptz.cgi?action=getStatus"')
        
        print("\n# Set absolute position (examples):")
        print(f'curl -u admin:admin "http://{self.ip}/form/setPTZCfg?command=setPosition&pan=100&tilt=50"')
        print(f'curl -u admin:admin "http://{self.ip}/cgi-bin/ptz.cgi?action=start&code=PositionABS&arg1=100&arg2=50&arg3=0"')
        
        print("\n# Preset position workflow:")
        print(f'curl -u admin:admin "http://{self.ip}/form/presetSet?existFlag=1&flag=3&language=en&presetNum=1"  # Set preset')
        print(f'curl -u admin:admin "http://{self.ip}/form/presetSet?flag=query&presetNum=1"  # Query preset')

def main():
    """Main test function."""
    print("🎯 Jidetech Camera Position Control Tester")
    print("=" * 60)
    print("Testing position control and feedback capabilities")
    print()
    
    tester = PositionControlTester()
    
    try:
        # Test position query endpoints
        working_endpoints = tester.test_position_query_endpoints()
        
        input("\nPress Enter to test position queries with parameters...")
        tester.test_position_with_parameters()
        
        input("\nPress Enter to test absolute position commands...")
        tester.test_absolute_position_commands()
        
        input("\nPress Enter to test preset position info...")
        tester.test_preset_position_info()
        
        input("\nPress Enter to test position range discovery...")
        tester.test_position_range_discovery()
        
        # Show curl commands
        tester.generate_position_curl_commands()
        
        print("\n🎯 Position control testing completed!")
        print("\nKey findings to look for:")
        print("1. Endpoints that return numeric position values")
        print("2. Commands that accept absolute position parameters")
        print("3. Position ranges and limits")
        print("4. Preset position information")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 