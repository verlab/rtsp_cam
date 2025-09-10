#!/usr/bin/env python3
"""
Test HTTP CGI commands for PTZ control.
Many IP cameras use CGI commands instead of or alongside ONVIF.
"""

import requests
import time
from requests.auth import HTTPBasicAuth, HTTPDigestAuth

class HTTPCGITester:
    """Test various HTTP CGI command patterns for PTZ control."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        
    def test_cgi_patterns(self):
        """Test common CGI patterns used by IP cameras."""
        print("🔍 Testing HTTP CGI PTZ Commands")
        print("=" * 50)
        
        # Common CGI patterns for different camera brands
        patterns = [
            # Generic patterns
            {
                "name": "Generic CGI Pattern 1",
                "base_url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "0", "arg2": "0", "arg3": "0"}
            },
            {
                "name": "Generic CGI Pattern 2", 
                "base_url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"cmd": "preset", "channel": "0", "aux": "set", "data": "1"}
            },
            # Dahua patterns
            {
                "name": "Dahua Pattern 1",
                "base_url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Dahua Pattern 2",
                "base_url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "code": "Right", "arg1": "5", "arg2": "5"}
            },
            # Hikvision patterns
            {
                "name": "Hikvision Pattern 1",
                "base_url": f"http://{self.ip}/ISAPI/PTZ/channels/1/continuous",
                "method": "PUT",
                "data": '<?xml version="1.0" encoding="UTF-8"?><PTZData><pan>10</pan><tilt>0</tilt><zoom>0</zoom></PTZData>'
            },
            {
                "name": "Hikvision Pattern 2",
                "base_url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "1", "code": "Right", "arg1": "0", "arg2": "0", "arg3": "0"}
            },
            # Alternative ports and paths
            {
                "name": "Alternative Port 80",
                "base_url": f"http://{self.ip}:80/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "0", "arg2": "0", "arg3": "0"}
            },
            {
                "name": "Direct CGI",
                "base_url": f"http://{self.ip}/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right"}
            },
            # RESTful patterns
            {
                "name": "RESTful Pattern 1",
                "base_url": f"http://{self.ip}/api/ptz/move",
                "method": "POST",
                "data": '{"pan": 10, "tilt": 0, "zoom": 0}'
            },
            {
                "name": "RESTful Pattern 2",
                "base_url": f"http://{self.ip}/api/v1/ptz",
                "method": "POST", 
                "data": '{"action": "move", "direction": "right", "speed": 5}'
            }
        ]
        
        for i, pattern in enumerate(patterns, 1):
            print(f"\n{i}. Testing: {pattern['name']}")
            self._test_pattern(pattern)
            time.sleep(2)
    
    def _test_pattern(self, pattern):
        """Test a specific CGI pattern."""
        try:
            url = pattern["base_url"]
            method = pattern.get("method", "GET")
            params = pattern.get("params", {})
            data = pattern.get("data", None)
            
            # Try different authentication methods
            auth_methods = [
                ("Basic", HTTPBasicAuth(self.username, self.password)),
                ("Digest", HTTPDigestAuth(self.username, self.password)),
                ("None", None)
            ]
            
            for auth_name, auth in auth_methods:
                try:
                    if method.upper() == "GET":
                        response = requests.get(url, params=params, auth=auth, timeout=5)
                    elif method.upper() == "POST":
                        if data:
                            headers = {'Content-Type': 'application/json'}
                            response = requests.post(url, data=data, headers=headers, auth=auth, timeout=5)
                        else:
                            response = requests.post(url, params=params, auth=auth, timeout=5)
                    elif method.upper() == "PUT":
                        headers = {'Content-Type': 'application/xml'}
                        response = requests.put(url, data=data, headers=headers, auth=auth, timeout=5)
                    else:
                        continue
                    
                    print(f"   {auth_name} Auth: Status {response.status_code}")
                    
                    if response.status_code == 200:
                        print(f"   ✅ SUCCESS with {auth_name} auth!")
                        if len(response.text) < 200:
                            print(f"   Response: {response.text}")
                        else:
                            print(f"   Response: {response.text[:100]}...")
                        return True
                    elif response.status_code == 401:
                        print(f"   🔐 Authentication required")
                    elif response.status_code == 404:
                        print(f"   ❌ Endpoint not found")
                    else:
                        print(f"   ⚠️  Unexpected status: {response.status_code}")
                        if len(response.text) < 100:
                            print(f"   Response: {response.text}")
                            
                except requests.exceptions.ConnectTimeout:
                    print(f"   ⏱️  {auth_name} Auth: Timeout")
                except requests.exceptions.ConnectionError:
                    print(f"   🔌 {auth_name} Auth: Connection error")
                except Exception as e:
                    print(f"   ❌ {auth_name} Auth: {str(e)[:50]}...")
                    
        except Exception as e:
            print(f"   ❌ Pattern failed: {e}")
        
        return False
    
    def test_discovery_endpoints(self):
        """Test common discovery endpoints to understand camera API."""
        print("\n🔍 Testing Discovery Endpoints")
        print("=" * 50)
        
        endpoints = [
            "/cgi-bin/ptz.cgi?action=getStatus",
            "/cgi-bin/ptz.cgi?action=getConfig", 
            "/cgi-bin/configManager.cgi?action=getConfig&name=PTZ",
            "/ISAPI/PTZ/channels/1/status",
            "/api/ptz/status",
            "/api/v1/ptz/status",
            "/cgi-bin/hi3510/ptzctrl.cgi",
            "/web/cgi-bin/hi3510/ptzctrl.cgi",
            "/cgi-bin/CGIProxy.fcgi?cmd=getPTZSpeed",
            "/onvif/device_service",
            "/onvif/ptz_service"
        ]
        
        print("Looking for working endpoints...")
        for endpoint in endpoints:
            url = f"http://{self.ip}{endpoint}"
            try:
                response = requests.get(url, auth=HTTPBasicAuth(self.username, self.password), timeout=3)
                if response.status_code == 200:
                    print(f"✅ {endpoint} - Status: {response.status_code}")
                    if len(response.text) < 300:
                        print(f"   Response: {response.text}")
                    else:
                        print(f"   Response: {response.text[:150]}...")
                elif response.status_code != 404:
                    print(f"⚠️  {endpoint} - Status: {response.status_code}")
            except:
                pass

def main():
    """Main test function."""
    print("🚀 HTTP CGI PTZ Command Tester")
    print("=" * 60)
    print("This will test various HTTP CGI patterns to find working PTZ commands.")
    print("Watch your camera for any movement!")
    print()
    
    input("Press Enter to start testing...")
    
    tester = HTTPCGITester()
    
    try:
        # First discover available endpoints
        tester.test_discovery_endpoints()
        
        print("\n" + "="*60)
        input("Press Enter to test PTZ command patterns...")
        
        # Test PTZ command patterns
        tester.test_cgi_patterns()
        
        print("\n🎯 HTTP CGI testing completed!")
        print("\nAnalysis:")
        print("1. Look for any commands that returned HTTP 200")
        print("2. Check if any commands caused visible camera movement")
        print("3. Working commands can be integrated into the main controller")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 