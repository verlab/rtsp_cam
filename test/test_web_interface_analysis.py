#!/usr/bin/env python3
"""
Web Interface Analysis for Jidetech Camera
This script helps capture the exact PTZ commands sent by the web interface
"""

import requests
import time
from requests.auth import HTTPBasicAuth, HTTPDigestAuth
import re
import json

class WebInterfaceAnalyzer:
    """Analyze the camera's web interface to find working PTZ commands."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        self.session.auth = HTTPBasicAuth(username, password)
    
    def discover_web_interface(self):
        """Discover web interface paths and PTZ control pages."""
        print("🔍 Discovering Web Interface Structure")
        print("=" * 50)
        
        # Common web interface paths
        paths = [
            "/",
            "/index.html",
            "/index.htm", 
            "/main.html",
            "/login.html",
            "/ptz.html",
            "/ptz.htm",
            "/control.html",
            "/control.htm",
            "/cgi-bin/",
            "/web/",
            "/ui/",
            "/admin/",
            "/setup/",
            "/config/"
        ]
        
        working_paths = []
        
        for path in paths:
            try:
                url = f"http://{self.ip}{path}"
                response = self.session.get(url, timeout=5)
                
                if response.status_code == 200:
                    print(f"✅ Found: {path}")
                    working_paths.append(path)
                    
                    # Look for PTZ-related content
                    content = response.text.lower()
                    if any(keyword in content for keyword in ['ptz', 'pan', 'tilt', 'zoom', 'preset']):
                        print(f"   🎯 Contains PTZ controls!")
                        
                elif response.status_code == 401:
                    print(f"🔐 Auth required: {path}")
                elif response.status_code == 404:
                    print(f"❌ Not found: {path}")
                else:
                    print(f"⚠️  Status {response.status_code}: {path}")
                    
            except Exception as e:
                print(f"❌ Error accessing {path}: {str(e)[:30]}...")
        
        return working_paths
    
    def analyze_ptz_page(self, path="/"):
        """Analyze PTZ control page for JavaScript commands."""
        print(f"\n🔍 Analyzing PTZ Controls on {path}")
        print("=" * 50)
        
        try:
            url = f"http://{self.ip}{path}"
            response = self.session.get(url, timeout=10)
            
            if response.status_code != 200:
                print(f"❌ Cannot access {path}: Status {response.status_code}")
                return
            
            content = response.text
            
            # Look for JavaScript PTZ functions
            ptz_patterns = [
                r'function\s+(\w*ptz\w*)\s*\([^)]*\)\s*{[^}]+}',
                r'function\s+(\w*pan\w*)\s*\([^)]*\)\s*{[^}]+}',
                r'function\s+(\w*tilt\w*)\s*\([^)]*\)\s*{[^}]+}',
                r'function\s+(\w*zoom\w*)\s*\([^)]*\)\s*{[^}]+}',
                r'function\s+(\w*move\w*)\s*\([^)]*\)\s*{[^}]+}',
                r'function\s+(\w*preset\w*)\s*\([^)]*\)\s*{[^}]+}'
            ]
            
            print("📋 Found PTZ JavaScript Functions:")
            for pattern in ptz_patterns:
                matches = re.findall(pattern, content, re.IGNORECASE | re.DOTALL)
                for match in matches:
                    print(f"   - {match}()")
            
            # Look for HTTP requests/AJAX calls
            print("\n📡 Found HTTP/AJAX Requests:")
            http_patterns = [
                r'(GET|POST)\s+["\']([^"\']*(?:ptz|cgi)[^"\']*)["\']',
                r'url\s*:\s*["\']([^"\']*(?:ptz|cgi)[^"\']*)["\']',
                r'open\s*\(\s*["\'](?:GET|POST)["\']\s*,\s*["\']([^"\']*)["\']',
                r'fetch\s*\(\s*["\']([^"\']*(?:ptz|cgi)[^"\']*)["\']'
            ]
            
            for pattern in http_patterns:
                matches = re.findall(pattern, content, re.IGNORECASE)
                for match in matches:
                    if isinstance(match, tuple):
                        print(f"   - {match[1] if len(match) > 1 else match[0]}")
                    else:
                        print(f"   - {match}")
            
            # Look for parameter patterns
            print("\n🔧 Found Parameter Patterns:")
            param_patterns = [
                r'["\'](?:action|cmd|command)["\']\s*:\s*["\']([^"\']+)["\']',
                r'["\'](?:code|dir|direction)["\']\s*:\s*["\']([^"\']+)["\']',
                r'["\'](?:speed|velocity)["\']\s*:\s*["\']?([^"\']+)["\']?',
                r'["\'](?:address|addr|channel)["\']\s*:\s*["\']?([^"\']+)["\']?'
            ]
            
            for pattern in param_patterns:
                matches = re.findall(pattern, content, re.IGNORECASE)
                for match in matches:
                    print(f"   - {match}")
            
            # Save full page for manual inspection
            with open(f"test/web_interface_{path.replace('/', '_')}.html", "w") as f:
                f.write(content)
            print(f"\n💾 Saved full page to: test/web_interface_{path.replace('/', '_')}.html")
            
        except Exception as e:
            print(f"❌ Error analyzing {path}: {e}")
    
    def test_discovered_commands(self):
        """Test common PTZ command patterns found in web interfaces."""
        print("\n🧪 Testing Common Web Interface PTZ Patterns")
        print("=" * 50)
        
        # Common patterns used by IP camera web interfaces
        test_commands = [
            # Standard CGI patterns
            {
                "name": "Standard PTZ CGI",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "code": "Right", "channel": "0", "arg1": "0", "arg2": "5"}
            },
            {
                "name": "Simple Direction CGI",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"cmd": "ptz", "dir": "right", "speed": "5"}
            },
            {
                "name": "Decoder Control",
                "url": f"http://{self.ip}/decoder_control.cgi",
                "params": {"command": "ptz", "onestep": "0", "aux": "right", "code": "1"}
            },
            {
                "name": "PTZ Control",
                "url": f"http://{self.ip}/ptzctrl.cgi",
                "params": {"ptzcmd": "right", "speed": "5"}
            },
            {
                "name": "Camera Control",
                "url": f"http://{self.ip}/camera-cgi/ptz.cgi",
                "params": {"action": "start", "channel": "1", "code": "Right", "arg1": "0", "arg2": "5"}
            },
            # Alternative paths
            {
                "name": "Web PTZ Control",
                "url": f"http://{self.ip}/web/ptz",
                "params": {"cmd": "right", "speed": "5"}
            },
            {
                "name": "API PTZ Control", 
                "url": f"http://{self.ip}/api/ptz",
                "params": {"direction": "right", "speed": "5"}
            }
        ]
        
        for cmd in test_commands:
            print(f"\nTesting: {cmd['name']}")
            self._test_command(cmd)
            time.sleep(1)
    
    def test_manual_curl_commands(self):
        """Provide manual curl commands for testing."""
        print("\n🔧 Manual cURL Commands to Test")
        print("=" * 50)
        print("Copy and paste these commands in another terminal:")
        print()
        
        curl_commands = [
            f'curl -u admin:admin "http://{self.ip}/cgi-bin/ptz.cgi?action=start&code=Right&channel=0&arg1=0&arg2=5"',
            f'curl -u admin:admin "http://{self.ip}/cgi-bin/ptz.cgi?cmd=ptz&dir=right&speed=5"',
            f'curl -u admin:admin "http://{self.ip}/decoder_control.cgi?command=ptz&onestep=0&aux=right&code=1"',
            f'curl -u admin:admin "http://{self.ip}/ptzctrl.cgi?ptzcmd=right&speed=5"',
            f'curl -u admin:admin "http://{self.ip}/camera-cgi/ptz.cgi?action=start&channel=1&code=Right&arg1=0&arg2=5"'
        ]
        
        for i, cmd in enumerate(curl_commands, 1):
            print(f"{i}. {cmd}")
        
        print("\nWatch the camera while running each command!")
        print("If any command makes the camera move, note which one!")
    
    def _test_command(self, cmd):
        """Test a single PTZ command."""
        try:
            response = self.session.get(cmd["url"], params=cmd["params"], timeout=5)
            
            if response.status_code == 200:
                print(f"✅ Status 200 - Response: {response.text[:50]}...")
            elif response.status_code == 404:
                print(f"❌ Status 404 - Endpoint not found")
            elif response.status_code == 401:
                print(f"🔐 Status 401 - Authentication failed")
            else:
                print(f"⚠️  Status {response.status_code}")
                
        except Exception as e:
            print(f"❌ Error: {str(e)[:30]}...")

def main():
    """Main analysis function."""
    print("🔍 Jidetech Web Interface PTZ Analyzer")
    print("=" * 60)
    print("This tool analyzes the camera's web interface to find")
    print("the exact PTZ commands used by the working web controls.")
    print()
    
    analyzer = WebInterfaceAnalyzer()
    
    try:
        # Discover web interface structure
        working_paths = analyzer.discover_web_interface()
        
        # Analyze main pages for PTZ controls
        for path in working_paths[:3]:  # Analyze first 3 working paths
            analyzer.analyze_ptz_page(path)
        
        # Test common patterns
        analyzer.test_discovered_commands()
        
        # Provide manual curl commands
        analyzer.test_manual_curl_commands()
        
        print("\n🎯 Analysis completed!")
        print("\nNext steps:")
        print("1. Check the saved HTML files for PTZ JavaScript")
        print("2. Try the manual curl commands")
        print("3. Look for working commands that make the camera move")
        print("4. Use browser developer tools to monitor network requests")
        
    except KeyboardInterrupt:
        print("\n⚠️  Analysis interrupted by user")
    except Exception as e:
        print(f"\n❌ Analysis failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 