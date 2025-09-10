#!/usr/bin/env python3
"""
Camera Interface Test Script for Jidetech Camera

This script tests various communication methods with the Jidetech camera
to determine the correct PTZ control interface and commands.
"""

import requests
import xml.etree.ElementTree as ET
import json
import time
import socket
from urllib.parse import urljoin
import argparse
import sys


class JidetechCameraInterfaceTester:
    """Test various camera interface methods."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        self.session.auth = (username, password)
    
    def _test_ping(self):
        """Test ping connectivity with fallback."""
        import subprocess
        import platform
        
        try:
            # Determine ping command based on OS
            if platform.system().lower() == "windows":
                ping_cmd = ['ping', '-n', '1', '-w', '2000', self.ip]
            else:
                ping_cmd = ['ping', '-c', '1', '-W', '2', self.ip]
            
            result = subprocess.run(ping_cmd, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print("✅ Ping successful")
                return True
            else:
                print("❌ Ping failed")
                return False
                
        except subprocess.TimeoutExpired:
            print("❌ Ping timeout")
            return False
        except FileNotFoundError:
            print("⚠️  Ping command not found")
            return False
        except Exception as e:
            print(f"❌ Ping test failed: {e}")
            return False
    
    def _test_socket_connection(self, port):
        """Test socket connection to specific port."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        try:
            result = sock.connect_ex((self.ip, port))
            return result == 0
        except Exception:
            return False
        finally:
            sock.close()
        
    def test_basic_connectivity(self):
        """Test basic network connectivity to camera."""
        print("="*60)
        print("🌐 TESTING BASIC CONNECTIVITY")
        print("="*60)
        
        # Test ping (with fallback if ping command not available)
        print(f"📡 Testing connectivity to {self.ip}...")
        ping_success = self._test_ping()
        
        if not ping_success:
            print("⚠️  Ping failed or unavailable, trying socket connection...")
            # Fallback: try socket connection to port 80
            sock_success = self._test_socket_connection(80)
            if not sock_success:
                print("❌ Cannot reach camera on any port")
                return False
            else:
                print("✅ Socket connection successful")
        
        # Test common ports
        ports_to_test = [80, 554, 8080, 8999, 443]
        print(f"\n🔌 Testing common ports...")
        
        for port in ports_to_test:
            if self._test_socket_connection(port):
                print(f"✅ Port {port}: Open")
            else:
                print(f"❌ Port {port}: Closed")
        
        return True
    
    def test_http_interface(self):
        """Test HTTP interface and discover available endpoints."""
        print("\n" + "="*60)
        print("🌐 TESTING HTTP INTERFACE")
        print("="*60)
        
        base_urls = [
            f"http://{self.ip}",
            f"http://{self.ip}:80",
            f"http://{self.ip}:8080"
        ]
        
        for base_url in base_urls:
            print(f"\n🔍 Testing {base_url}...")
            
            # Test basic HTTP connection
            try:
                response = self.session.get(base_url, timeout=5)
                print(f"✅ HTTP connection successful (Status: {response.status_code})")
                
                # Check for common camera paths
                common_paths = [
                    "/",
                    "/cgi-bin/",
                    "/cgi-bin/ptz.cgi",
                    "/cgi-bin/snapshot.cgi",
                    "/cgi-bin/configManager.cgi?action=getConfig&name=PTZ",
                    "/cgi-bin/devVideoInput.cgi?action=getCaps",
                    "/onvif/device_service",
                    "/onvif/ptz_service",
                    "/ISAPI/PTZCtrl/channels/1/continuous",
                    "/axis-cgi/com/ptz.cgi",
                    "/web/cgi-bin/hi3510/ptzctrl.cgi"
                ]
                
                for path in common_paths:
                    self.test_http_endpoint(base_url, path)
                    
            except Exception as e:
                print(f"❌ HTTP connection failed: {e}")
    
    def test_http_endpoint(self, base_url, path):
        """Test a specific HTTP endpoint."""
        try:
            url = urljoin(base_url, path)
            response = self.session.get(url, timeout=3)
            
            if response.status_code == 200:
                print(f"  ✅ {path} - OK")
                if len(response.text) < 500:  # Show short responses
                    print(f"      Response: {response.text[:200]}...")
            elif response.status_code == 401:
                print(f"  🔐 {path} - Authentication required")
            elif response.status_code == 404:
                print(f"  ❌ {path} - Not found")
            else:
                print(f"  ⚠️  {path} - Status {response.status_code}")
                
        except Exception as e:
            print(f"  ❌ {path} - Error: {e}")
    
    def test_onvif_discovery(self):
        """Test ONVIF service discovery."""
        print("\n" + "="*60)
        print("🔍 TESTING ONVIF INTERFACE")
        print("="*60)
        
        onvif_ports = [8999, 80, 8080]
        
        for port in onvif_ports:
            print(f"\n🔍 Testing ONVIF on port {port}...")
            self.test_onvif_port(port)
    
    def test_onvif_port(self, port):
        """Test ONVIF on specific port."""
        base_url = f"http://{self.ip}:{port}"
        
        # Test device service
        device_url = f"{base_url}/onvif/device_service"
        ptz_url = f"{base_url}/onvif/ptz_service"
        
        # Test GetCapabilities
        self.test_onvif_get_capabilities(device_url)
        
        # Test GetProfiles
        self.test_onvif_get_profiles(device_url)
        
        # Test PTZ service
        self.test_onvif_ptz_service(ptz_url)
    
    def test_onvif_get_capabilities(self, url):
        """Test ONVIF GetCapabilities."""
        soap_body = '''<?xml version="1.0" encoding="UTF-8"?>
        <soap:Envelope xmlns:soap="http://www.w3.org/2003/05/soap-envelope">
            <soap:Header>
                <Security xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{}</Username>
                        <Password>{}</Password>
                    </UsernameToken>
                </Security>
            </soap:Header>
            <soap:Body>
                <GetCapabilities xmlns="http://www.onvif.org/ver10/device/wsdl">
                    <Category>All</Category>
                </GetCapabilities>
            </soap:Body>
        </soap:Envelope>'''.format(self.username, self.password)
        
        headers = {
            'Content-Type': 'application/soap+xml; charset=utf-8',
            'SOAPAction': '"http://www.onvif.org/ver10/device/wsdl/GetCapabilities"'
        }
        
        try:
            response = requests.post(url, data=soap_body, headers=headers, timeout=5)
            print(f"  📋 GetCapabilities: Status {response.status_code}")
            
            if response.status_code == 200:
                print("  ✅ ONVIF Device service responding")
                # Parse capabilities
                try:
                    root = ET.fromstring(response.text)
                    # Look for PTZ capabilities
                    for elem in root.iter():
                        if 'PTZ' in elem.tag or 'ptz' in elem.tag.lower():
                            print(f"      PTZ capability found: {elem.tag}")
                except:
                    pass
            else:
                print(f"  ❌ GetCapabilities failed: {response.status_code}")
                print(f"      Response: {response.text[:200]}...")
                
        except Exception as e:
            print(f"  ❌ GetCapabilities error: {e}")
    
    def test_onvif_get_profiles(self, url):
        """Test ONVIF GetProfiles."""
        soap_body = '''<?xml version="1.0" encoding="UTF-8"?>
        <soap:Envelope xmlns:soap="http://www.w3.org/2003/05/soap-envelope">
            <soap:Header>
                <Security xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{}</Username>
                        <Password>{}</Password>
                    </UsernameToken>
                </Security>
            </soap:Header>
            <soap:Body>
                <GetProfiles xmlns="http://www.onvif.org/ver10/media/wsdl"/>
            </soap:Body>
        </soap:Envelope>'''.format(self.username, self.password)
        
        headers = {
            'Content-Type': 'application/soap+xml; charset=utf-8',
            'SOAPAction': '"http://www.onvif.org/ver10/media/wsdl/GetProfiles"'
        }
        
        try:
            # Try media service URL
            media_url = url.replace('/device_service', '/media_service')
            response = requests.post(media_url, data=soap_body, headers=headers, timeout=5)
            print(f"  📋 GetProfiles: Status {response.status_code}")
            
            if response.status_code == 200:
                print("  ✅ ONVIF Media service responding")
                # Parse profiles
                try:
                    root = ET.fromstring(response.text)
                    profile_count = 0
                    for elem in root.iter():
                        if 'Profile' in elem.tag:
                            profile_count += 1
                            if 'token' in elem.attrib:
                                print(f"      Profile token: {elem.attrib['token']}")
                    print(f"      Found {profile_count} profiles")
                except:
                    pass
            else:
                print(f"  ❌ GetProfiles failed: {response.status_code}")
                
        except Exception as e:
            print(f"  ❌ GetProfiles error: {e}")
    
    def test_onvif_ptz_service(self, url):
        """Test ONVIF PTZ service."""
        soap_body = '''<?xml version="1.0" encoding="UTF-8"?>
        <soap:Envelope xmlns:soap="http://www.w3.org/2003/05/soap-envelope">
            <soap:Header>
                <Security xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{}</Username>
                        <Password>{}</Password>
                    </UsernameToken>
                </Security>
            </soap:Header>
            <soap:Body>
                <GetStatus xmlns="http://www.onvif.org/ver20/ptz/wsdl">
                    <ProfileToken>Profile_1</ProfileToken>
                </GetStatus>
            </soap:Body>
        </soap:Envelope>'''.format(self.username, self.password)
        
        headers = {
            'Content-Type': 'application/soap+xml; charset=utf-8',
            'SOAPAction': '"http://www.onvif.org/ver20/ptz/wsdl/GetStatus"'
        }
        
        try:
            response = requests.post(url, data=soap_body, headers=headers, timeout=5)
            print(f"  📋 PTZ GetStatus: Status {response.status_code}")
            
            if response.status_code == 200:
                print("  ✅ ONVIF PTZ service responding")
                print(f"      Response: {response.text[:300]}...")
            else:
                print(f"  ❌ PTZ GetStatus failed: {response.status_code}")
                print(f"      Response: {response.text[:200]}...")
                
        except Exception as e:
            print(f"  ❌ PTZ GetStatus error: {e}")
    
    def test_cgi_ptz_commands(self):
        """Test CGI-based PTZ commands."""
        print("\n" + "="*60)
        print("🎮 TESTING CGI PTZ COMMANDS")
        print("="*60)
        
        # Common CGI PTZ patterns
        ptz_patterns = [
            # Dahua/Hikvision style
            {
                "name": "Dahua Style",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Up", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Hikvision ISAPI",
                "url": f"http://{self.ip}/ISAPI/PTZCtrl/channels/1/continuous",
                "method": "PUT",
                "data": '<?xml version="1.0" encoding="UTF-8"?><PTZData><pan>10</pan><tilt>10</tilt></PTZData>'
            },
            # Generic CGI patterns
            {
                "name": "Generic PTZ 1",
                "url": f"http://{self.ip}/cgi-bin/ptz.cgi",
                "params": {"pan": "10", "tilt": "10", "zoom": "0"}
            },
            {
                "name": "Generic PTZ 2", 
                "url": f"http://{self.ip}/cgi-bin/hi3510/ptzctrl.cgi",
                "params": {"pan": "10", "tilt": "10"}
            },
            # Axis style
            {
                "name": "Axis Style",
                "url": f"http://{self.ip}/axis-cgi/com/ptz.cgi",
                "params": {"rpan": "10", "rtilt": "10"}
            }
        ]
        
        for pattern in ptz_patterns:
            print(f"\n🔍 Testing {pattern['name']}...")
            self.test_ptz_pattern(pattern)
    
    def test_ptz_pattern(self, pattern):
        """Test a specific PTZ command pattern."""
        try:
            url = pattern["url"]
            method = pattern.get("method", "GET")
            
            if method == "GET":
                params = pattern.get("params", {})
                response = self.session.get(url, params=params, timeout=5)
            else:
                data = pattern.get("data", "")
                headers = {"Content-Type": "application/xml"}
                response = self.session.put(url, data=data, headers=headers, timeout=5)
            
            print(f"  📡 {method} {url}")
            print(f"  📊 Status: {response.status_code}")
            
            if response.status_code == 200:
                print("  ✅ Command accepted")
                if response.text:
                    print(f"      Response: {response.text[:100]}...")
            elif response.status_code == 401:
                print("  🔐 Authentication required")
            elif response.status_code == 404:
                print("  ❌ Endpoint not found")
            else:
                print(f"  ⚠️  Unexpected status: {response.status_code}")
                print(f"      Response: {response.text[:100]}...")
                
        except Exception as e:
            print(f"  ❌ Error: {e}")
    
    def test_camera_info_discovery(self):
        """Discover camera model and capabilities."""
        print("\n" + "="*60)
        print("📷 CAMERA INFO DISCOVERY")
        print("="*60)
        
        # Try to get device info
        info_urls = [
            f"http://{self.ip}/cgi-bin/magicBox.cgi?action=getDeviceType",
            f"http://{self.ip}/cgi-bin/magicBox.cgi?action=getMachineName",
            f"http://{self.ip}/cgi-bin/configManager.cgi?action=getConfig&name=General",
            f"http://{self.ip}/cgi-bin/devVideoInput.cgi?action=getCaps",
            f"http://{self.ip}/ISAPI/System/deviceInfo",
            f"http://{self.ip}/axis-cgi/param.cgi?action=list&group=Brand"
        ]
        
        for url in info_urls:
            print(f"\n🔍 Trying {url}...")
            try:
                response = self.session.get(url, timeout=5)
                if response.status_code == 200:
                    print(f"  ✅ Success!")
                    print(f"      Response: {response.text[:300]}...")
                else:
                    print(f"  ❌ Status: {response.status_code}")
            except Exception as e:
                print(f"  ❌ Error: {e}")
    
    def generate_test_report(self):
        """Generate a summary test report."""
        print("\n" + "="*60)
        print("📋 TEST SUMMARY & RECOMMENDATIONS")
        print("="*60)
        
        print("\n🔧 RECOMMENDED NEXT STEPS:")
        print("1. Check which HTTP endpoints returned status 200")
        print("2. Try the working PTZ command patterns in your code")
        print("3. If ONVIF failed, use HTTP CGI commands instead")
        print("4. Update the PTZ controller to use working interface")
        
        print("\n💡 COMMON FIXES:")
        print("- Set 'use_onvif: false' in config if ONVIF failed")
        print("- Update PTZ URLs in ptz_controller.py based on working endpoints")
        print("- Adjust authentication method if needed")
        print("- Check camera manual for specific PTZ command format")
    
    def run_all_tests(self):
        """Run all interface tests."""
        print("🚀 STARTING COMPREHENSIVE CAMERA INTERFACE TEST")
        print(f"📍 Camera IP: {self.ip}")
        print(f"👤 Username: {self.username}")
        print(f"🔑 Password: {self.password}")
        
        # Run all tests
        self.test_basic_connectivity()
        self.test_http_interface()
        self.test_onvif_discovery()
        self.test_cgi_ptz_commands()
        self.test_camera_info_discovery()
        self.generate_test_report()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Jidetech Camera Interface Tester')
    parser.add_argument('--ip', default='192.168.0.18', help='Camera IP address')
    parser.add_argument('--username', default='admin', help='Camera username')
    parser.add_argument('--password', default='admin', help='Camera password')
    parser.add_argument('--test', choices=['connectivity', 'http', 'onvif', 'cgi', 'info', 'all'], 
                       default='all', help='Test type to run')
    
    args = parser.parse_args()
    
    # Create tester
    tester = JidetechCameraInterfaceTester(args.ip, args.username, args.password)
    
    # Run selected tests
    if args.test == 'connectivity':
        tester.test_basic_connectivity()
    elif args.test == 'http':
        tester.test_http_interface()
    elif args.test == 'onvif':
        tester.test_onvif_discovery()
    elif args.test == 'cgi':
        tester.test_cgi_ptz_commands()
    elif args.test == 'info':
        tester.test_camera_info_discovery()
    elif args.test == 'all':
        tester.run_all_tests()


if __name__ == '__main__':
    main() 