#!/usr/bin/env python3
"""
Jidetech P1-$X-2mp Specific PTZ Test Script
Based on ONVIF 2.4 with Hikvision, Dahua, XM, Amcrest compatibility
"""

import requests
import time
import xml.etree.ElementTree as ET
from requests.auth import HTTPBasicAuth, HTTPDigestAuth

class JidetechPTZTester:
    """Specific tester for Jidetech P1-$X-2mp camera."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.base_url = f"http://{ip}"
        
    def test_onvif_capabilities(self):
        """Get detailed ONVIF capabilities to understand supported commands."""
        print("🔍 Getting ONVIF Capabilities")
        print("=" * 50)
        
        try:
            # Get PTZ capabilities
            url = f"{self.base_url}:8999/onvif/ptz_service"
            headers = {
                'Content-Type': 'application/soap+xml; charset=utf-8',
                'SOAPAction': '"http://www.onvif.org/ver20/ptz/wsdl/GetNodes"'
            }
            
            soap_body = f"""<?xml version="1.0" encoding="UTF-8"?>
            <soap:Envelope xmlns:soap="http://www.w3.org/2003/05/soap-envelope">
                <soap:Header>
                    <Security xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                        <UsernameToken>
                            <Username>{self.username}</Username>
                            <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">
                                {self.password}
                            </Password>
                        </UsernameToken>
                    </Security>
                </soap:Header>
                <soap:Body>
                    <GetNodes xmlns="http://www.onvif.org/ver20/ptz/wsdl"/>
                </soap:Body>
            </soap:Envelope>"""
            
            response = requests.post(url, data=soap_body, headers=headers, timeout=5)
            print(f"GetNodes Status: {response.status_code}")
            if response.status_code == 200:
                print("PTZ Nodes Response:")
                print(response.text[:500] + "..." if len(response.text) > 500 else response.text)
                
        except Exception as e:
            print(f"Error getting capabilities: {e}")
    
    def test_hikvision_compatible_commands(self):
        """Test Hikvision-compatible PTZ commands."""
        print("\n🔍 Testing Hikvision-Compatible Commands")
        print("=" * 50)
        
        # Hikvision ONVIF commands
        commands = [
            {
                "name": "Hikvision ContinuousMove",
                "action": "ContinuousMove",
                "namespace": "http://www.onvif.org/ver20/ptz/wsdl",
                "velocity": '<Velocity><PanTilt x="0.5" y="0.0"/><Zoom x="0.0"/></Velocity>'
            },
            {
                "name": "Hikvision ContinuousMove with Timeout",
                "action": "ContinuousMove", 
                "namespace": "http://www.onvif.org/ver20/ptz/wsdl",
                "velocity": '<Velocity><PanTilt x="0.5" y="0.0"/><Zoom x="0.0"/></Velocity><Timeout>PT3S</Timeout>'
            }
        ]
        
        for cmd in commands:
            print(f"\nTesting: {cmd['name']}")
            self._send_onvif_command(cmd)
            time.sleep(3)
            self._send_stop_command()
            time.sleep(1)
    
    def test_dahua_compatible_commands(self):
        """Test Dahua-compatible PTZ commands."""
        print("\n🔍 Testing Dahua-Compatible Commands")
        print("=" * 50)
        
        # Dahua CGI commands
        cgi_commands = [
            {
                "name": "Dahua CGI Right",
                "url": f"{self.base_url}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Dahua CGI Left", 
                "url": f"{self.base_url}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Left", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Dahua CGI Up",
                "url": f"{self.base_url}/cgi-bin/ptz.cgi", 
                "params": {"action": "start", "channel": "0", "code": "Up", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Dahua CGI Down",
                "url": f"{self.base_url}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Down", "arg1": "0", "arg2": "5", "arg3": "0"}
            }
        ]
        
        for cmd in cgi_commands:
            print(f"\nTesting: {cmd['name']}")
            self._send_cgi_command(cmd)
            time.sleep(2)
            # Send stop command
            stop_cmd = {
                "url": f"{self.base_url}/cgi-bin/ptz.cgi",
                "params": {"action": "stop", "channel": "0", "code": cmd['params']['code']}
            }
            self._send_cgi_command(stop_cmd, is_stop=True)
            time.sleep(1)
    
    def test_xm_amcrest_commands(self):
        """Test XM/Amcrest compatible commands."""
        print("\n🔍 Testing XM/Amcrest-Compatible Commands")
        print("=" * 50)
        
        # XM/Amcrest style commands
        commands = [
            {
                "name": "XM Style Right",
                "url": f"{self.base_url}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "code": "Right", "arg1": "5", "arg2": "5"}
            },
            {
                "name": "Amcrest Style Right",
                "url": f"{self.base_url}/cgi-bin/ptz.cgi", 
                "params": {"action": "start", "channel": "1", "code": "Right", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Alternative XM Right",
                "url": f"{self.base_url}/cgi-bin/hi3510/ptzctrl.cgi",
                "params": {"action": "start", "code": "Right", "arg1": "0", "arg2": "5"}
            }
        ]
        
        for cmd in commands:
            print(f"\nTesting: {cmd['name']}")
            self._send_cgi_command(cmd)
            time.sleep(2)
    
    def test_preset_commands(self):
        """Test preset position commands."""
        print("\n🔍 Testing Preset Commands")
        print("=" * 50)
        
        # Try to set and go to preset positions
        preset_commands = [
            {
                "name": "Set Preset 1",
                "url": f"{self.base_url}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "SetPreset", "arg1": "0", "arg2": "1", "arg3": "0"}
            },
            {
                "name": "Go to Preset 1", 
                "url": f"{self.base_url}/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "GotoPreset", "arg1": "0", "arg2": "1", "arg3": "0"}
            },
            {
                "name": "ONVIF Go to Preset",
                "action": "GotoPreset",
                "namespace": "http://www.onvif.org/ver20/ptz/wsdl",
                "preset": "1"
            }
        ]
        
        for cmd in preset_commands:
            print(f"\nTesting: {cmd['name']}")
            if "action" in cmd:  # ONVIF command
                self._send_preset_command(cmd)
            else:  # CGI command
                self._send_cgi_command(cmd)
            time.sleep(2)
    
    def test_alternative_ports_and_paths(self):
        """Test alternative ports and paths."""
        print("\n🔍 Testing Alternative Ports and Paths")
        print("=" * 50)
        
        alternatives = [
            {
                "name": "Port 80 CGI",
                "url": f"{self.base_url}:80/cgi-bin/ptz.cgi",
                "params": {"action": "start", "channel": "0", "code": "Right", "arg1": "0", "arg2": "5", "arg3": "0"}
            },
            {
                "name": "Direct PTZ CGI",
                "url": f"{self.base_url}/ptz.cgi",
                "params": {"action": "start", "code": "Right", "arg1": "5"}
            },
            {
                "name": "Web CGI Path",
                "url": f"{self.base_url}/web/cgi-bin/hi3510/ptzctrl.cgi",
                "params": {"action": "start", "code": "Right", "arg1": "5"}
            }
        ]
        
        for cmd in alternatives:
            print(f"\nTesting: {cmd['name']}")
            self._send_cgi_command(cmd)
            time.sleep(2)
    
    def _send_onvif_command(self, cmd):
        """Send ONVIF command."""
        try:
            url = f"{self.base_url}:8999/onvif/ptz_service"
            headers = {
                'Content-Type': 'application/soap+xml; charset=utf-8',
                'SOAPAction': f'"{cmd["namespace"]}/{cmd["action"]}"'
            }
            
            soap_body = f"""<?xml version="1.0" encoding="UTF-8"?>
            <soap:Envelope xmlns:soap="http://www.w3.org/2003/05/soap-envelope">
                <soap:Header>
                    <Security xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                        <UsernameToken>
                            <Username>{self.username}</Username>
                            <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">
                                {self.password}
                            </Password>
                        </UsernameToken>
                    </Security>
                </soap:Header>
                <soap:Body>
                    <{cmd["action"]} xmlns="{cmd["namespace"]}">
                        <ProfileToken>MainStream</ProfileToken>
                        {cmd["velocity"]}
                    </{cmd["action"]}>
                </soap:Body>
            </soap:Envelope>"""
            
            response = requests.post(url, data=soap_body, headers=headers, timeout=5)
            print(f"   Status: {response.status_code}")
            if response.status_code != 200:
                print(f"   Response: {response.text[:200]}...")
                
        except Exception as e:
            print(f"   Error: {e}")
    
    def _send_preset_command(self, cmd):
        """Send ONVIF preset command."""
        try:
            url = f"{self.base_url}:8999/onvif/ptz_service"
            headers = {
                'Content-Type': 'application/soap+xml; charset=utf-8',
                'SOAPAction': f'"{cmd["namespace"]}/{cmd["action"]}"'
            }
            
            soap_body = f"""<?xml version="1.0" encoding="UTF-8"?>
            <soap:Envelope xmlns:soap="http://www.w3.org/2003/05/soap-envelope">
                <soap:Header>
                    <Security xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                        <UsernameToken>
                            <Username>{self.username}</Username>
                            <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">
                                {self.password}
                            </Password>
                        </UsernameToken>
                    </Security>
                </soap:Header>
                <soap:Body>
                    <{cmd["action"]} xmlns="{cmd["namespace"]}">
                        <ProfileToken>MainStream</ProfileToken>
                        <PresetToken>{cmd["preset"]}</PresetToken>
                    </{cmd["action"]}>
                </soap:Body>
            </soap:Envelope>"""
            
            response = requests.post(url, data=soap_body, headers=headers, timeout=5)
            print(f"   Status: {response.status_code}")
            if response.status_code != 200:
                print(f"   Response: {response.text[:200]}...")
                
        except Exception as e:
            print(f"   Error: {e}")
    
    def _send_cgi_command(self, cmd, is_stop=False):
        """Send CGI command."""
        try:
            auth_methods = [
                HTTPBasicAuth(self.username, self.password),
                HTTPDigestAuth(self.username, self.password),
                None
            ]
            
            for auth in auth_methods:
                try:
                    response = requests.get(cmd["url"], params=cmd["params"], auth=auth, timeout=5)
                    if response.status_code == 200:
                        print(f"   ✅ SUCCESS: Status {response.status_code}")
                        if len(response.text) < 100:
                            print(f"   Response: {response.text}")
                        return True
                    elif response.status_code == 401:
                        continue  # Try next auth method
                    else:
                        print(f"   Status: {response.status_code}")
                        
                except Exception as e:
                    continue
                    
            print(f"   ❌ All auth methods failed")
            return False
            
        except Exception as e:
            print(f"   Error: {e}")
            return False
    
    def _send_stop_command(self):
        """Send ONVIF stop command."""
        try:
            url = f"{self.base_url}:8999/onvif/ptz_service"
            headers = {
                'Content-Type': 'application/soap+xml; charset=utf-8',
                'SOAPAction': '"http://www.onvif.org/ver20/ptz/wsdl/Stop"'
            }
            
            soap_body = f"""<?xml version="1.0" encoding="UTF-8"?>
            <soap:Envelope xmlns:soap="http://www.w3.org/2003/05/soap-envelope">
                <soap:Header>
                    <Security xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                        <UsernameToken>
                            <Username>{self.username}</Username>
                            <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">
                                {self.password}
                            </Password>
                        </UsernameToken>
                    </Security>
                </soap:Header>
                <soap:Body>
                    <Stop xmlns="http://www.onvif.org/ver20/ptz/wsdl">
                        <ProfileToken>MainStream</ProfileToken>
                        <PanTilt>true</PanTilt>
                        <Zoom>true</Zoom>
                    </Stop>
                </soap:Body>
            </soap:Envelope>"""
            
            response = requests.post(url, data=soap_body, headers=headers, timeout=5)
            print(f"   Stop: {response.status_code}")
            
        except Exception as e:
            print(f"   Stop Error: {e}")

def main():
    """Main test function."""
    print("🚀 Jidetech P1-$X-2mp PTZ Tester")
    print("=" * 60)
    print("Model: Jidetech P1-$X-2mp")
    print("ONVIF: 2.4 (Hikvision, Dahua, XM, Amcrest compatible)")
    print()
    print("Watch your camera carefully for movement!")
    print()
    
    input("Press Enter to start testing...")
    
    tester = JidetechPTZTester()
    
    try:
        # Get capabilities first
        tester.test_onvif_capabilities()
        
        print("\n" + "="*60)
        input("Press Enter to test Hikvision-compatible commands...")
        tester.test_hikvision_compatible_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test Dahua-compatible commands...")
        tester.test_dahua_compatible_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test XM/Amcrest commands...")
        tester.test_xm_amcrest_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test preset commands...")
        tester.test_preset_commands()
        
        print("\n" + "="*60)
        input("Press Enter to test alternative paths...")
        tester.test_alternative_ports_and_paths()
        
        print("\n🎯 Jidetech-specific testing completed!")
        print("\nLook for any commands that caused visible movement or returned success responses.")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 