#!/usr/bin/env python3
"""
Debug PTZ commands to find the correct format that actually moves the camera.
"""

import sys
import os
import time
import requests

# Add the parent directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from jidetech_camera.ptz_controller import ONVIFController, PTZStatus

class DebugPTZController:
    """Enhanced PTZ controller for debugging different command formats."""
    
    def __init__(self, ip="192.168.0.18", port=8999, username="admin", password="admin"):
        self.ip = ip
        self.port = port
        self.username = username
        self.password = password
        self.base_url = f"http://{ip}:{port}"
    
    def test_continuous_move_variations(self):
        """Test different ContinuousMove command variations."""
        print("🔍 Testing ContinuousMove Variations")
        print("=" * 50)
        
        variations = [
            {
                "name": "Current Format (Velocity)",
                "velocity_xml": '<Velocity><PanTilt x="0.5" y="0.0" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace"/><Zoom x="0.0" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace"/></Velocity>'
            },
            {
                "name": "Higher Speed Values",
                "velocity_xml": '<Velocity><PanTilt x="1.0" y="0.0" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace"/><Zoom x="0.0" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace"/></Velocity>'
            },
            {
                "name": "Without Space Attributes",
                "velocity_xml": '<Velocity><PanTilt x="0.5" y="0.0"/><Zoom x="0.0"/></Velocity>'
            },
            {
                "name": "Normalized Space",
                "velocity_xml": '<Velocity><PanTilt x="0.5" y="0.0" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpace"/><Zoom x="0.0" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/GenericSpace"/></Velocity>'
            },
            {
                "name": "With Timeout",
                "velocity_xml": '<Velocity><PanTilt x="0.5" y="0.0" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/VelocityGenericSpace"/><Zoom x="0.0" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/VelocityGenericSpace"/></Velocity><Timeout>PT2S</Timeout>'
            }
        ]
        
        for i, variation in enumerate(variations, 1):
            print(f"\n{i}. Testing: {variation['name']}")
            self._send_continuous_move(variation['velocity_xml'])
            time.sleep(3)  # Wait longer to see movement
            self._send_stop()
            time.sleep(1)
    
    def test_absolute_move(self):
        """Test AbsoluteMove commands."""
        print("\n🔍 Testing AbsoluteMove Commands")
        print("=" * 50)
        
        positions = [
            {"name": "Small Pan", "pan": "0.1", "tilt": "0.0", "zoom": "0.0"},
            {"name": "Larger Pan", "pan": "0.5", "tilt": "0.0", "zoom": "0.0"},
            {"name": "Normalized Pan", "pan": "32800", "tilt": "32768", "zoom": "2223"},  # Based on current status
        ]
        
        for pos in positions:
            print(f"\nTesting {pos['name']}: Pan={pos['pan']}, Tilt={pos['tilt']}, Zoom={pos['zoom']}")
            self._send_absolute_move(pos['pan'], pos['tilt'], pos['zoom'])
            time.sleep(3)
    
    def test_relative_move(self):
        """Test RelativeMove commands."""
        print("\n🔍 Testing RelativeMove Commands")
        print("=" * 50)
        
        moves = [
            {"name": "Small Relative Pan", "pan": "0.1", "tilt": "0.0", "zoom": "0.0"},
            {"name": "Larger Relative Pan", "pan": "0.5", "tilt": "0.0", "zoom": "0.0"},
        ]
        
        for move in moves:
            print(f"\nTesting {move['name']}: Pan={move['pan']}, Tilt={move['tilt']}, Zoom={move['zoom']}")
            self._send_relative_move(move['pan'], move['tilt'], move['zoom'])
            time.sleep(3)
    
    def _send_continuous_move(self, velocity_xml):
        """Send ContinuousMove command with custom velocity XML."""
        try:
            ptz_url = f"{self.base_url}/onvif/ptz_service"
            headers = {
                'Content-Type': 'application/soap+xml; charset=utf-8',
                'SOAPAction': '"http://www.onvif.org/ver10/ptz/wsdl/ContinuousMove"'
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
                    <ContinuousMove xmlns="http://www.onvif.org/ver10/ptz/wsdl">
                        <ProfileToken>MainStream</ProfileToken>
                        {velocity_xml}
                    </ContinuousMove>
                </soap:Body>
            </soap:Envelope>"""
            
            response = requests.post(ptz_url, data=soap_body, headers=headers, timeout=5)
            print(f"   Status: {response.status_code}")
            if response.status_code != 200:
                print(f"   Response: {response.text[:200]}...")
                
        except Exception as e:
            print(f"   Error: {e}")
    
    def _send_absolute_move(self, pan, tilt, zoom):
        """Send AbsoluteMove command."""
        try:
            ptz_url = f"{self.base_url}/onvif/ptz_service"
            headers = {
                'Content-Type': 'application/soap+xml; charset=utf-8',
                'SOAPAction': '"http://www.onvif.org/ver10/ptz/wsdl/AbsoluteMove"'
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
                    <AbsoluteMove xmlns="http://www.onvif.org/ver10/ptz/wsdl">
                        <ProfileToken>MainStream</ProfileToken>
                        <Position>
                            <PanTilt x="{pan}" y="{tilt}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
                            <Zoom x="{zoom}" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
                        </Position>
                    </AbsoluteMove>
                </soap:Body>
            </soap:Envelope>"""
            
            response = requests.post(ptz_url, data=soap_body, headers=headers, timeout=5)
            print(f"   Status: {response.status_code}")
            if response.status_code != 200:
                print(f"   Response: {response.text[:200]}...")
                
        except Exception as e:
            print(f"   Error: {e}")
    
    def _send_relative_move(self, pan, tilt, zoom):
        """Send RelativeMove command."""
        try:
            ptz_url = f"{self.base_url}/onvif/ptz_service"
            headers = {
                'Content-Type': 'application/soap+xml; charset=utf-8',
                'SOAPAction': '"http://www.onvif.org/ver10/ptz/wsdl/RelativeMove"'
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
                    <RelativeMove xmlns="http://www.onvif.org/ver10/ptz/wsdl">
                        <ProfileToken>MainStream</ProfileToken>
                        <Translation>
                            <PanTilt x="{pan}" y="{tilt}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace"/>
                            <Zoom x="{zoom}" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/TranslationGenericSpace"/>
                        </Translation>
                    </RelativeMove>
                </soap:Body>
            </soap:Envelope>"""
            
            response = requests.post(ptz_url, data=soap_body, headers=headers, timeout=5)
            print(f"   Status: {response.status_code}")
            if response.status_code != 200:
                print(f"   Response: {response.text[:200]}...")
                
        except Exception as e:
            print(f"   Error: {e}")
    
    def _send_stop(self):
        """Send Stop command."""
        try:
            ptz_url = f"{self.base_url}/onvif/ptz_service"
            headers = {
                'Content-Type': 'application/soap+xml; charset=utf-8',
                'SOAPAction': '"http://www.onvif.org/ver10/ptz/wsdl/Stop"'
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
                    <Stop xmlns="http://www.onvif.org/ver10/ptz/wsdl">
                        <ProfileToken>MainStream</ProfileToken>
                        <PanTilt>true</PanTilt>
                        <Zoom>true</Zoom>
                    </Stop>
                </soap:Body>
            </soap:Envelope>"""
            
            response = requests.post(ptz_url, data=soap_body, headers=headers, timeout=5)
            print(f"   Stop Status: {response.status_code}")
                
        except Exception as e:
            print(f"   Stop Error: {e}")

def main():
    """Main debug function."""
    print("🔧 PTZ Movement Debug Tool")
    print("=" * 60)
    print("This will test different PTZ command formats to find one that works.")
    print("Watch your camera carefully for any movement!")
    print()
    
    input("Press Enter to start testing (make sure you can see the camera)...")
    
    controller = DebugPTZController()
    
    try:
        # Test different continuous move variations
        controller.test_continuous_move_variations()
        
        print("\n" + "="*60)
        input("Press Enter to test AbsoluteMove commands...")
        
        # Test absolute move
        controller.test_absolute_move()
        
        print("\n" + "="*60)
        input("Press Enter to test RelativeMove commands...")
        
        # Test relative move
        controller.test_relative_move()
        
        print("\n🎯 Debug testing completed!")
        print("\nAnalysis:")
        print("1. Check which commands (if any) caused visible movement")
        print("2. Note any error messages or different response codes")
        print("3. The working format can be used to update the main controller")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 