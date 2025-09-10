#!/usr/bin/env python3
"""
🎯 Absolute Position Discovery for Jidetech Camera

This script tests various absolute positioning commands to find
commands that accept numerical pan/tilt coordinates (degrees/units).

Goal: Find commands like setPTZ(pan=120, tilt=45) with precise positioning.
"""

import requests
import time
import sys
from requests.auth import HTTPBasicAuth
from typing import Optional, Dict, Any, List, Tuple
import json

class AbsolutePositionTester:
    """Test absolute positioning commands with numerical coordinates"""
    
    def __init__(self, ip: str = "192.168.0.18", username: str = "admin", password: str = "admin"):
        self.ip = ip
        self.auth = HTTPBasicAuth(username, password)
        self.base_url = f"http://{ip}"
        self.timeout = 10
        
    def _test_command(self, endpoint: str, params: Dict[str, Any] = None, method: str = "GET") -> Optional[requests.Response]:
        """Test HTTP command and return response"""
        try:
            url = f"{self.base_url}{endpoint}"
            if method.upper() == "GET":
                response = requests.get(url, auth=self.auth, params=params, timeout=self.timeout)
            else:
                response = requests.post(url, auth=self.auth, data=params, timeout=self.timeout)
                
            status_icon = "✅" if response.status_code == 200 else "❌"
            print(f"{status_icon} {response.status_code} | {method} {url}")
            
            if params:
                print(f"   📋 Params: {params}")
                
            if response.text and len(response.text) < 200:
                print(f"   📄 Response: {response.text.strip()}")
                
            return response
        except Exception as e:
            print(f"❌ Error: {e}")
            return None

    def get_current_position(self) -> Optional[Dict]:
        """Try to get current absolute position"""
        print("\n🔍 TESTING POSITION QUERY COMMANDS")
        print("="*50)
        
        # Test various position query endpoints
        endpoints = [
            # Standard PTZ position queries
            ("/form/getPTZCfg", {}),
            ("/cgi-bin/ptz.cgi", {"action": "getPosition"}),
            ("/cgi-bin/ptz.cgi", {"action": "status"}),
            
            # Position status endpoints
            ("/form/getStatus", {}),
            ("/form/getPTZStatus", {}),
            ("/form/getPTZPosition", {}),
            
            # Alternative formats
            ("/ptz.cgi", {"cmd": "getPosition"}),
            ("/axis-cgi/com/ptz.cgi", {"query": "position"}),
            
            # Dahua-style
            ("/cgi-bin/ptz.cgi", {"action": "getCurrentProtocolCaps"}),
            ("/cgi-bin/ptz.cgi", {"action": "getCurrentPosition"}),
            
            # Hikvision-style  
            ("/PTZCTRL/channels/1/status", {}),
            ("/PTZCtrl/channels/1/status", {}),
        ]
        
        results = []
        for endpoint, params in endpoints:
            print(f"\n🔍 Testing: {endpoint}")
            response = self._test_command(endpoint, params)
            if response and response.status_code == 200:
                results.append({
                    "endpoint": endpoint,
                    "params": params,
                    "response": response.text
                })
                
        return results

    def test_absolute_positioning(self) -> List[Dict]:
        """Test absolute positioning commands with numerical coordinates"""
        print("\n🎯 TESTING ABSOLUTE POSITIONING COMMANDS")
        print("="*50)
        
        # Test coordinates (pan, tilt) - using various ranges
        test_positions = [
            (0, 0),      # Center
            (90, 0),     # Right
            (-90, 0),    # Left  
            (0, 45),     # Up
            (0, -45),    # Down
            (45, 30),    # Right-Up
            (180, 0),    # Far right
            (3600, 0),   # Degrees * 100
            (900, 0),    # 90 degrees * 10
        ]
        
        # Test various absolute positioning endpoints
        endpoints = [
            # Standard absolute positioning
            "/form/setPTZCfg",
            "/cgi-bin/ptz.cgi", 
            "/ptz.cgi",
            "/axis-cgi/com/ptz.cgi",
            
            # Alternative formats
            "/form/setPTZ",
            "/form/setPTZPosition", 
            "/form/movePTZ",
            
            # Dahua-style
            "/cgi-bin/ptz.cgi",
            
            # Hikvision-style
            "/PTZCtrl/channels/1/absolute",
            "/PTZCTRL/channels/1/absolute",
        ]
        
        # Parameter variations for absolute positioning
        param_formats = [
            # Standard formats
            {"action": "absoluteMove", "pan": "{pan}", "tilt": "{tilt}"},
            {"cmd": "absoluteMove", "pan": "{pan}", "tilt": "{tilt}"},
            {"command": "absoluteMove", "pan": "{pan}", "tilt": "{tilt}"},
            
            # Alternative parameter names
            {"action": "setPosition", "panPos": "{pan}", "tiltPos": "{tilt}"},
            {"cmd": "setPosition", "x": "{pan}", "y": "{tilt}"},
            {"command": "setPosition", "panPosition": "{pan}", "tiltPosition": "{tilt}"},
            
            # Numeric command formats
            {"command": "16", "pan": "{pan}", "tilt": "{tilt}"},  # Command 16 might be absolute
            {"command": "17", "pan": "{pan}", "tilt": "{tilt}"},  # Command 17 might be absolute
            {"command": "18", "pan": "{pan}", "tilt": "{tilt}"},  # Command 18 might be absolute
            
            # Speed + position
            {"action": "absoluteMove", "pan": "{pan}", "tilt": "{tilt}", "panSpeed": "50", "tiltSpeed": "50"},
            {"cmd": "goto", "pan": "{pan}", "tilt": "{tilt}", "speed": "50"},
            
            # Dahua format
            {"action": "start", "channel": "0", "code": "PositionABS", "arg1": "{pan}", "arg2": "{tilt}", "arg3": "0"},
            
            # ONVIF-style but via HTTP
            {"AbsoluteMove": "true", "Pan": "{pan}", "Tilt": "{tilt}"},
        ]
        
        successful_commands = []
        
        for endpoint in endpoints:
            for param_format in param_formats:
                for pan, tilt in test_positions[:3]:  # Test first 3 positions only
                    # Format parameters
                    params = {}
                    for key, value in param_format.items():
                        if isinstance(value, str) and "{pan}" in value:
                            params[key] = value.format(pan=pan, tilt=tilt)
                        elif isinstance(value, str) and "{tilt}" in value:
                            params[key] = value.format(pan=pan, tilt=tilt)
                        else:
                            params[key] = value
                    
                    print(f"\n🎯 Testing: {endpoint} with pan={pan}, tilt={tilt}")
                    response = self._test_command(endpoint, params)
                    
                    if response and response.status_code == 200:
                        successful_commands.append({
                            "endpoint": endpoint,
                            "params": params,
                            "pan": pan,
                            "tilt": tilt,
                            "response": response.text
                        })
                        print(f"   ✅ SUCCESS! This might work for absolute positioning")
                        
                        # Wait a bit to see if camera moves
                        time.sleep(2)
                        
                    # Don't spam too fast
                    time.sleep(0.5)
                    
        return successful_commands

    def test_position_feedback_detailed(self):
        """Test detailed position feedback to understand coordinate system"""
        print("\n📊 TESTING DETAILED POSITION FEEDBACK")
        print("="*50)
        
        # Move to known positions and check feedback
        movements = [
            ("center", {"command": 0}),  # Stop/center
            ("right", {"command": 4}),   # Move right
            ("left", {"command": 3}),    # Move left  
            ("up", {"command": 1}),      # Move up
            ("down", {"command": 2}),    # Move down
        ]
        
        for name, move_cmd in movements:
            print(f"\n🎮 Moving {name}...")
            
            # Execute movement
            if move_cmd["command"] != 0:
                self._test_command("/form/setPTZCfg", move_cmd)
                time.sleep(2)  # Move for 2 seconds
                self._test_command("/form/setPTZCfg", {"command": 0})  # Stop
                time.sleep(1)
            
            # Query position after movement
            print(f"📊 Position after {name}:")
            position_results = self.get_current_position()
            
            # Also test if we can get numeric position
            numeric_tests = [
                ("/form/getPTZCfg", {"query": "position"}),
                ("/cgi-bin/ptz.cgi", {"action": "getPos"}),
                ("/form/presetSet", {"flag": "query"}),
            ]
            
            for endpoint, params in numeric_tests:
                response = self._test_command(endpoint, params)
                if response and response.status_code == 200 and response.text:
                    print(f"   🔢 Numeric data: {response.text}")

def main():
    """Main discovery function"""
    print("🔍 Jidetech Camera - Absolute Position Discovery")
    print("="*60)
    print("Goal: Find commands for precise pan/tilt positioning with numerical coordinates")
    print("="*60)
    
    tester = AbsolutePositionTester()
    
    # Test connection
    print("🔗 Testing connection...")
    response = tester._test_command("/form/setPTZCfg", {"command": 0})
    if not response or response.status_code != 200:
        print("❌ Cannot connect to camera!")
        sys.exit(1)
    print("✅ Camera connection OK!")
    
    try:
        # Step 1: Get current position capabilities
        print("\n" + "="*60)
        print("STEP 1: DISCOVER POSITION QUERY METHODS")
        print("="*60)
        position_results = tester.get_current_position()
        
        if position_results:
            print(f"\n✅ Found {len(position_results)} working position query methods:")
            for i, result in enumerate(position_results, 1):
                print(f"   {i}. {result['endpoint']} -> {result['response'][:100]}...")
        
        # Step 2: Test absolute positioning
        print("\n" + "="*60)
        print("STEP 2: DISCOVER ABSOLUTE POSITIONING COMMANDS")  
        print("="*60)
        absolute_results = tester.test_absolute_positioning()
        
        if absolute_results:
            print(f"\n🎉 Found {len(absolute_results)} potential absolute positioning commands:")
            for i, result in enumerate(absolute_results, 1):
                print(f"   {i}. {result['endpoint']} with params {result['params']}")
        else:
            print("\n❌ No absolute positioning commands found")
            print("   Camera might only support relative movement + presets")
        
        # Step 3: Detailed position feedback
        print("\n" + "="*60)
        print("STEP 3: ANALYZE POSITION COORDINATE SYSTEM")
        print("="*60)
        tester.test_position_feedback_detailed()
        
        # Summary
        print("\n" + "="*60)
        print("🎯 DISCOVERY SUMMARY")
        print("="*60)
        
        if absolute_results:
            print("✅ FOUND ABSOLUTE POSITIONING METHODS:")
            for result in absolute_results:
                print(f"   • {result['endpoint']} with {result['params']}")
                
            print(f"\n📋 Next steps:")
            print(f"   1. Test the successful commands manually")
            print(f"   2. Verify camera actually moves to specified coordinates")
            print(f"   3. Understand the coordinate system (degrees, units, etc.)")
        else:
            print("❌ NO ABSOLUTE POSITIONING FOUND")
            print("   This camera might only support:")
            print("   • Directional movement (commands 1-4)")
            print("   • Preset-based positioning")
            print("   • Relative movement with timing")
            
        if position_results:
            print(f"\n✅ POSITION FEEDBACK AVAILABLE:")
            for result in position_results:
                print(f"   • {result['endpoint']}")
        
    except KeyboardInterrupt:
        print("\n🛑 Discovery interrupted")
        tester._test_command("/form/setPTZCfg", {"command": 0})  # Stop camera
        
    except Exception as e:
        print(f"\n❌ Discovery failed: {e}")
        tester._test_command("/form/setPTZCfg", {"command": 0})  # Stop camera

if __name__ == "__main__":
    main() 