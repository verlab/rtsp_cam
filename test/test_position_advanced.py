#!/usr/bin/env python3
"""
Advanced Position Control Test
Based on findings: preset queries work, command 15 exists, position data available
"""

import requests
import time
import re
from requests.auth import HTTPBasicAuth

class AdvancedPositionTester:
    """Advanced position control testing."""
    
    def __init__(self, ip="192.168.0.18", username="admin", password="admin"):
        self.ip = ip
        self.username = username
        self.password = password
        self.session = requests.Session()
        self.session.auth = HTTPBasicAuth(username, password)
    
    def test_preset_position_extraction(self):
        """Extract position information from presets."""
        print("🎯 Extracting Position Information from Presets")
        print("=" * 50)
        
        # Move to different positions and store as presets
        movements = [
            {"name": "Center", "commands": []},  # Current position
            {"name": "Right", "commands": [("4", 2)]},  # Move right 2 seconds
            {"name": "Left", "commands": [("3", 4)]},   # Move left 4 seconds  
            {"name": "Up", "commands": [("1", 2)]},     # Move up 2 seconds
            {"name": "Down", "commands": [("2", 4)]},   # Move down 4 seconds
        ]
        
        positions = {}
        
        for i, movement in enumerate(movements, 1):
            print(f"\n🎯 Testing position {i}: {movement['name']}")
            
            # Execute movement commands
            for cmd, duration in movement['commands']:
                print(f"   Executing command {cmd} for {duration}s...")
                self.session.get(f"http://{self.ip}/form/setPTZCfg", params={"command": cmd}, timeout=5)
                time.sleep(duration)
                self.session.get(f"http://{self.ip}/form/setPTZCfg", params={"command": "0"}, timeout=5)
                time.sleep(0.5)
            
            # Store current position as preset
            print(f"   Storing as preset {i}...")
            set_response = self.session.get(
                f"http://{self.ip}/form/presetSet?existFlag=1&flag=3&language=en&presetNum={i}",
                timeout=5
            )
            print(f"   Set Status: {set_response.status_code}")
            
            time.sleep(1)
            
            # Query preset to get position data
            print(f"   Querying preset {i} position...")
            query_response = self.session.get(
                f"http://{self.ip}/form/presetSet?flag=query&presetNum={i}",
                timeout=5
            )
            
            if query_response.status_code == 200:
                content = query_response.text.strip()
                print(f"   Response: {content}")
                
                # Try to extract numeric values
                numbers = re.findall(r'-?\d+\.?\d*', content)
                if numbers:
                    print(f"   📊 Extracted numbers: {numbers}")
                    positions[movement['name']] = {
                        'preset': i,
                        'raw_response': content,
                        'numbers': numbers
                    }
            
            time.sleep(1)
        
        # Analyze position data
        print(f"\n📊 Position Analysis:")
        print("=" * 30)
        for name, data in positions.items():
            print(f"{name}: {data['numbers']}")
        
        return positions
    
    def test_command_15_absolute_positioning(self):
        """Test command 15 for absolute positioning."""
        print("\n🎯 Testing Command 15 (Absolute Positioning)")
        print("=" * 50)
        
        # Test different parameter combinations with command 15
        test_params = [
            {"command": "15", "pan": "0", "tilt": "0"},
            {"command": "15", "pan": "100", "tilt": "0"},
            {"command": "15", "pan": "-100", "tilt": "0"},
            {"command": "15", "pan": "0", "tilt": "100"},
            {"command": "15", "pan": "0", "tilt": "-100"},
            {"command": "15", "pan": "50", "tilt": "50"},
            {"command": "15", "panPosition": "100", "tiltPosition": "50"},
            {"command": "15", "panSpeed": "100", "tiltSpeed": "50"},
            {"command": "15", "x": "100", "y": "50"},
            {"command": "15", "arg1": "100", "arg2": "50", "arg3": "0"},
        ]
        
        for i, params in enumerate(test_params, 1):
            print(f"\n🎯 Test {i}: {params}")
            
            response = self.session.get(f"http://{self.ip}/form/setPTZCfg", params=params, timeout=5)
            print(f"   Status: {response.status_code}")
            
            if response.status_code == 200:
                content = response.text.strip()
                print(f"   Response: {content[:100]}...")
                
                # Check if camera moved
                print("   ⏱️  Waiting 3 seconds to observe movement...")
                time.sleep(3)
                
                # Stop any movement
                self.session.get(f"http://{self.ip}/form/setPTZCfg", params={"command": "0"}, timeout=5)
            
            time.sleep(1)
    
    def test_position_query_variations(self):
        """Test different ways to query current position."""
        print("\n🔍 Testing Position Query Variations")
        print("=" * 50)
        
        # Test various query methods
        query_methods = [
            {"url": "/form/presetSet", "params": {"flag": "query"}},
            {"url": "/form/presetSet", "params": {"flag": "getPosition"}},
            {"url": "/form/presetSet", "params": {"flag": "status"}},
            {"url": "/form/setPTZCfg", "params": {"command": "query"}},
            {"url": "/form/setPTZCfg", "params": {"command": "getStatus"}},
            {"url": "/form/setPTZCfg", "params": {"command": "getPosition"}},
            {"url": "/form/setPTZCfg", "params": {"action": "query"}},
            {"url": "/form/setPTZCfg", "params": {"status": "1"}},
            {"url": "/form/setPTZCfg", "params": {"info": "1"}},
        ]
        
        for method in query_methods:
            try:
                print(f"\n🎯 Testing: {method['url']} with {method['params']}")
                response = self.session.get(f"http://{self.ip}{method['url']}", params=method['params'], timeout=5)
                
                if response.status_code == 200:
                    content = response.text.strip()
                    print(f"   Status: {response.status_code}")
                    print(f"   Response: {content}")
                    
                    # Look for numeric values
                    numbers = re.findall(r'-?\d+\.?\d*', content)
                    if numbers:
                        print(f"   📊 Numbers found: {numbers}")
                        
            except Exception as e:
                print(f"   ❌ Error: {str(e)[:30]}...")
    
    def test_advanced_preset_operations(self):
        """Test advanced preset operations."""
        print("\n📍 Testing Advanced Preset Operations")
        print("=" * 50)
        
        # Test different preset flags
        preset_flags = [
            {"flag": "1", "description": "Flag 1"},
            {"flag": "2", "description": "Flag 2"}, 
            {"flag": "3", "description": "Set preset"},
            {"flag": "4", "description": "Recall preset"},
            {"flag": "5", "description": "Flag 5"},
            {"flag": "query", "description": "Query preset"},
            {"flag": "list", "description": "List presets"},
            {"flag": "info", "description": "Preset info"},
            {"flag": "status", "description": "Preset status"},
        ]
        
        for flag_info in preset_flags:
            try:
                print(f"\n🎯 Testing flag: {flag_info['flag']} ({flag_info['description']})")
                response = self.session.get(
                    f"http://{self.ip}/form/presetSet",
                    params={"flag": flag_info['flag'], "presetNum": "1"},
                    timeout=5
                )
                
                if response.status_code == 200:
                    content = response.text.strip()
                    print(f"   Status: {response.status_code}")
                    print(f"   Response: {content}")
                    
            except Exception as e:
                print(f"   ❌ Error: {str(e)[:30]}...")
    
    def test_position_with_movement_tracking(self):
        """Track position changes during movement."""
        print("\n📈 Testing Position Tracking During Movement")
        print("=" * 50)
        
        # Function to get current position
        def get_position():
            try:
                response = self.session.get(
                    f"http://{self.ip}/form/presetSet?flag=query",
                    timeout=5
                )
                if response.status_code == 200:
                    numbers = re.findall(r'-?\d+\.?\d*', response.text)
                    return numbers
            except:
                pass
            return None
        
        print("🎯 Tracking position during right movement:")
        
        # Get initial position
        initial_pos = get_position()
        print(f"   Initial position: {initial_pos}")
        
        # Start movement
        print("   Starting right movement...")
        self.session.get(f"http://{self.ip}/form/setPTZCfg", params={"command": "4"}, timeout=5)
        
        # Track position during movement
        for i in range(5):
            time.sleep(1)
            current_pos = get_position()
            print(f"   Position after {i+1}s: {current_pos}")
        
        # Stop movement
        print("   Stopping movement...")
        self.session.get(f"http://{self.ip}/form/setPTZCfg", params={"command": "0"}, timeout=5)
        
        # Get final position
        time.sleep(1)
        final_pos = get_position()
        print(f"   Final position: {final_pos}")
        
        # Analyze changes
        if initial_pos and final_pos and len(initial_pos) == len(final_pos):
            print(f"   📊 Position changes:")
            for i, (init, final) in enumerate(zip(initial_pos, final_pos)):
                try:
                    change = float(final) - float(init)
                    print(f"      Value {i}: {init} → {final} (Δ{change:+.1f})")
                except:
                    print(f"      Value {i}: {init} → {final}")
    
    def generate_position_control_summary(self):
        """Generate summary of position control capabilities."""
        print("\n📋 Position Control Summary")
        print("=" * 50)
        
        print("✅ Working Methods:")
        print("   • Movement commands: 1=Up, 2=Down, 3=Left, 4=Right, 0=Stop")
        print("   • Preset storage: /form/presetSet?flag=3&presetNum=X")
        print("   • Preset recall: /form/presetSet?flag=4&presetNum=X")
        print("   • Position query: /form/presetSet?flag=query")
        
        print("\n🔬 Investigation Needed:")
        print("   • Command 15 parameters for absolute positioning")
        print("   • Position value interpretation and ranges")
        print("   • Real-time position feedback format")
        
        print("\n🛠️ Implementation Strategy:")
        print("   1. Use presets to store known positions")
        print("   2. Query preset flag to get current position")
        print("   3. Implement relative movement with position tracking")
        print("   4. Research command 15 for absolute positioning")

def main():
    """Main test function."""
    print("🚀 Advanced Position Control Tester")
    print("=" * 60)
    print("Testing advanced position control based on initial findings")
    print()
    
    tester = AdvancedPositionTester()
    
    try:
        # Test preset position extraction
        positions = tester.test_preset_position_extraction()
        
        input("\nPress Enter to test command 15 absolute positioning...")
        tester.test_command_15_absolute_positioning()
        
        input("\nPress Enter to test position query variations...")
        tester.test_position_query_variations()
        
        input("\nPress Enter to test advanced preset operations...")
        tester.test_advanced_preset_operations()
        
        input("\nPress Enter to test position tracking during movement...")
        tester.test_position_with_movement_tracking()
        
        # Generate summary
        tester.generate_position_control_summary()
        
        print("\n🎯 Advanced position testing completed!")
        
    except KeyboardInterrupt:
        print("\n⚠️  Testing interrupted by user")
    except Exception as e:
        print(f"\n❌ Testing failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 