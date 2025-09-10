#!/usr/bin/env python3
"""
Test script to verify PTZ fixes work with corrected ONVIF implementation.
"""

import sys
import os
import time

# Add the parent directory to Python path to import jidetech_camera
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from jidetech_camera.ptz_controller import ONVIFController, HTTPController, PTZStatus

def test_onvif_ptz():
    """Test ONVIF PTZ controller with corrected implementation."""
    print("🔧 Testing ONVIF PTZ Controller (Fixed Version)")
    print("=" * 60)
    
    # Initialize controller with corrected settings
    controller = ONVIFController(
        ip="192.168.0.18",
        port=8999,
        username="admin", 
        password="admin",
        timeout_ms=5000
    )
    
    print("\n1. Testing PTZ Status...")
    status = controller.get_ptz_status()
    print(f"   Current Status: Pan={status.pan_position:.2f}, Tilt={status.tilt_position:.2f}, Zoom={status.zoom_position:.2f}")
    
    print("\n2. Testing PTZ Movement Commands...")
    
    # Test small pan movement
    print("   → Testing Pan Right (0.1)...")
    controller.move_ptz(pan=0.1, tilt=0.0, zoom=0.0)
    time.sleep(2)
    
    # Stop movement
    print("   → Stopping PTZ...")
    controller.stop_ptz()
    time.sleep(1)
    
    # Test small tilt movement  
    print("   → Testing Tilt Up (0.1)...")
    controller.move_ptz(pan=0.0, tilt=0.1, zoom=0.0)
    time.sleep(2)
    
    # Stop movement
    print("   → Stopping PTZ...")
    controller.stop_ptz()
    time.sleep(1)
    
    # Test zoom
    print("   → Testing Zoom In (0.1)...")
    controller.move_ptz(pan=0.0, tilt=0.0, zoom=0.1)
    time.sleep(2)
    
    # Stop movement
    print("   → Stopping PTZ...")
    controller.stop_ptz()
    
    print("\n3. Final Status Check...")
    status = controller.get_ptz_status()
    print(f"   Final Status: Pan={status.pan_position:.2f}, Tilt={status.tilt_position:.2f}, Zoom={status.zoom_position:.2f}")
    
    print("\n✅ ONVIF PTZ test completed!")

def test_http_fallback():
    """Test HTTP fallback controller."""
    print("\n🔧 Testing HTTP Fallback Controller")
    print("=" * 60)
    
    controller = HTTPController(
        ip="192.168.0.18",
        port=80,
        username="admin",
        password="admin",
        timeout_ms=5000
    )
    
    print("\n1. Testing HTTP PTZ Commands...")
    
    # Test movements (these might not work without proper HTTP API knowledge)
    print("   → Testing Pan Right...")
    controller.move_ptz(pan=0.1, tilt=0.0, zoom=0.0)
    time.sleep(1)
    
    print("   → Stopping PTZ...")
    controller.stop_ptz()
    
    print("\n✅ HTTP fallback test completed!")

def main():
    """Main test function."""
    print("🚀 PTZ Controller Fix Verification")
    print("=" * 60)
    print("Testing fixes for:")
    print("  ✓ Correct profile token (MainStream)")
    print("  ✓ Proper ONVIF namespace (ver10)")
    print("  ✓ Simplified authentication (PasswordText)")
    print("  ✓ Velocity space instead of position space")
    print()
    
    try:
        # Test ONVIF first (primary method)
        test_onvif_ptz()
        
        # Test HTTP fallback  
        test_http_fallback()
        
        print("\n🎉 All PTZ tests completed!")
        print("\nNext steps:")
        print("1. If ONVIF commands work now, the issue is resolved!")
        print("2. If still getting 400 errors, we may need to try different SOAP formats")
        print("3. Monitor the camera for actual movement during tests")
        
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 