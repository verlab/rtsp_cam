# 🎯 Curl Tests for Absolute Position Commands

These curl commands test for **numerical absolute positioning** (pan=X°, tilt=Y°).

## 🔍 **Step 1: Test Position Query Commands**

First, let's see if we can get current numerical position:

```bash
# Test various position query endpoints
curl -u admin:admin "http://192.168.0.18/form/getPTZCfg"
curl -u admin:admin "http://192.168.0.18/form/getPTZStatus" 
curl -u admin:admin "http://192.168.0.18/form/getPTZPosition"
curl -u admin:admin "http://192.168.0.18/form/getStatus"

# CGI-style position queries
curl -u admin:admin "http://192.168.0.18/cgi-bin/ptz.cgi?action=getPosition"
curl -u admin:admin "http://192.168.0.18/cgi-bin/ptz.cgi?action=status"
curl -u admin:admin "http://192.168.0.18/cgi-bin/ptz.cgi?action=getCurrentPosition"

# Alternative formats
curl -u admin:admin "http://192.168.0.18/ptz.cgi?cmd=getPosition"
curl -u admin:admin "http://192.168.0.18/axis-cgi/com/ptz.cgi?query=position"
```

## 🎯 **Step 2: Test Absolute Positioning Commands**

Test commands that might accept numerical pan/tilt coordinates:

### **Standard Absolute Move Commands:**
```bash
# Test absolute move with degrees
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=90&tilt=0"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?cmd=absoluteMove&pan=45&tilt=30"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=absoluteMove&pan=0&tilt=0"

# Test setPosition commands
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=setPosition&panPos=120&tiltPos=45"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?cmd=setPosition&x=90&y=30"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=setPosition&panPosition=180&tiltPosition=0"
```

### **Alternative Endpoints:**
```bash
# Test different endpoints
curl -u admin:admin "http://192.168.0.18/form/setPTZ?pan=90&tilt=45"
curl -u admin:admin "http://192.168.0.18/form/setPTZPosition?pan=0&tilt=0"
curl -u admin:admin "http://192.168.0.18/form/movePTZ?action=absoluteMove&pan=45&tilt=30"

# CGI-bin style
curl -u admin:admin "http://192.168.0.18/cgi-bin/ptz.cgi?action=absoluteMove&pan=90&tilt=0"
curl -u admin:admin "http://192.168.0.18/cgi-bin/ptz.cgi?cmd=goto&pan=45&tilt=30&speed=50"
```

### **Numeric Command Variations:**
```bash
# Test if commands 16-20 are absolute positioning
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=16&pan=90&tilt=0"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=17&pan=45&tilt=30"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=18&pan=0&tilt=45"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=19&pan=180&tilt=0"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=20&pan=270&tilt=0"
```

### **Different Coordinate Systems:**
```bash
# Test different coordinate ranges
# Degrees (0-360, -180 to +180)
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=90&tilt=45"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=-90&tilt=-30"

# Degrees * 10 (0-3600)
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=900&tilt=450"

# Degrees * 100 (0-36000)
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=9000&tilt=4500"

# Percentage (0-100)
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=50&tilt=25"
```

### **With Speed Control:**
```bash
# Absolute move with speed
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=90&tilt=0&panSpeed=50&tiltSpeed=50"
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?cmd=goto&pan=45&tilt=30&speed=30"
```

### **Dahua-style Commands:**
```bash
# Dahua absolute positioning format
curl -u admin:admin "http://192.168.0.18/cgi-bin/ptz.cgi?action=start&channel=0&code=PositionABS&arg1=90&arg2=45&arg3=0"
curl -u admin:admin "http://192.168.0.18/cgi-bin/ptz.cgi?action=start&channel=0&code=GotoPreset&arg1=90&arg2=45&arg3=50"
```

## 🔍 **Step 3: Test Position Feedback After Movement**

After trying absolute positioning, check if position changed:

```bash
# Move to a position
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=90&tilt=0"

# Check current position
curl -u admin:admin "http://192.168.0.18/form/getPTZCfg"
curl -u admin:admin "http://192.168.0.18/cgi-bin/ptz.cgi?action=getPosition"

# Move to different position  
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=0&tilt=45"

# Check position again
curl -u admin:admin "http://192.168.0.18/form/getPTZCfg"
```

## 🎮 **Step 4: Systematic Testing**

Test one coordinate system at a time:

```bash
# Test 1: Standard degrees (0-360)
echo "Testing degrees 0-360..."
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=0&tilt=0"
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=90&tilt=0" 
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=180&tilt=0"
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=270&tilt=0"

# Test 2: Signed degrees (-180 to +180)
echo "Testing signed degrees..."
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=0&tilt=0"
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=90&tilt=0"
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=-90&tilt=0"

# Test 3: Degrees * 10
echo "Testing degrees * 10..."
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=0&tilt=0"
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=900&tilt=0"
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=1800&tilt=0"
```

## 📊 **Step 5: Analyze Results**

Look for these indicators of success:

1. **HTTP 200 Response**: Command was accepted
2. **Camera Movement**: Physical movement to specified position
3. **Position Feedback**: Query returns the coordinates you set
4. **Consistent Behavior**: Same command produces same result

## 💡 **Expected Findings**

If absolute positioning works, you should see:
- ✅ HTTP 200 responses for successful commands
- ✅ Camera moves to specific positions
- ✅ Position queries return numerical coordinates
- ✅ Repeatable positioning (same input = same position)

If only relative positioning works:
- ❌ Absolute commands return errors or don't move camera
- ✅ Only directional commands (1-4) and presets work
- ❌ No numerical coordinate feedback

## 🚀 **Quick Test Script**

```bash
#!/bin/bash
# Quick test for absolute positioning

echo "🎯 Testing Absolute Positioning..."

# Test basic absolute move
echo "Testing: absoluteMove to pan=90, tilt=0"
curl -s -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=90&tilt=0"
echo "Response code: $?"

sleep 3

# Check position
echo "Getting current position..."
curl -s -u admin:admin "http://192.168.0.18/form/getPTZCfg"

# Test different position
echo "Testing: absoluteMove to pan=0, tilt=45"  
curl -s -u admin:admin "http://192.168.0.18/form/setPTZCfg?action=absoluteMove&pan=0&tilt=45"

sleep 3

# Check position again
echo "Getting position after move..."
curl -s -u admin:admin "http://192.168.0.18/form/getPTZCfg"
```

Save this as `test_absolute.sh`, make it executable with `chmod +x test_absolute.sh`, and run it! 