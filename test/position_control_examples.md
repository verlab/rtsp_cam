# 📍 Jidetech Camera Position Control Guide

Based on testing, the camera uses a **preset-based position system** rather than direct absolute coordinates.

## 🎯 **Working Position Control Methods**

### **1. Basic Movement (Directional)**
```bash
# Move right for 3 seconds, then stop
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=4"
sleep 3
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"

# Move left for 2 seconds, then stop  
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=3"
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"

# Move up for 1 second, then stop
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=1"
sleep 1
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"

# Move down for 2 seconds, then stop
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=2"
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"
```

### **2. Preset-Based Position Control**
```bash
# Store current position as preset 1
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=3&language=en&presetNum=1"

# Store current position as preset 2
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=3&language=en&presetNum=2"

# Go to preset 1 (absolute positioning!)
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=4&language=en&presetNum=1"

# Go to preset 2 (absolute positioning!)
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=4&language=en&presetNum=2"
```

### **3. Position Feedback**
```bash
# Get current position info
curl -u admin:admin "http://192.168.0.18/form/presetSet?flag=query"

# Query specific preset position
curl -u admin:admin "http://192.168.0.18/form/presetSet?flag=query&presetNum=1"
```

### **4. Speed Control (Command 15)**
```bash
# Set movement speed (not absolute position)
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=15&panSpeed=50&tiltSpeed=30"
```

## 🎮 **Position Control Commands**

| Command | Action | Example |
|---------|--------|---------|
| `command=1` | Move Up | `curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=1"` |
| `command=2` | Move Down | `curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=2"` |
| `command=3` | Move Left | `curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=3"` |
| `command=4` | Move Right | `curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=4"` |
| `command=0` | **STOP** | `curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"` |
| `command=15` | Set Speed | `curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=15&panSpeed=X&tiltSpeed=Y"` |

## 📍 **Preset Commands**

| Flag | Action | Example |
|------|--------|---------|
| `flag=3` | Store Preset | `curl -u admin:admin "http://192.168.0.18/form/presetSet?flag=3&presetNum=1"` |
| `flag=4` | Recall Preset | `curl -u admin:admin "http://192.168.0.18/form/presetSet?flag=4&presetNum=1"` |
| `flag=query` | Get Position | `curl -u admin:admin "http://192.168.0.18/form/presetSet?flag=query"` |

## 🎯 **Practical Position Control Workflow**

### **Setup Known Positions:**
```bash
# 1. Move camera to desired position manually or with commands
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=4"  # Move right
sleep 3
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"  # Stop

# 2. Store as preset "Home"
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=3&language=en&presetNum=1"

# 3. Move to another position
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=1"  # Move up
sleep 2
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"  # Stop

# 4. Store as preset "Watch"
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=3&language=en&presetNum=2"
```

### **Use Absolute Positioning:**
```bash
# Go to "Home" position (instant absolute positioning!)
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=4&language=en&presetNum=1"

# Go to "Watch" position (instant absolute positioning!)
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=4&language=en&presetNum=2"

# Check current position
curl -u admin:admin "http://192.168.0.18/form/presetSet?flag=query"
```

## 🚀 **Quick Test Commands**

### **Test Basic Movement:**
```bash
# Test all directions with stops
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=4" && sleep 1 && curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"  # Right
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=3" && sleep 1 && curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"  # Left  
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=1" && sleep 1 && curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"  # Up
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=2" && sleep 1 && curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"  # Down
```

### **Test Preset System:**
```bash
# Store current position
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=3&language=en&presetNum=5"

# Move camera somewhere else
curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=4" && sleep 3 && curl -u admin:admin "http://192.168.0.18/form/setPTZCfg?command=0"

# Return to stored position (should move back instantly!)
curl -u admin:admin "http://192.168.0.18/form/presetSet?existFlag=1&flag=4&language=en&presetNum=5"
```

## 💡 **Key Insights**

1. **No Direct Absolute Coordinates**: Camera doesn't accept X,Y coordinates directly
2. **Preset-Based Absolute Positioning**: Use presets for "absolute" positioning
3. **Movement + Timer = Relative Positioning**: Combine directional movement with timing for relative positioning
4. **Always Send STOP**: Must send `command=0` to stop movement
5. **Position Feedback Limited**: Query returns preset numbers, not coordinates

## 🛠️ **Implementation Strategy for ROS 2**

1. **Use presets for absolute positioning** (store common positions)
2. **Use directional commands + timing for relative movement**
3. **Implement position tracking via preset queries**
4. **Auto-stop functionality** (ZoneMinder style)
5. **Position interpolation** between known presets 