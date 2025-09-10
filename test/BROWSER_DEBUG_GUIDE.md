# 🔍 Browser Developer Tools Guide
## Capture Working PTZ Commands from Web Interface

Since you can control the camera via web interface, we can capture the **exact commands** it sends!

## 📋 Step-by-Step Instructions:

### 1. Open Camera Web Interface
- Open browser and go to: `http://192.168.0.18`
- Login with: `admin` / `admin`
- Navigate to PTZ control page

### 2. Open Developer Tools
- **Chrome/Edge**: Press `F12` or `Ctrl+Shift+I`
- **Firefox**: Press `F12` or `Ctrl+Shift+I`
- **Safari**: Press `Cmd+Option+I`

### 3. Monitor Network Traffic
- Click on **"Network"** tab in developer tools
- **Clear** any existing requests (trash/clear button)
- **Filter** by "XHR" or "Fetch" to see only AJAX requests

### 4. Test PTZ Movement
- **Move the camera** using web interface controls (pan right, tilt up, etc.)
- **Watch the Network tab** - you'll see HTTP requests appear!

### 5. Analyze the Requests
For each PTZ command that works, **right-click** on the network request and:
- **Copy as cURL** (this gives you the exact command!)
- Or click on the request to see:
  - **Request URL**
  - **Request Method** (GET/POST)
  - **Query Parameters**
  - **Request Headers**

### 6. Test the Captured Commands
Copy the cURL commands and test them in terminal:
```bash
# Example of what you might find:
curl -u admin:admin "http://192.168.0.18/some/path?param1=value1&param2=value2"
```

## 🎯 What to Look For:

### Common PTZ URL Patterns:
- `/cgi-bin/ptz.cgi`
- `/decoder_control.cgi`
- `/ptzctrl.cgi`  
- `/camera-cgi/ptz.cgi`
- `/web/ptz`
- `/api/ptz`

### Common Parameters:
- `action=start` / `action=stop`
- `code=Right` / `code=Left` / `code=Up` / `code=Down`
- `cmd=ptz`
- `direction=right`
- `speed=5`
- `channel=0` or `channel=1`
- `arg1=1` (address)
- `arg2=20` (speed)

## 📝 Example Network Request Analysis:

```
Request URL: http://192.168.0.18/cgi-bin/ptz.cgi?action=start&code=Right&channel=0&arg1=1&arg2=20&arg3=0
Method: GET
Status: 200 OK
```

This would translate to our Python test as:
```python
{
    "url": "http://192.168.0.18/cgi-bin/ptz.cgi",
    "params": {
        "action": "start",
        "code": "Right", 
        "channel": "0",
        "arg1": "1",
        "arg2": "20",
        "arg3": "0"
    }
}
```

## 🚀 Quick Alternative Method:

If developer tools seem complex, you can also:

1. **Run the web interface analyzer**:
   ```bash
   python3 test/test_web_interface_analysis.py
   ```

2. **Try the manual curl commands** it provides

3. **Check the saved HTML files** for JavaScript PTZ functions

## 💡 Pro Tips:

- **Clear network tab** before each PTZ command to isolate the request
- **Try different PTZ movements** (pan, tilt, zoom, presets) to see all command types
- **Look for both "start" and "stop" commands**
- **Note any authentication headers or cookies**
- **Check if commands are GET or POST requests**

## 🎯 Expected Outcome:

Once you find the working command format, we can update the ROS 2 PTZ controller to use the **exact same format**!

The key is finding the **specific URL and parameters** that your camera's web interface uses for PTZ control. 