# 🤖 ROS# Direct Camera Integration Setup Guide

## Overview
This setup connects Unity directly to ROS topics using **ROS#** (siemens/ros-sharp) for real-time camera streaming.

---

## 📦 What Was Installed:

1. **ROS# Package** - Added to `Packages/manifest.json`
2. **RosSharpImageSubscriber.cs** - Subscribes to individual camera topics
3. **RosSharpCameraManager.cs** - Manages multiple camera feeds

---

## 🚀 Setup Steps:

### Step 1: Start ROS Bridge on Robot

SSH into your robot and start the ROS bridge server:

```bash
# SSH to robot
ssh rm@100.114.29.13

# Start rosbridge_server (websocket server for ROS)
roslaunch rosbridge_server rosbridge_websocket.launch

# You should see:
# [INFO] Rosbridge WebSocket server started on port 9090
```

**Note:** Port 9090 is the standard rosbridge_server port.

---

### Step 2: Start Camera Drivers

In another terminal on the robot:

```bash
# Start your camera drivers
roslaunch door_traverse collect.launch

# Verify cameras are publishing
rostopic list | grep camera

# Expected output:
# /camera0/image_raw
# /camera1/image_raw
# /camera_0/color/image_raw
# /camera_1/color/image_raw
# etc.
```

---

### Step 3: Unity Setup

#### Option A: Quick Setup (Automatic)

1. **Create GameObject** in your scene:
   - Right-click Hierarchy → Create Empty
   - Name it "ROSCameraSystem"

2. **Add Script:**
   - Add Component → **RosSharpCameraManager**

3. **Configure in Inspector:**
   - **Ros Bridge Server Url**: `ws://100.114.29.13:9090`
   - **Auto Create Display**: ✓ (checked)
   - **Grid Columns**: 2
   - **Camera Topics**: (already pre-configured for common topics)

4. **Press Play!**

#### Option B: Custom Setup (Manual)

1. **Create GameObject with RosConnector:**
   ```
   GameObject → Create Empty → "RosConnector"
   Add Component → RosSharp → RosConnector
   ```

2. **Configure RosConnector:**
   - **Ros Bridge Server Url**: `ws://100.114.29.13:9090`
   - **Serializer**: JSON
   - **Protocol**: WebSocketNET

3. **For each camera, create a display:**
   ```
   GameObject → UI → Raw Image
   Add Component → RosSharpImageSubscriber
   ```

4. **Configure each subscriber:**
   - **Camera Topic**: `/camera0/image_raw`
   - **Camera Name**: `USB_Cam_0`
   - **Display Image**: Assign the RawImage component
   - Drag RosConnector to the subscriber

---

## 🔧 Configuration:

### Camera Topics Configuration

Edit in Inspector or code:

```csharp
// USB Cameras
/camera0/image_raw → USB_Cam_0
/camera1/image_raw → USB_Cam_1

// D435 Cameras  
/camera_0/color/image_raw → D435_Cam_0
/camera_1/color/image_raw → D435_Cam_1
/camera_2/color/image_raw → D435_Cam_2
/camera_3/color/image_raw → D435_Cam_3
```

### Performance Settings

**Compressed Images** (Recommended):
- Uses `/camera0/image_raw/compressed` topics
- Much lower bandwidth
- Better performance over Tailscale/WiFi

**Raw Images**:
- Uses `/camera0/image_raw` topics
- Higher bandwidth
- Better quality

The system automatically tries compressed first, then falls back to raw.

---

## 🌐 Network Requirements:

### Port Configuration:

| Port | Service | Required |
|------|---------|----------|
| 9090 | rosbridge_server (WebSocket) | ✅ Yes |
| 22 | SSH (optional) | Optional |

### Tailscale VPN:

```powershell
# Ensure Tailscale is running
& "C:\Program Files\Tailscale\tailscale.exe" status

# Should show robot online:
# 100.114.29.13   realman   linux   -
```

### Firewall (on robot):

```bash
# Allow rosbridge port
sudo ufw allow 9090

# Check firewall status
sudo ufw status
```

---

## 📊 Expected Results:

### Console Output (Unity):

```
RosSharpCameraManager: Initializing ROS camera system...
Connecting to ROS Bridge at: ws://100.114.29.13:9090
✓ Connected to ROS Bridge successfully!
Creating camera display grid...
Created 4 camera displays
✓ Subscribed to ROS topic: /camera0/image_raw
✓ Subscribed to ROS topic: /camera1/image_raw
✓ Subscribed to ROS topic: /camera_0/color/image_raw
✓ Subscribed to ROS topic: /camera_1/color/image_raw
```

### In VR Scene:

- 2x2 grid of camera feeds appears in front of player
- Each camera shows live feed at ~30 FPS
- Labels show camera name and current FPS
- Green text indicates active streaming

---

## 🐛 Troubleshooting:

### "Not connected to ROS Bridge"

**Problem:** Can't connect to ws://100.114.29.13:9090

**Solutions:**

1. **Check rosbridge is running:**
```bash
ssh rm@100.114.29.13
ps aux | grep rosbridge

# If not running:
roslaunch rosbridge_server rosbridge_websocket.launch
```

2. **Check port is accessible:**
```powershell
# Test from Windows
Test-NetConnection -ComputerName 100.114.29.13 -Port 9090
```

3. **Check Tailscale:**
```powershell
& "C:\Program Files\Tailscale\tailscale.exe" status
ping 100.114.29.13
```

---

### "Failed to subscribe to topic"

**Problem:** Camera topics not available

**Solutions:**

1. **Check ROS topics exist:**
```bash
rostopic list | grep camera
```

2. **Check cameras are publishing:**
```bash
rostopic hz /camera0/image_raw
# Should show frequency like "average rate: 30.000"
```

3. **Check topic names match:**
   - Your actual topics might be different
   - Update camera topics in Unity Inspector

---

### Low FPS / Laggy

**Solutions:**

1. **Use compressed images:**
   - Camera topics automatically use `/compressed` version
   - Much lower bandwidth

2. **Reduce resolution on robot:**
```bash
# In your launch file, reduce camera resolution
<param name="width" value="640"/>
<param name="height" value="480"/>
```

3. **Reduce frame rate:**
```bash
<param name="framerate" value="15"/>
```

---

## 🎯 Comparison: ROS# vs HTTP:

| Feature | ROS# (Direct) | HTTP (Flask) |
|---------|---------------|--------------|
| Connection | WebSocket (9090) | HTTP (8080) |
| Latency | Lower | Higher |
| Bandwidth | Better (compressed) | Higher |
| Setup | rosbridge_server | Python Flask |
| Best for | Robotics apps | Web browsers |
| This guide | ✅ **Recommended** | Alternative |

---

## 📝 Quick Command Reference:

### On Robot:

```bash
# Start rosbridge
roslaunch rosbridge_server rosbridge_websocket.launch

# Start cameras
roslaunch door_traverse collect.launch

# List topics
rostopic list

# Check camera frequency
rostopic hz /camera0/image_raw

# Check rosbridge is running
ps aux | grep rosbridge

# Test locally
rostopic echo /rosbridge_status
```

### On PC (Windows):

```powershell
# Check Tailscale
& "C:\Program Files\Tailscale\tailscale.exe" status

# Test robot connection
ping 100.114.29.13

# Test rosbridge port
Test-NetConnection -ComputerName 100.114.29.13 -Port 9090
```

---

## ✅ Checklist:

**On Robot:**
- [ ] Tailscale running
- [ ] rosbridge_server running on port 9090
- [ ] Camera drivers publishing topics
- [ ] Firewall allows port 9090

**On PC:**
- [ ] Tailscale connected
- [ ] Can ping robot: `ping 100.114.29.13`
- [ ] Can reach port 9090

**In Unity:**
- [ ] ROS# package installed
- [ ] ROSCameraManagerDirect script added
- [ ] Ros Bridge URL: `ws://100.114.29.13:9090`
- [ ] Press Play → See cameras!

---

## 🎉 You're Done!

Press **Play** in Unity and you should see your ROS cameras streaming live in VR!

The cameras will appear as floating screens in front of you, showing real-time feeds from your robot with low latency through the ROS bridge.

---

**Key Advantages:**
- ✅ Direct ROS integration
- ✅ Low latency WebSocket connection
- ✅ Automatic compressed image support
- ✅ Native ROS message handling
- ✅ No Python Flask server needed
