# RTAB-Map 3D Mapping with Intel RealSense D435i

3D SLAM (Simultaneous Localization and Mapping) on the ASR-D501 using the Intel RealSense D435i depth camera.

---

## Prerequisites

- Ubuntu 24.04 LTS on the ASR-D501
- ROS 2 Jazzy installed
- Intel RealSense D435i connected via USB 3.x

---

## 1. Install Intel RealSense SDK (ARM64)

Since the ASR-D501 is ARM64, librealsense must be built from source.

### 1.1 Install Build Dependencies

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
  build-essential cmake git pkg-config wget curl unzip \
  libssl-dev libusb-1.0-0-dev libudev-dev \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
  python3-dev python3-pip python3-numpy
```

### 1.2 Clone and Build librealsense

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Set up udev rules (camera works without sudo)
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Build with the RSUSB backend (critical on ARM64 — the kernel patch approach is unreliable outside x86):

```bash
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_EXAMPLES=true \
  -DBUILD_GRAPHICAL_EXAMPLES=true \
  -DBUILD_PYTHON_BINDINGS=true \
  -DPYTHON_EXECUTABLE=$(which python3)

make -j$(nproc)
sudo make install
sudo ldconfig
```

### 1.3 Verify the Camera

```bash
rs-enumerate-devices
```

Should show the D435I with firmware version and **USB Type: 3.2**. If it shows USB 2.x, use a different port or cable.

---

## 2. Install ROS 2 RealSense Wrapper

```bash
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description
```

If apt packages are not available, build from source:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

cd ~/ros2_ws
sudo apt install -y python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Install RTAB-Map

```bash
sudo apt install -y \
  ros-jazzy-rtabmap \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rtabmap-launch \
  ros-jazzy-rtabmap-examples \
  ros-jazzy-rtabmap-msgs \
  ros-jazzy-rtabmap-odom \
  ros-jazzy-rtabmap-slam \
  ros-jazzy-rtabmap-viz \
  ros-jazzy-rtabmap-rviz-plugins \
  ros-jazzy-imu-filter-madgwick
```

---

## 4. Running RTAB-Map (Handheld Mapping)

Open **three terminals**.

### Terminal 1 — Start Camera

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true \
  enable_color:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  align_depth.enable:=true \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30 \
  depth_module.emitter_enabled:=1 \
  depth_module.laser_power:=360 \
  spatial_filter.enable:=true \
  temporal_filter.enable:=true
```

### Terminal 2 — Start IMU Filter

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=false \
  -p publish_tf:=false \
  -p world_frame:="enu" \
  -p gain:=0.01 \
  -p frequency:=100.0 \
  -r /imu/data_raw:=/camera/camera/imu
```

### Terminal 3 — Start Mapping

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  args:="--delete_db_on_start" \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=false \
  wait_imu_to_init:=true \
  imu_topic:=/imu/data \
  qos:=1
```

### Mapping Tips

- Move the camera slowly (~0.3 m/s, ~30 deg/s rotation)
- Keep 50%+ visual overlap between frames
- Revisit areas to trigger loop closure (corrects drift)
- Point at textured surfaces — featureless walls cause tracking loss
- Usable depth range: ~0.3 to 3 meters indoors

---

## 5. Viewing and Exporting Maps

### View a Saved Map

The map database is saved automatically to `~/.ros/rtabmap.db`.

To view it:

```bash
rtabmap
```

Then: **File > Open Database** and select the `.db` file.

### Export as OctoMap (.bt)
 
Inside rtabmap: **Window > Preferences** > find and enable the **OctoMap** checkbox. Then **File > Export OctoMap**.
 
### View OctoMap in RViz2
 
Start the OctoMap server with `frame_id:=map` so RViz2 doesn't require manually changing the fixed frame:
 
```bash
ros2 run octomap_server octomap_server_node --ros-args -p octomap_path:=/home/<USERNAME>/.ros/maps/<MAPNAME>.bt -p frame_id:=map
```
 
Then open RViz2:
 
```bash
rviz2
```
 
- **Add > By Topic > PointCloud2**
- Open PointCloud2 dropdown > open Topic dropdown > change `durability policy` to `Transient Local` 

 
> If topics don't appear in RViz: set **Reliability** to `Reliable` and **Durability** to `Transient Local` in the topic settings.
---

## 6. Troubleshooting

| Problem | Fix |
|---------|-----|
| "No device connected" | Use USB 3.x cable in USB-C port. Remove other USB devices. |
| "Did not receive data since 5 seconds" | Set `qos:=1`. Verify topics with `ros2 topic list`. |
| "Odom: quality=0" | Move slower. Point at textured surfaces. |
| Frame drops / stuttering | Reduce to 640x480@15. Check thermal: `cat /sys/class/thermal/thermal_zone*/temp` |
| Depth noisy outdoors | IR projector gets washed out by sunlight. Indoor use only for depth. |