# Installing ROS 2 Jazzy and MAVROS on a Companion Computer

This guide covers installing ROS 2 Jazzy and MAVROS on an Ubuntu 24.04 LTS companion computer (ASR-D501, Raspberry Pi or similar) for use with ArduPilot/Pixhawk drones.

---

## Prerequisites
 
- **Ubuntu 24.04 LTS is required.** ROS 2 Jazzy binary packages are only available for Ubuntu 24.04. If you are running Ubuntu 22.04 or older, Jazzy will not install via apt. You would need to either upgrade to 24.04 or use an older ROS 2 distro (Humble for 22.04).
- Architecture: amd64 or aarch64 (ARM64). Both are supported.
 
Check your Ubuntu version:
 
```bash
lsb_release -a
```
 
---

## Step 1: Prepare the System
 
Make sure the system is up to date. ROS 2 requires a UTF-8 locale but it does not have to be English. If your system is already using a UTF-8 locale (e.g. `ja_JP.UTF-8`, `zh_CN.UTF-8`, `ko_KR.UTF-8`), you can skip the locale commands and just run the update.
 
Check your current locale:
 
```bash
locale
```
 
If the output shows `.UTF-8` in `LANG`, you're fine. If not (or if it shows `POSIX` or `C`), set one:
 
```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
 
Then update the system:
 
```bash
sudo apt update && sudo apt upgrade -y
```
 
---

## Step 2: Set Up ROS 2 Repositories

Add the ROS 2 apt repositories to your system so it knows where to download the packages.

```bash
# Enable the Ubuntu Universe repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# Add the ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

## Step 3: Install ROS 2 Jazzy

Since this is running on a companion computer (which usually acts as a headless board on the drone), install the `ros-base` version to save CPU and storage. It omits the heavy GUI tools that you don't need on the drone.

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base -y
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```

Source the installation so your terminal recognizes ROS commands. Adding it to `.bashrc` makes it persistent across reboots.

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


---

## Step 4: Install MAVROS

MAVROS is the bridge between ROS 2 and the ArduPilot flight controller over MAVLink.

```bash
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras -y
```

**Crucial step:** MAVROS requires GeographicLib datasets to handle GPS coordinate math. If you skip this, MAVROS will crash on startup with a missing dataset error.

```bash
sudo wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```

---

## Step 5: Serial Port Permissions
 
On a fresh Ubuntu install, your user does not have permission to access the serial port (`/dev/ttyACM0`). MAVROS will fail with "Permission denied" if you skip this.
 
Add your user to the `dialout` group:
 
```bash
sudo usermod -a -G dialout $USER
```
 
**You must log out and log back in** (or reboot) for this to take effect. You can verify it worked with:
 
```bash
groups
```
 
You should see `dialout` in the list.
 
---

## Step 6: Verify the Installation

Check that everything is installed correctly.

```bash
# Verify ROS 2
ros2 --version

# Verify MAVROS packages are available
ros2 pkg list | grep mavros
```

You should see `mavros`, `mavros_extras`, and `mavros_msgs` in the output.

---

## Step 7: Launch MAVROS

Connect the Pixhawk flight controller to the companion computer via USB, then launch MAVROS.

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:115200
```

If you see `FCU: ArduCopter` and heartbeat messages, the connection is working. If the USB device is different on your board, check with `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`.

--- 

## NPU Acceleration (Optional)
To be able to run yolo detection on the drone using the Qualcomm AI accelerator, you will need to follow this guide: 

https://github.com/ADVANTECH-Corp/EdgeAI_Workflow/blob/main/ai_system/qualcomm/aom-dk2721/linux/object_detection_demo-using-qc_ai_hub.md

In case of using Ubuntu with USB camera (instead of Yocto with MIPI camera), you will need to modify yolov8_cam_ai_hub.sh script to be compatible with Ubuntu and your camera output format.