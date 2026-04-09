# oak-aruco-board-pose-ros2

ROS 2 package for real-time ArUco board pose estimation using a Luxonis OAK camera. The package detects a configured ArUco marker board, estimates its 6-DoF pose from camera calibration + board geometry, and publishes:

- `geometry_msgs/PoseStamped` on `/board_pose`
- `geometry_msgs/Vector3Stamped` on `/board_rpy`
- `std_msgs/Bool` on `/board_visible`
- `std_msgs/Int32MultiArray` on `/board_used_ids`
- TF transform from `oak_camera_frame` to `board_frame`

## 1. Project context

This package is part of the **SmartFollower & Tracker (SFT)** project for warehouse anomaly investigation. The larger system goal is an indoor TurtleBot 4–based robot that can detect and follow an Object of Interest (OOI), handle occlusions, navigate safely, and produce map/trajectory/evidence logs. The project site explicitly lists **ArUco-based OOI detection and tracking** as part of scope, along with smart following control and 2D reconstruction.

### How this package contributes to the big picture

This package provides the **perception layer** for board/marker-based target localization:

- detects the configured ArUco board on the target
- estimates target-relative pose `(x, y, z, rx, ry, rz)`
- publishes pose in ROS 2 for downstream consumers
- provides visibility status and detected marker IDs for diagnostics

That makes it a building block for:

- target reacquisition
- follower control
- state estimation fusion
- logging / evidence generation
- visualization in RViz / TF tools

## 2. ROS 2 version used

This package was developed with **ROS 2 Jazzy Jalisco** on Ubuntu. ROS 2 Jazzy is a supported ROS 2 distribution with standard support for Python packages, launch files, parameters, topics, and tf2, all of which are used here.

## 3. Features

- Luxonis OAK camera input via `depthai`
- ArUco marker detection via OpenCV `aruco`
- pose estimation via `cv2.solvePnP`
- support for per-marker rotation in the board config
- optional exponential smoothing on translation
- ROS 2 topic publishing
- TF broadcasting
- logger output for:
  - board visibility
  - detected marker IDs
  - `x y z`
  - `rx ry rz`

## 4. Repository structure

```text
oak-aruco-board-pose-ros2/
├── board_pose_ros/
│   ├── __init__.py
│   └── board_pose_node.py
├── config/
│   ├── board_config.json
│   └── camera_calib_oak.npz
├── launch/
│   └── board_pose.launch.py
├── package.xml
├── resource/
│   └── board_pose_ros
├── setup.cfg
├── setup.py
└── README.md
```


## 5. Requirements

### Hardware

* Luxonis OAK camera
* Printed ArUco board with known marker IDs and geometry

### Software

* Ubuntu
* ROS 2 Jazzy
* Python 3
* `depthai`
* `opencv-contrib-python`
* `numpy`

## 6. Installation and setup

### 6.1 Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Mobile-Robots-UGV/oak-aruco-board-pose-ros2.git
```

### 6.2 Install ROS 2 Jazzy

Install ROS 2 Jazzy on Ubuntu using the official ROS 2 Jazzy instructions.

### 6.3 Install Python dependencies

Because Ubuntu may block direct `pip` writes into system Python, install with:

```bash
python3 -m pip install --user --break-system-packages depthai opencv-contrib-python numpy
```

Verify:

```bash
python3 - <<'PY'
import depthai
import cv2
import numpy
print("depthai:", depthai.__version__)
print("cv2:", cv2.__version__)
print("numpy:", numpy.__version__)
PY
```

### 6.4 Add OAK / Movidius udev rule

```bash
sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug the OAK camera after this.

### 6.5 Build the package

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select board_pose_ros --symlink-install
source ~/ros2_ws/install/setup.bash
```

## 7. Calibration workflow

This package expects a valid camera calibration file, for example:

* `config/camera_calib_oak.npz`

The calibration used here was produced from a ChArUco calibration board and saved as:

* `camera_calib_oak.npz`

A successful calibration should give a reasonable RMS reprojection error and save:

* `camera_matrix`
* `dist_coeffs`

## 8. Board configuration

The board geometry is defined in:

* `config/board_config.json`

Example fields:

* `board_size_m`
* `marker_size_m`
* `dictionary`
* `markers`

  * marker ID
  * `top_left_xy_m`
  * optional `rotation_deg`

Example:

```json
{
  "board_size_m": 0.15,
  "marker_size_m": 0.0225,
  "dictionary": "DICT_6X6_250",
  "frame_description": "Board frame origin at board center. +X points right on the board. +Y points up on the board. Z=0 is the board plane.",
  "markers": {
    "1": {"top_left_xy_m": [-0.075, 0.075], "rotation_deg": 0},
    "3": {"top_left_xy_m": [0.0525, 0.075], "rotation_deg": 0},
    "2": {"top_left_xy_m": [0.0525, -0.0525], "rotation_deg": 180},
    "4": {"top_left_xy_m": [-0.075, -0.0525], "rotation_deg": 0}
  }
}
```

## 9. Running the package

Launch the node:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch board_pose_ros board_pose.launch.py
```

## 10. Published topics

### `/board_visible`

Type: `std_msgs/Bool`

Indicates whether the configured board is currently detected.

### `/board_used_ids`

Type: `std_msgs/Int32MultiArray`

Contains the marker IDs used for the current pose estimate.

### `/board_pose`

Type: `geometry_msgs/PoseStamped`

Contains:

* position: `x y z` in meters
* orientation: quaternion

### `/board_rpy`

Type: `geometry_msgs/Vector3Stamped`

Contains:

* `rx = roll`
* `ry = pitch`
* `rz = yaw`

By default these are in **radians** unless you only convert them in logs.

### TF

Transform:

* parent: `oak_camera_frame`
* child: `board_frame`

## 11. Inspecting runtime output

### Topic list

```bash
ros2 topic list | grep board
```

### Echo visibility

```bash
ros2 topic echo /board_visible
```

### Echo IDs

```bash
ros2 topic echo /board_used_ids
```

### Echo pose

```bash
ros2 topic echo /board_pose
```

### Echo roll/pitch/yaw

```bash
ros2 topic echo /board_rpy
```

### Check TF

```bash
ros2 run tf2_ros tf2_echo oak_camera_frame board_frame
```

## 12. Example logger output

```text
[INFO] [board_pose_node]: visible=True ids=[1,2,3,4] x=0.0698 y=0.0998 z=0.6311 rx=2.9174 ry=0.3477 rz=-0.0737
```

This means:

* board is visible
* all four configured markers were used
* translation is `(x, y, z)` in meters
* rotation is `(roll, pitch, yaw)` in radians

## 13. Configuration notes

### Logging

The node supports:

* `log_pose`
* `log_every_n`
* `log_rpy_degrees`

### Pose quality

Pose is generally best when:

* more markers are visible
* the board is not motion-blurred
* lighting is stable
* the board fills a useful portion of the frame
* camera calibration is accurate

### Partial visibility

The node can still estimate pose with a subset of the markers, but pose quality may degrade when only one or two markers are visible.

## 14. Troubleshooting

### `No available devices`

The OAK camera is not accessible.
Check:

* USB connection
* udev rule
* no other process is holding the device

Test:

```bash
python3 - <<'PY'
import depthai as dai
print(dai.Device.getAllAvailableDevices())
PY
```

### `ModuleNotFoundError: No module named 'depthai'`

Install Python dependencies into the Python environment used by ROS 2:

```bash
python3 -m pip install --user --break-system-packages depthai opencv-contrib-python numpy
```

### Board not detected

Check:

* marker dictionary
* marker IDs
* board config geometry
* marker size
* marker rotations
* lighting / blur / occlusion

### Pose scale looks wrong

Usually caused by:

* wrong `marker_size_m`
* wrong board coordinates
* bad camera calibration

## 15. Future extensions

* publish debug image overlays as a ROS image topic
* add RViz markers
* publish covariance / quality metrics
* integrate with follower controller
* fuse board pose with robot localization and target-tracking state machine

## 16. Relation to the [SmartFollower & Tracker project](https://mobile-robots-ugv.github.io/)

The [SmartFollower & Tracker project](https://mobile-robots-ugv.github.io/) targets ArUco-based target detection, robust target following, safety-aware navigation, and 2D reconstruction/evidence logging in warehouse-like environments. This repository contributes the target pose-estimation module that gives ROS 2-accessible target-relative pose for downstream planning, control, logging, and visualization.
