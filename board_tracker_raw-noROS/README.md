
# board_tracker_raw-noROS

Standalone, non-ROS Python tools for calibrating an OAK camera and estimating the pose of an ArUco marker board.

This folder contains the raw OpenCV + DepthAI workflow that was used before wrapping the system in ROS 2. It is useful for:

- camera bring-up
- marker ID checking
- ChArUco calibration
- board pose debugging
- validating board geometry and marker rotations before using the ROS 2 node

## Contents

Typical workflow files in this folder:

- `capture_calibration.py` — capture calibration images from the OAK camera
- `calibrate_from_images.py` — compute camera intrinsics from ChArUco images
- `detect_marker_ids.py` — show live marker detections and print marker IDs
- `track_board_pose.py` — estimate board pose and show `X Y Z` live
- `board_config.json` — board geometry, marker size, marker IDs, and rotations

The calibration script is ChArUco-based and uses an 11 x 8 board with configurable square size, marker size, and 4x4 dictionary.
The board tracker loads a camera calibration file and a JSON board configuration, then estimates pose with `solvePnP`.
The live ID detector uses OpenCV ArUco detection and prints the detected IDs from the camera stream.
The current board config uses `DICT_6X6_250`, `marker_size_m = 0.0225`, and marker `2` rotated by 180 degrees.

## Requirements

### Hardware

- Luxonis OAK camera
- printed ChArUco calibration board
- printed ArUco tracking board

### Python packages

Install these:

```bash id="d3u1pm"
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install depthai opencv-contrib-python numpy
````

Why these packages:

* `depthai` — OAK camera access
* `opencv-contrib-python` — OpenCV + ArUco module
* `numpy` — matrix math and calibration file handling

## OAK camera permissions

On Linux, add the Movidius/OAK udev rule:

```bash id="z4r2ql"
sudo tee /etc/udev/rules.d/80-movidius.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug the OAK camera.

Test that the camera is visible:

```bash id="w8y7sf"
python3 - <<'PY'
import depthai as dai
print(dai.Device.getAllAvailableDevices())
PY
```

## Step-by-step workflow

## 1) Capture calibration images

Use the OAK camera to capture images of your ChArUco board:

```bash id="k7c2dn"
python capture_calibration.py --out calib_images
```

Press:

* `SPACE` to save an image
* `q` to quit

Take about 20 to 30 images with:

* different distances
* different tilts
* different positions in the frame
* sharp focus
* low glare

## 2) Calibrate the camera

Run the ChArUco calibration script:

```bash id="v9a8mh"
python calibrate_from_images.py \
  --images calib_images \
  --cols 11 \
  --rows 8 \
  --square 0.015 \
  --marker 0.011 \
  --dict DICT_4X4_50 \
  --out camera_calib_oak.npz
```

What these mean:

* `--cols 11` = number of squares horizontally
* `--rows 8` = number of squares vertically
* `--square 0.015` = square size in meters
* `--marker 0.011` = inner ArUco marker size in meters
* `--dict DICT_4X4_50` = ChArUco marker dictionary

The calibration script saves:

* `camera_matrix`
* `dist_coeffs`
* image size
* board settings

in the output `.npz` file. 

A good calibration should report a reasonable RMS error.

## 3) Check your tracking board marker IDs

Before running pose tracking, verify that the printed marker board is detected correctly:

```bash id="k1m2qv"
python detect_marker_ids.py --dict DICT_6X6_250
```

This opens a live OpenCV window and prints the detected IDs periodically. 

Use this step to confirm:

* dictionary is correct
* marker IDs are correct
* all markers are visible
* print quality is good

## 4) Verify your board configuration

The tracker uses `board_config.json` for the physical layout of the tracking board. The current config is:

```json id="n2v4kb"
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

This means:

* board size = 15 cm x 15 cm
* marker size = 2.25 cm
* dictionary = `DICT_6X6_250`
* marker IDs = `1, 3, 2, 4`
* marker `2` is rotated by 180 degrees 

Adjust this file if your board geometry is different.

## 5) Run live board pose tracking

Once calibration and config are correct:

```bash id="j5h6yf"
python track_board_pose.py \
  --calib camera_calib_oak.npz \
  --config board_config.json \
  --draw-axes
```

The tracker:

* loads `camera_calib_oak.npz`
* loads `board_config.json`
* detects configured markers
* builds 2D-3D correspondences
* runs `cv2.solvePnP`
* overlays pose info on screen 

Expected live output includes:

* `used ids`
* `X`
* `Y`
* `Z`
* board pixel center
* optional coordinate axes

## What each script does

### `capture_calibration.py`

Captures calibration images from a live camera stream and saves them to a folder.
Use this first when creating a fresh calibration dataset. 

### `calibrate_from_images.py`

Runs ChArUco calibration from the saved images and writes a `.npz` file containing camera intrinsics and distortion coefficients. 

### `detect_marker_ids.py`

Live camera utility for checking which marker IDs are visible and whether the selected ArUco dictionary is correct. 

### `track_board_pose.py`

Loads the calibration + board config and performs pose estimation of the configured board in real time. 

## Output interpretation

### Translation

The tracker reports:

* `X`
* `Y`
* `Z`

These are in meters internally and often displayed in centimeters in the GUI.

### Rotation

The board orientation is estimated from the PnP solution. In the raw non-ROS version, the GUI focuses on translation and axes visualization; the ROS version additionally publishes pose and RPY.

### Marker usage

`used ids` shows which configured markers contributed to the pose estimate.
Pose quality is best when more markers are visible.

## Typical run order

```bash id="h1v2cb"
source .venv/bin/activate

python capture_calibration.py --out calib_images

python calibrate_from_images.py \
  --images calib_images \
  --cols 11 \
  --rows 8 \
  --square 0.015 \
  --marker 0.011 \
  --dict DICT_4X4_50 \
  --out camera_calib_oak.npz

python detect_marker_ids.py --dict DICT_6X6_250

python track_board_pose.py \
  --calib camera_calib_oak.npz \
  --config board_config.json \
  --draw-axes
```

## Troubleshooting

### `No available devices`

The OAK camera is not accessible.

Check:

* USB cable / port
* udev rule
* no other process is holding the device

Test with:

```bash id="z0f4cs"
python3 - <<'PY'
import depthai as dai
print(dai.Device.getAllAvailableDevices())
PY
```

### `ModuleNotFoundError: No module named 'depthai'`

Install:

```bash id="d0m7qa"
pip install depthai
```

or inside a venv:

```bash id="t0w8dz"
python3 -m venv .venv
source .venv/bin/activate
pip install depthai opencv-contrib-python numpy
```

### Calibration rejects all images

Usually caused by:

* bad ChArUco board visibility
* wrong `cols`, `rows`, `square`, `marker`, or dictionary
* board too small in frame
* blur or glare

### Wrong pose scale

Usually caused by:

* wrong `marker_size_m`
* wrong board marker coordinates
* wrong camera calibration file

## Relation to the ROS package

This folder is the raw, non-ROS development path for the same board-tracking problem. It is useful for:

* debugging camera access
* calibrating the OAK camera
* validating the marker board
* verifying pose estimation before running the ROS 2 wrapper

Once this works, the ROS 2 package wraps the same logic into launchable nodes, topics, TF, and logs.


