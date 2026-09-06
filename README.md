# SSL_Robotics_Baseline

The Smart Systems Lab's baseline code for 7-DoF WidowX robotic arms: computer-vision assisted pick & place, cluster detection + sorting, planner-friendly pose utilities, and LLM vision integration for natural-language-driven pick & place. It’s aimed at researchers and engineers using Interbotix WidowX arms and ROS 2 to prototype vision → manipulation workflows.

## Highlights / Features
- Non-reactive pick & place using 3D point-cloud clusters (scripts/pick_place.py)
- Color-based object sorting demo using camera color + point cloud (scripts/color_sorter.py)
- Natural-language / vision-guided pick & place via a local Ollama LLM vision model (scripts/ollama_pick_place.py)
- ROS 2-based integration using Interbotix perception and control modules
- Calibration and AR-tag based self-localization helpers (works with Interbotix XS perception tools)

---

## Stack
- Languages: Python (runtime scripts), Shell/CMake for workspace layout and install
- Runtime: ROS 2 (rclpy) + Interbotix ROS packages (Interbotix XS)
- Notable libraries / packages:
  - interbotix_xs_modules, interbotix_perception_modules, interbotix_common_modules
  - rclpy, tf2_ros, tf_transformations
  - OpenCV / cv_bridge (optional but required for direct image handling)
  - Ollama (optional local LLM vision server) for scripts/ollama_pick_place.py

---

## Repository layout (top-level)
```
README.md
LICENSE
documentation.txt                 # developer notes and setup hints
interbotix_ws/                     # ROS workspace layout (install, src, log)
  install/
  log/
  src/                              # entry placeholders referencing Interbotix packages
scripts/
  pick_place.py                     # simple pick & place (point-cloud clusters)
  color_sorter.py                   # detect clusters + sort by color into baskets
  ollama_pick_place.py              # ask "find the red ball" via Ollama vision model
.gitignore
```

How it fits together:
- The ROS workspace (interbotix_ws) holds Interbotix packages used at runtime. The scripts use Interbotix Python APIs to get clusters from the perception stack, convert cluster centres into poses, and command the manipulator end effector.
- scripts/* are the runnable demos. Each script expects the Interbotix perception nodes and ROS TFs to be available.

---

## Quick start — minimum path to run a demo
Prerequisites (high level):
- ROS 2 (the Interbotix perception package you use; tested on ROS 2 distributions supported by Interbotix packages)
- Interbotix XS packages installed (interbotix_xs_modules, interbotix_perception_modules, interbotix_common_modules)
- A color-depth camera (e.g., Intel RealSense) publishing the expected topics
- The WidowX model you use (wx250s is used in the scripts) physically available and wired / powered
- (Optional) Ollama local server running for LLM vision calls (scripts/ollama_pick_place.py)

Clone and prepare
```bash
git clone https://github.com/Just-Antonio/SSL_Robotics_Baseline.git
cd SSL_Robotics_Baseline
```

Launch the Interbotix perception stack (example; matches documentation.txt):
```bash
# Start interbotix XSARM perception with AR tag tuner and pointcloud tuner GUI
ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py \
  robot_model:=wx250s \
  use_armtag_tuner_gui:=true \
  use_pointcloud_tuner_gui:=true
```
Notes:
- Use the robot_model that matches your hardware (scripts currently use `wx250s`).
- The AR tag tuner GUI and pointcloud tuner GUI help calibrate the camera-to-arm transform and crop/filter settings.

Run a demo script (in a separate terminal):
```bash
# pick & place using pointcloud clusters
python3 scripts/pick_place.py

# color sorter
python3 scripts/color_sorter.py

# ollama-backed natural language + vision (interactive)
# ensure an Ollama server is listening if you want LLM vision
python3 scripts/ollama_pick_place.py
```

---

## Scripts — purpose and usage

### scripts/pick_place.py
- Behavior: Polls the Interbotix pointcloud interface for clusters, excludes clusters near the arm AR tag, moves the arm to pick the nearest cluster and places it at a fixed PLACE_POSE.
- Configuration variables at top of file: ROBOT_MODEL, REF_FRAME, ARM_TAG_FRAME, ARM_BASE_FRAME, approach/pick clearances, PLACE_POSE.
- Running: `python3 scripts/pick_place.py`
- Notes: Non-reactive straight-line picks; designed as a simple baseline.

### scripts/color_sorter.py
- Behavior: Reads cluster color information and places objects into different drop locations depending on the hue (color thresholds implemented in color_compare()).
- Important: documentation.txt suggests running the Interbotix perception launch for wx200 (example in the script comments) but script uses `wx250s` in code — ensure robot_model argument and the script ROBOT_MODEL match your hardware.
- Running: `python3 scripts/color_sorter.py`
- Notes: Requires a well-calibrated camera to locate baskets and AR tag position.

### scripts/ollama_pick_place.py
- Behavior: Interactive prompt asks for a target (e.g., "red ball"), grabs a camera frame, sends an image + prompt to an Ollama generate endpoint running locally, parses returned bbox (xc,yc,w,h in 0..1), projects 3D cluster centers into the camera image, matches the nearest cluster to the bbox center, then picks and places it.
- Configurable environment variables (see code):
  - OLLAMA_URL (default: http://localhost:11434)
  - OLLAMA_MODEL (default: moondream)
  - OLLAMA_HTTP_TIMEOUT (default: 45)
- Camera topics used (defaults in script):
  - CAMERA_INFO_TOPIC = /camera/camera/color/camera_info
  - IMAGE_TOPIC = /camera/camera/color/image_raw
- Running: ensure camera topics publish, then `python3 scripts/ollama_pick_place.py`. If OpenCV/cv_bridge are not available the direct image path is disabled and the script will print a warning.

---

## Calibration and recommended setup details
- AR tag-based calibration:
  - Use the ARTag tuner GUI (launched via the interbotix launch args above) to align the arm tag so it is upright and centered in the camera view, then press "snap" to update transforms in RViz.
  - The scripts use the TF from the AR tag to compute a conservative exclusion radius so the arm won't try to pick itself.
- Point cloud tuner:
  - Use the pointcloud tuner GUI to crop/threshold the scene for cleaner clusters (crop box, voxel filters, plane removal).
- Camera intrinsics:
  - scripts/ollama_pick_place.py reads CameraInfo to compute projections — supplying accurate CameraInfo improves 2D→3D matching.

---

## Configuration matrix / constants (where to edit)
- Robot model and frame names: top of each script (ROBOT_MODEL, ROBOT_NAME, REF_FRAME, ARM_TAG_FRAME, ARM_BASE_FRAME)
- Pick/place geometry and clearances: APPROACH_CLEARANCE_M, PICK_CLEARANCE_M, PICK_TOUCHDOWN_M, PITCH_DEFAULT, PLACE_POSE
- Exclude radius to avoid picking near the arm’s tag: EXCLUDE_ARMTAG_RADIUS_M
- Ollama settings: OLLAMA_URL, OLLAMA_MODEL, OLLAMA_HTTP_TIMEOUT (environment variables supported in ollama_pick_place.py)

---

## Dependencies
- System / ROS:
  - ROS 2 distribution compatible with your Interbotix packages
  - interbotix_xs_modules, interbotix_perception_modules, interbotix_common_modules (install per Interbotix instructions)
  - tf2_ros, sensor_msgs, geometry_msgs
- Python:
  - rclpy, numpy
  - tf_transformations
  - (Optional but required for image path): opencv-python and cv_bridge
- Optional:
  - A local Ollama server with a vision-capable model (the scripts use a default model "moondream")

---

## Troubleshooting / common warnings
- TF lookup failures: you may see warnings like "TF lookup ... failed". Ensure your AR tag is visible to the camera and that the interbotix_xsarm_perception node published the transform.
- No CameraInfo: If CameraInfo isn't available, 2D→3D projections will be approximate or disabled — check the camera driver node and topic names.
- OpenCV/cv_bridge missing: ollama_pick_place.py will still run but direct vision (capturing frames in the script) will be disabled. Install cv_bridge for native ROS → OpenCV conversion.
- Ollama API errors or "none" bbox: ensure the Ollama server URL and model are correct and that the server is reachable (check OLLAMA_URL and OLLAMA_MODEL env vars).

---

## Tests & Validation
- There are no formal unit tests in this repository. Validation is intended to be performed on hardware with the Interbotix perception stack running and the camera properly calibrated.
- Recommended validation steps:
  1. Start perception stack and confirm TFs (RViz).
  2. Visualize clusters using the pointcloud tuner and confirm cluster centers.
  3. Run pick_place.py to verify safe pick/place cycles on simple objects.

---

## Contributing
- Contributions welcome. For code additions:
  - Follow ROS 2 and Python style used in existing scripts.
  - Add documentation for new scripts or dependencies in documentation.txt and update this README.
- Opening issues: include logs, which script you ran, exact TF warnings, and camera topic names.

---

## License
This repository includes a LICENSE file at the top level. Check LICENSE for allowed usage and attribution.

---

## Acknowledgements
- Interbotix / Trossen Robotics for the XS arm ROS packages and perception modules.
- Ollama for local LLM vision model integration used in ollama_pick_place.py.

---

## Where to look next (files of interest)
- documentation.txt — longer notes and setup hints.
- scripts/pick_place.py — baseline pick & place flow.
- scripts/color_sorter.py — color-based sorter logic and hue thresholds.
- scripts/ollama_pick_place.py — integration with Ollama vision model and camera projection code.
