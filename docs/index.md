# Interbotix XS Perception Notes

In this Project I have used the Interbotix xsarm perception toolbox and the Ollama `moondream` model to control the WidowX250s arm. The Interbotix perception package is a ROS package provided by Trossen/Interbotix that publishes clusters and an AR tag transform which these scripts use.

## Quick run hints

1. Launch the interbotix xsarm perception package with AR tag tuner and pointcloud tuner GUIs:

```bash
ros2 launch interbotix_xsarm_perception xsarm_perception.launch.py \
  robot_model:=wx250s use_armtag_tuner_gui:=true use_pointcloud_tuner_gui:=true
```

2. Use the ARTag tuner GUI to align the arm tag so it is upright and not inverted when seen by the camera. Press "snap" to update transforms in RViz.

3. Use the PointCloud tuner GUI to set crop-box filters and remove planes so that cluster detection is robust.

4. The pick & place demo (scripts/pick_place.py) uses cluster centers exposed by the perception node. The color sorter demo (scripts/color_sorter.py) uses cluster['color'] hue checks. The Ollama demo (scripts/ollama_pick_place.py) sends a resized 640x480 image to a local Ollama server and expects a JSON bbox response.

## Camera & TF notes

- Ensure your camera publishes CameraInfo and Image topics used by the scripts (defaults in scripts/ollama_pick_place.py):
  - /camera/camera/color/camera_info
  - /camera/camera/color/image_raw

- The AR tag transform is consumed to compute an exclusion radius (avoid picking the arm itself).

## Calibration

- Place the AR tag on the arm and position it under the camera for calibration. The scripts attempt to locate the AR tag and will issue a warning if TFs are not yet available.

