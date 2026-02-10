# traffic_sign_yolo

## Runtime Notes (ROS 2 Jazzy)

## Create venv

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
```

This package uses `cv_bridge`, which (in Jazzy) is built against NumPy 1.x.
To avoid import errors, keep NumPy below 2 and OpenCV below 4.12.

Recommended venv installs:

```bash
python3 -m pip install "numpy<2" "opencv-python<4.12" ultralytics colcon-common-extensions
```

System dependencies (recommended):

```bash
rosdep install --from-paths src --ignore-src -r -y
```

If you use `ros2 run`, build with your venv Python so the entry script points
to the venv interpreter:

```bash
python3 -m colcon build --symlink-install --packages-select traffic_sign_yolo
source install/setup.bash
```

## Run

### Neubauen und Run

```bash
python3 -m colcon build --symlink-install --packages-select traffic_sign_yolo
source install/setup.bash
ros2 run traffic_sign_yolo detect_traffic_sign --ros-args -p publish_debug_image:=false
```


Full example with all parameters:

```bash
ros2 run traffic_sign_yolo detect_traffic_sign --ros-args \
  -p image_topic:=/camera/realsense2_camera_node/color/image_raw \
  -p detections_topic:=/traffic_sign/detections \
  -p debug_image_topic:=/traffic_sign/debug_image \
  -p model_path:=src/traffic_sign_yolo/model/best_run_1_long_30_01_2026.pt \
  -p conf_th:=0.35 \
  -p iou_th:=0.45 \
  -p imgsz:=512 \
  -p device:=cpu \
  -p publish_debug_image:=true \
  -p infer_hz:=15.0
```

Only override the debug image publishing:

```bash
ros2 run traffic_sign_yolo detect_traffic_sign --ros-args -p publish_debug_image:=false
```

Model file `src/traffic_sign_yolo/model/best_run_1_long_30_01_2026.pt` must
exist on the target system (it may be gitignored).


## Beispiel Message
 ```
 psaf@psaf-nuc-3:~/Rusty-Racer/wise-2025-26-gruppe-c$ ros2 topic echo /traffic_sign/detections --once
header:
  stamp:
    sec: 1770284269
    nanosec: 899821289
  frame_id: camera_color_optical_frame
detections:
- header:
    stamp:
      sec: 1770284269
      nanosec: 899821289
    frame_id: camera_color_optical_frame
  results:
  - hypothesis:
      class_id: stop_sign
      score: 0.9466321468353271
    pose:
      pose:
        position:
          x: 0.0
          y: 0.0
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
      covariance:
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
      - 0.0
  bbox:
    center:
      position:
        x: 599.9803466796875
        y: 187.14163970947266
      theta: 0.0
    size_x: 195.297607421875
    size_y: 201.0995330810547
  id: ''
---
```
