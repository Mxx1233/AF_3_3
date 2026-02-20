# Main How-To:


## Terminal 1 - Start camera
```
cd ws-template
colcon build
ros2 launch realsense2_camera rs_launch.py
```

## Terminal 2 - Lane Detection
```
cd Rusty-Racer/wise-2025-26-gruppe-c/src
colcon build
source install/setup.bash
ros2 run psaf_lane_detection_outside lane_detection_outside
```

## Terminal 3 - Trajectory Plan
```
cd Rusty-Racer/wise-2025-26-gruppe-c/src
colcon build
source install/setup.bash
ros2 run psaf_trajectory_plan trajectory_plan
```

## Terminal 4 - Main run (Now include starting camera)
```
cd Rusty-Racer/wise-2025-26-gruppe-c
colcon build
source install/setup.bash
ros2 launch psaf_launch main_psaf1.launch.py
```

## Optional - Monitor trajectory parameters
`ros2 topic echo /lane_deviation`

## Terminal 5 - To collect the data form the car
cd Rusty-Racer/wise-2025-26-gruppe-c
source install/setup.bash
python3 monitor_control.py 40

## Terminal 6 - To plot the data
source ~/venv/bin/activate
python3 plot_control.py data/the name of the data file.csv
deactivate

=============================================================================
=============================================================================


=============================================================================
=============================================================================

Regarding the /lane_deviation and what the fields mean:
- ros2 topic echo /lane_deviation
- lateral_error: how far left or right off the center of the lane.
    - POSITIVE = we are right of center lane.
- heading_error: 
    - NEGATIVE = we are pointing left of lane direction.
- curvature: if there is a curve in front (currently not used anywhere).


=============================================================================
=============================================================================

To calibrate the camera:

- For the python environment: source /home/psaf/pycamcal-env-py310/bin/activate
- Then follow the guide from GitHub
- start that one tool, select RGB then take a snapshot
- save it somewhere and run the python code on it

=============================================================================
=============================================================================
## To work with the log_viewer debug tool - Foxglove:
- to record every topic do (this is very memory intensive, so delete old recordings if not needed):
```
ros2 bag record -a
```

- to record specific ones, replace "-a" with the topic name, e.g. /camera/camera/color/image_raw
- intereting topics:
    - /camera/camera/color/image_raw
    - /control/debug_overlay
    - /lane_detection/debug_overlay
    - /trajectory/debug_overlay
e.g.:
ros2 bag record /control/debug_overlay /lane_detection/debug_overlay /trajectory/debug_overlay

=============================================================================
=============================================================================

