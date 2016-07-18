# Romeo Whole body control demos

## Gaze control demo

### Description:
Romeo will try to keep in the center of the image the target, using a whole body control.

#
* Set Romeo initial pose:
`$ cd /scripts`
`$ python setRomeoHalfSittingPose.py`
* Launch camera node: 
`$ roslaunch romeo_sensors_py camera_romeo_rennes.launch` 
* Launch Whycon tracker
`$ roslaunch romeo_mc whycon_romeo.launch`
* Start velocity controller:
`$ roslaunch visp_naoqi_ros romeo_cmd_vel.launch`
* Start gaze demos:
`$ roslaunch romeo_mc test_gaze_task_romeo_tracker.launch tracker_type:=whycon`

