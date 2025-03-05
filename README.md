# camera_lane_motor
Single line patrol

open camera and pub img
```bash
ros2 run lane_detection camera
```

hsv right and pub error
```bash
$ ros2 run lane_detection right_line
```


sub error to motor
```bash
$ ros2 run motor_control motor_subscriber
```
