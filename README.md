# camera_lane_motor
Single line patrol

open camera and pub img
$ ros2 run lane_detection camera 

hsv right and pub error
$ ros2 run lane_detection right_line

sub error to motor
$ ros2 run motor_control motor_subscriber
