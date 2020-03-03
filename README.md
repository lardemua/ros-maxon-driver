# ros-maxon-driver
This is the ROS driver that enables to send commands from a joystick to the MAXON_DES 70/10.   

To launch the node that controls the joy buttons and convert them into messages to the motor:
```
roslaunch driver_maxon atlas_joy.launch
```

To run the node that sends the angle to the Servo:
```
rosrun rosserial_python serial_node.py _baud:=57600 _port:=/dev/ttyACM0
```
