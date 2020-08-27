# ros-maxon-driver

# Steps to get the package working


1 - Clone the [maxon-des library](https://github.com/tmralmeida/maxon_des):
```
git clone https://github.com/tmralmeida/maxon_des
```


2 - Install doxygen by following these [steps](https://www.tutorialspoint.com/how-to-install-doxygen-on-ubuntu).


3 - Inside of the library directory run the commands:
```
mkdir build && cd build
```

```
cmake ..   
```

```
sudo make install
```


4 - Then, clone this ROS driver that enables to send commands from a joystick to the MAXON_DES 70/10.   



5 - To launch the node that controls the joy buttons and convert them into messages to the motor:
```
roslaunch driver_maxon atlas_joy.launch
```


6 - To run the node that sends the angle to the Servo:
```
rosrun rosserial_python serial_node.py _baud:=57600 _port:=/dev/ttyACM0
```
