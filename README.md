# NegativeStiffnessHoneycomb
## UR5 modern driver: improving usability as well as enabling compatibility of ros_control
```
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.0.101
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.101
```
## moveit: planning capability
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch	 
```

## How to change initial position and orientation of UR5
* ### manually move the robot via UR5 teach pendant
* ### check `rostopic echo /joint_states` 
* ### copy the position to *move_robot_back.py*
* ### source your environment
```
source ~/rustamch/devel/setup.bash 
```
* ### Run your python script 
``` 
rosrun calibrate_rgb_sensor_rustam move_robot_back_rustam.py
```
or
```
rosrun calibrate_rgb_sensor_rustam move_robot_back_rustam_light.py
```
#
# How to RUN experiment

## Listener for the UR5
```
rosrun move_ur_sim desired_velocities_node
```
## Main programs for experiments
```
rosrun calibrate_rgb_sensor_rustam ur5_nsh_experiment
rosrun calibrate_rgb_sensor_rustam velcontrol
rosrun calibrate_rgb_sensor_rustam light_touch
```

#
# WEISS Sensor

### weiss_kms40/launch/config.yaml
```
ping 192.168.1.30
source ~/rustamch/devel/setup.bash
roslaunch weiss_kms40 kms40.launch
```
### if everything is ok:
```
[ INFO] [1632746849.394712440]: Trying to connect to KMS40 ...
[ INFO] [1632746849.395446253]: Connection established. Starting data acquisition.
```

## see plot:
### rqt -> matplot -> topic: /weiss_wrench/wrench/force/z


#
# WITTENSTEIN Sensor
```
rosrun wittenstein wittenstein_main 
[ INFO] [1647871387.851390016]: *****Connecting*****
```
### check: 
```
rostopic echo /wittenstein_topic
```



#
# Rosbag write
* ### create .msg message in /src/my_msgs/msg
* ### CMakeLists.kz: 
```
add_message_files(
  your_message.msg
)
```
#
# bag to csv
```
rostopic echo -b myfile.bag -p /myfile > /home/rustam/catkin_ws/.../myfile.csv
```

# Deleting repeating rows from csv
```
awk -F, 'seen[$3]++{print $0 > "dups.csv"; next}{print $0 > "new.csv"}' mycsv.csv
```
### seen[$3]: which column we are looking at
### dups.csv: all duplicates from mycsv.csv
### new.csv: raw with unique column3 elements
https://stackoverflow.com/questions/25393281/linux-bash-commands-to-remove-duplicates-from-a-csv-file

#
# Github process
### token: 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
``` 
git init

git add processed10trials/csvmultipleplot.py OR git add processed10trials/

git commit -m "3rd commit"

git push -u origin_rustam main
```


#
# ROSSERIAL
### /home/rustam/snap/arduino/current/Arduino
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1 _baud:=115200
rosrun hallsensor hallsensor_listener
```

#
# Arduino message .msg to header .h
### 1. put .msg to /home/rustam/catkin_ws/src/rosserial/rosserial_arduino/msg/your_message.msg
### 2. /home/rustam/catkin_ws/src/rosserial/rosserial_arduino/CMakeList.txt:
```
add_message_files(FILES
  Adc.msg
  hallsensor.msg
  hallsensor_teensy.msg
  your_new_message.msg
)
```
### 3. catkin build
### 4. run make_library script:
```
rosrun rosserial_arduino make_library.py <arduino_library_path> <message_pkg_name>
```
### <arduino_library_path> : is the absolute path to the Arduino library.
### <message_pkg_name> : is the name of the precompiled package containing your custom crazy_msg.

## example:
```
rosrun rosserial_arduino make_libraries.py /home/rustam/Arduino/libraries /home/rustam/Arduino/libraries/rosserial_arduino
```
### A new ros_lib folder is created where all compiled libraries
### *No difference with: rosrun rosserial_arduino make_libraries.py /home/rustam/Arduino/libraries*
#
## vscode crash:
*crash of vscode week 1-6 april 2022
Version: 1.66.0
Commit: e18005f0f1b33c29e81d732535d8c0e47cafb0b5
Date: 2022-03-30T05:50:41.156Z
Electron: 17.2.0
Chromium: 98.0.4758.109
Node.js: 16.13.0
V8: 9.8.177.11-electron.0
OS: Linux x64 5.4.0-107-generic snap*


#
# MATLAB
 
https://www.mathworks.com/matlabcentral/answers/20-how-do-i-make-a-desktop-launcher-for-matlab-in-linux
