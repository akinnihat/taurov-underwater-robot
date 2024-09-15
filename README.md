# underwater-robot-mec319

This software has been developed in order to control an Autonomous Underwater Vehicle (AUV) for object detection and navigation tasks, specifically tailored for competing in the Teknofest Unmanned Underwater Systems Advanced Category. The project has evolved over time, utilizing various hardware configurations such as Raspberry Pi (2021) and Jetson Nano (2022) and different vehicle frames. Key hardware components include Pixhawk for flight control and Arduino. The software stack is developed using Python and C++ within the ROS2 framework, leveraging libraries such as OpenCV for computer vision tasks and Pymavlink for communication with the flight controller.

#### Dependencies
- ROS2
- OpenCV
- pymavlink
- brping

```
ros2 launch mec319_bringup mission.launch.py
```
First variant (2021) uses the frame with 6 thruster (with side-by-side vertical thrusters) and the second variant (2022) uses the frame with 8 thruster (configuration with 6-DOF control) (Frame images source: [ArduSub](http://www.ardusub.com/introduction/features.html)) 

<img src="https://github.com/user-attachments/assets/f438094a-b4f1-4740-9121-82ef253ec0d3" width="256">


### TauRov 2021





https://github.com/user-attachments/assets/4b8badfb-5e8d-488d-a757-ead8604f074b

### TauRov 2022
https://github.com/user-attachments/assets/d1d63afb-c27f-42e4-bb78-a12bbe54992d





