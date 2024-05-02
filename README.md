# Feeling Drones 
This repository contains code that was used by me (Anish Jadoenathmisier) to complete my thesis at the TU Delft. I used ROS2 Humble as my main ROS2 framework and Python 3.10 was used to write the Python ROS2 nodes, while CMake version 3.8 
was used to compile the C++ Ros2 nodes. 
The repository is broken down into the following folders:
* custom_msgs
* Manipulator
* offboard
* raspberry-pi-code
* ros2-app
* teensy-code
## Custom_msgs
This folder contains the ROS2 packages that are needed to compile code that utilizes custom msgs that were defined during the project.
To be more specifc, I wanted to have timestamps at each MultiArray msg thus this custom msgs gives us that ability.
## Manipulator
Here some basic kinematics and visualization can be found on the 3 joint manipulator, furthermore some scripts on how the sizing of
components such as springs were performed can be found.
## Offboard
All plotting scripts can be found in this folder, from ros2 bag files you should be able to generate plots using these scripts.
## Raspberry-Pi-Code
The core of the code that was running on the Raspberry Pi can be found here, this includes the drivers for the touch sensor and the 
servo motors. Furthermore this folder contains a docker file that allows you to create a docker image which can be run on a Raspberry Pi 
## Ros2-App
In the ros2-app folder you can find the bulk of the tactile control algorithm, interfaces between px4 and ROS2 and launch files
with which a simple simulation can be run (to verify drone movements). A small explanation on the controllers, data_mocker, simulation is as follows:
### controllers 
This folder contains the code that is used for the state machine onboard the drone. As long as you are able to accuractly localize the drone, 
you can use this code to run ROS2 nodes that generate trajectories for the drone to fly. Furthermore a state machine that uses 
tactile information is found in this folder
### data_mocker
From here you can run various nodes that mock data that is normally sent by hardware, you can use this for debugging or other usecases. At the moment there are 
only mockers for the tactile sensors and the servos
### simulation
Running the launch file in the folder should give you the ability to simulate the behaviour of the state machine, combined with the data_mocker you can then 
verify whether the drone is doing what you want it to do
## Teensy Code
Here all the code that was put on the Teensy can be found, keep in mind that to upload this code to the Teeny you will need somehting like the Arduino IDE. 

## Contact Info
The structure might not be the most optimal and the code itself does not always follow the DRY principle, nevertheless I think that it should be
reasonably understandable what is going on in the code
If you have any questions regarding the code, dont hesitate to send me an email on ajadoe@outlook.com
