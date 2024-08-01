# Bridge Between ROS1 Noetic and ROS2 Foxy Using the Robot Arm Package
#### 
## Task Description:-
Use ros1_bridge to print a topic from ROS1 (Noetic) to ROS2 (Foxy)
####
## A Step-By-Step Guide:-
### Create a workspace for ROS1 Noetic and ROS2 Foxy
####
#### ROS1 Noetic:-
1- Set Up the Catkin Workspace:
#### 
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
#### 
![catkin-make(2)](https://github.com/user-attachments/assets/a71a3f49-c8f2-4dc0-8f74-0a4e7eb04cdf)
#### 
#### Install Arduino_robot_arm package in ROS1 Noetic:-
#### 
1- Clone the "arduino_robot_arm" repository into the just created Catkin workspace's "src" directory:
####
```bash
cd ~/catkin_ws/src
git clone https://github.com/smart-methods/arduino_robot_arm.git
```
#### 
2- Install the dependencies:
#### 
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
#### 
![clone-github-to-catkin(3)](https://github.com/user-attachments/assets/7f1d16ee-1ec2-4236-93c2-5b949738b8a4)
#### 
3- Build the package:
#### 
```bash
cd ~/catkin_ws
catkin_make
```
#### 
![buildpackage(4)](https://github.com/user-attachments/assets/ae5a5876-a1cd-42c0-b3c4-05e407f58a23)
#### 
4- Source the workspace:
#### 
```bash
source ~/catkin_ws/devel/setup.bash
```
#### 
5- You can verify the installation by listing the available packages and checking if "arduino_robot_arm" is listed:
```bash
rospack list | grep arduino_robot_arm
```
#### 
6- Run the package:
#### 
```bash
roslaunch arduino_robot_arm check_motors.launch
```
####
![roslaunch-arm(6)](https://github.com/user-attachments/assets/d2a8a610-f4cf-4f82-b63c-8132b7221ecb)
#### 
![ROBOARM](https://github.com/user-attachments/assets/fd9b1f50-be0d-437f-bcbe-ca5d7802dc77)
#### 
#### ROS2 Foxy:-
1- Source the ROS2 Foxy Setup Script:
####
```bash
source /opt/ros/foxy/setup.bash
```
#### 
2- Create the ROS2 Workspace:
#### 
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
#### 
3- Build the Workspace:
####
```bash
colcon build
```
#### 
4- Source the Workspace Setup Script:
#### 
```bash
source install/setup.bash
```
#### 
![source-ros2-mkdir-ros2src(7)](https://github.com/user-attachments/assets/9c74ee4e-a97f-45bf-9833-4565549fdf23)
#### 
### Create "ros1_bridge" workspace and clone the package
#### 
1- Source ROS 1 and ROS 2 Setup Scripts:
#### 
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
```
#### 
2- Create the ros1_bridge Workspace:
#### 
```bash
mkdir -p ~/ros1_bridge_ws/src
cd ~/ros1_bridge_ws/src
git clone -b foxy https://github.com/ros2/ros1_bridge.git
```
#### 
3- Install Dependencies:
#### 
```bash
cd ~/ros1_bridge_ws
rosdep install --from-paths src --ignore-src -r -y
```
#### 
4- Build the Workspace:
#### 
```bash
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```
#### 
![colcon-build-ros1bridge-all](https://github.com/user-attachments/assets/c1023ace-d54f-46a6-8fca-1f60141ce9e4)
#### 
5- Source the Workspace:
#### 
```bash
source install/setup.bash
```
#### 
