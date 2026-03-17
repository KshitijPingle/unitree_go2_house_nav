# unitree_go2_house_nav
Add more details here  

## ROS2 Distribution  
Humble  

## Setup Instructions  
Git clone this repo using:  
`git clone https://github.com/KshitijPingle/unitree_go2_house_nav.git`    

Add third party dependencies using rosdep   
`rosdep install --from-paths src --ignore-src -y -r`  

Then build the project  
`colcon build --symlink-install`  

Always source the setup.bash before running scripts  
`source install/local_setup.bash`  

## Launch Navigation (IN PROGRESS)  
`ros2 launch go2_core go2_start.launch.py`  

Additionally, control the robot using your own keyboard with:  
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`  


## Node Documentation
1. Node 1  
Clearly List the following for each node  
- Subscribed Topics  
Name and Type   
- Published Topics   
Name and Type  
- Parameters  
List all parameters and their default values if any  

2. Node 2
