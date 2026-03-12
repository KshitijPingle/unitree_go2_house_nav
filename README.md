# unitree_go2_house_nav
Add more details here  

## ROS2 Distribution  
Humble  

## Setup Instructions  
Git clone this repo using:  
`https://github.com/KshitijPingle/unitree_go2_house_nav.git`    

Add third party dependencies using rosdep  
`rosdep install --from-paths src --ignore-src -y -r`  

Then build the project  
`colcon build --symlink-install`  

Always source the setup.bash before running scripts  
`source install/setup.bash`  


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
