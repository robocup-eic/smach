# Main repo for RoboCup@Home task
Task available
1. GPSR
2. Carry my luggage
3. Storing groceries
4. Receptionist
5. What is that?
6. Tidy up

# Installation
1. $ catkin_make at walkie2_ws
2. download meshes.zip from https://drive.google.com/file/d/1WM-3rvoMUGFi42VdVNdGbP3F58k13_vk/view?usp=sharing and extract to walkie2_ws/src/cr3_lowlevel_pkg/meshes

# walkie1_ws
1. $ roslaunch walkie_bringup walkie_bringup.launch
2. $ roslaunch walkie_navigation walkie_navigation.launch 
3. $ python <smach program>