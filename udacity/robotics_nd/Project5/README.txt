This project simulates a home service robot using the robot created in previous projects. 
The following 3 steps are done in order for the home service robot to function.

1. Mapping
The environment is first mapped using SLAM. Use the scripts/test_slam.sh to initialize the environment.
The test_slam.sh 
a. Launches the world file and places the robot in the world.
b. Creates the mapping node.
c. Launches the teleop_twist_keyboard for robot navigaton.
d. Launches rviz.

Use the teleop_twist_keyboard to navigate the robot and generate a map. 
Once an acceptable map is generated in rviz, save the map using the command `rosrun map_server map_saver`.
The generated map from my run is saved under map/ directory.


2. Localization
Check that the robot can localize itself correctly using scripts/test_navigation.sh.
The test_navigation.sh
a. Launches the world file and places the robot in the world.
b. Uses amcl.launch and the map file generated in the previous step to specify the start location for the robot.
It also specifies the localization parameters.
c. Launches rviz.

Use the 2DNavigation goal to check that the robot is localizing itself correctly.


3. Navigation
a. pick_objects provides 2 goal locations for the robot. The first is a pick-up location 
where the robot picks up a virtual object. The second is the drop-off location where the robot 
drops off the virtual object.
b. add_markers places the virtual object in the pick-up location. It subscribes to the robot
odometry to determine the location of the robot. Once the robot reaches the pick-up location, it 
removes the marker and places it again in the drop-off zone once the robot reaches the drop-off zone.

The scripts/home_service.sh
a. Launches the world file and places the robot in the world.
b. Uses amcl.launch and the map file generated in the mapping step to specify the start location for the robot.
It also specifies the localization parameters.
c. Launches rviz.
d. Starts the pick_objects and add_markers nodes.
The robot receives the first goal - the pick-up location, where the marker is present. It plans a path
to that location. Once it reaches there, the marker is removed, and the drop-off location is received.
The robot plans a path to the drop-off location and drops off the marker there.

