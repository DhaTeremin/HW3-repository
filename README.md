# HW3 repository - Fly your drone
To correctly setup the model, it is necessary to clone the PX4-Autopilot and px4_msgs repositories inside the hw3_ws directory beforehand.
````
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
git clone https://github.com/PX4/px4_msgs.git
````
For both of the cloned repos, the correct version to checkout is v1.16.
````
cd PX4-Autopilot
git checkout v1.16.0
````
````
cd px4_msgs
git checkout release/1.16
````
The px4_msgs needs then to be built.
````
colcon build --packages-select px4_msgs
````
Setup the overlay
````
. install/setup.bash
````
Then it is mandatory to start the bridge and allow the communication between ros2 nodes and PX4 uOrbs.
````
./DDS_run.sh
````
The last step is to open QGround.
````
./QGroundControl-x86_64.AppImage
````
## Build your custom drone
To start the custom drone
````
cd PX4_Autopilot
make px4_sitl gz_hw3_drone
````

## Modify force_land node
To run the modified force_land node, it is first necessary to open a new terminal and then source the overlay and run the ros2 node while keeping running the drone terminal, the DDS bridge terminal and QGround
````
colcon build --packages-select force_land
. install/setup.bash
ros2 run force_land force_land
````
## Design trajectory waypoints
To run the trajectory planned, it is first necessary to open a new terminal and then source the overlay and run the ros2 node while keeping running the drone terminal, the DDS bridge terminal and QGround

````
colcon build --packages-select offboard_rl
. install/setup.bash
ros2 run offboard_rl trajectory_planner
````



