# kinova_door_opening_simulation
Basic trajectory planning using moveIt! to simulate door opening motion using a Kinova Arm (Jaco 2 - 7 DOF) in Gazebo.

## User Guide for Kinova Jaco2 7DOF Arm 

### Creating Workspace 

To avoid running into build error issues, I recommend creating a dedicated workspace for all kinova packages. Run the following commands in your terminal to create a new catkin workspace: 
  1. Source your environment 
  ```
  $ source /opt/ros/melodic/setup.bash 
  ```

  **replace melodic with your ros distro**

  2. Create and build workspace 
  ```
  $ mkdir -p ~/kinova_ws/src 
  $ cd ~/kinova_ws/ 
  $ catkin_make 
  ```
  3. Source the setup file in the workspace 
  ```
  $ cd ~/kinova_ws/ 
  $ source devel/setup.bash 
  ```
### Installing Packages 

  1. Install MoveIt package from the terminal using the following: 
  ```
  sudo apt-get install moveit* 
  ```
  2. Git clone the kinova-ros package in the src folder of your workspace and 
  catkin_make using the following commands: 
  ```
  $ cd ~/kinova_ws/src $ git clone https://github.com/Kinovarobotics/kinova-ros.git kinova-ros 
  $ cd ~/kinova_ws $ catkin_make 
  ```
  
  Now your system is ready to run kinova-ros package. You can spawn the Kinova arm in Gazebo and visualize the arm in rviz using ROS. 
  
  **3. Our next goal is to Fix the Grasping issue using graspit-pkg. You can find the installation guide for graspit-pkg at https://github.com/JenniferBuehler/graspit-pkgs/wiki/Installation 
  
  *Install the graspit-pkg and its dependencies in the kinova-ros workspace that you have build.*
  
  **The xacro file for Grasp Fix Plugin is included in our commit.**
  
  4. Create a new package called kinova_scripts inside the kinova-ros folder using the following commands: 
  ```
  $ cd ~/kinova_ws/src/kinova-ros/ 
  $ catkin_create_pkg kinova_scripts std_msgs rospy roscpp 
  $ cd ~/kinova_ws 
  $ catkin_make 
  ```
  
  5. Next step is to **clone our repository** 
  
  6. Merge the files in the correct folder structure of the kinova-ros package.
  
  *For convenience, we have made sure to commit the files in required folder structure.*
 
### Running the setup
  
  1. Launching the Kinova Arm in a terminal window.
  ```
  roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2s7s300
  ```
  2. Launch moveIt configurations in a seperate terminal window.
  ```
  roslaunch j2s7s300_moveit_config j2s7s300_gazebo_demo.launch
  ```
  3. Spawn the door in a seperate terminal window.
  ```
  roslaunch kinova_description door_spawn.launch
  ```
  4. Navigate to the kinova_scripts directory and run the kinova_path_planning.py using: 
  ```
  $ cd kinova_ws/src/kinova-ros/kinova_scripts/src/ 
  $ ./kinova_path_planning.py
  ```

### Description of kinova_path_planning.py script

The simple door model is considered. Door Handle waypoints are calculated on the circular path that a door is bound to follow. 
Following with the motion planning is done using the RRT*  algorithm.

