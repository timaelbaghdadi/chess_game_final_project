**ros1920-final-project**

This metapackage contains the repositories for the development of the final project of the course [Introduction to ROS](https://sir.upc.edu/projects/rostutorials) (*ETSEIB-Universitat Politècnica de Catalunya*)

* **chesslab_setup**: Project scene setup and seerver for basic planning services
* **ff**: ROS wrapper to the Fast Forward Task Planner
* **UR3-IK**: Inverse Kinematics of the UR3 robot
* **realsense-ros**: ROS Wrapper for Intel RealSense Devices
* **realsense_lab**: Realsense extrinsic parameters calibration
* **aruco_ros**: Software package and ROS wrappers of the Aruco Augmented Reality marker detector library (forked from pal-robotics/aruco_ros)
* **universal_robot**: This repository provides ROS support for the universal robots (forked from ros-industrial/universal_robot)
* **Universal_Robots_ROS_Driver**: Driver enabling ROS operation of UR robots. 
* **robotiq**: ROS-Industrial robotiq meta-package (forked from ros-industrial/robotiq)
* **my_chesslab_setup**: Project with the actions implemented and published in gazebo and Rviz.


Authors: Lluis Ballber, Fàtima el Baghdadi.

This project is based on ROS, where the users can play the chess using a robot (1 robot of each user). Several packages are provided by the teachers and a new package called 'my_chesslab_setup' where several services have been created to implement the actions.


There are 3 modules: manager module, planning module and sensing module.

The menager module call the services of planning module to compute the trajectories and publish them in Rviz and Gazebo, also call the services of sensing module to obtain the position of the cell and the pieces of the chess.

The services implemented are: 
Rosservices of sensing module node:

piece2cell: rosservice that is subscribed to the cameras of the gazebo and we obtain the position and the ID aruco. Note that the arucos on the right side are subscribed to the camera on the right  and those of the left side are subscribed to the camera on the left. This rosservice returns the aruco ID, the position, and the cell. There is a video that shows this demo.

Cell2pose: rosservice that receives a cell like B4 and returns the position of the cell.

position_srviz: rosservice that we give the piece, the state ‘give’ that represents if we are saving the new position of the piece or if we are obtaining the last position saved. It returns the ID of the piece, the position (in case we want to read the last position) and the equivalent name of the piece in gazebo (like pawnB6).

Rosservices of planning services node:

The rosservice move_trajectory goes from an initial point to a final point computing the inverse kinematic and using the TransformStamped
to convert the position (that is with respect to the world) into a relative position with respect to the base link of the robot.

The rosservice desceding_trajectory goes from an initial height to another height mantainting the orientation of the gripper and the position x and y and computing the inverse kinematic (the movement is rectilinear trajectory in cartesian). In this resservice at the end of the trajectory, it is executed the attachment or dettachment of the piece that depends on the boolean variable “attached” that is passed to the function. It also uses TransformStamped
to convert the position (that is with respect to the world) into a relative position with respect to the base link of the robot.

The rosservice ascending_trajectory does the same as the previous one but the attach or dettach is during all the trajectory to maintain the piece in the gripper or not depending on the passed variable attached.



Rosservices of planning services node:

The rosservice move_action computes the movement given an initial and final position. It calls the services: move_trajectory, desceding_trajectory, ascending_trajectory to compute the trajectories and to publish them in gazebo and rviz. 

The rosservice kill_action computes the movement given the initial place of the team selected and the piece of the rival team. To do that it need a position called “safety_pose” to leave there the killed cell that is out the chessboard.  It calls the services: move_trajectory, desceding_trajectory, ascending_trajectory to compute the trajectories and to publish them in gazebo and rviz.


The rosservice castle_operation computes the movement given the initial place ‘king’, the position where to place it and  the ‘tower’ piece and  the position where to place it also. It also calls the services: move_trajectory, desceding_trajectory, ascending_trajectory to compute the trajectories and to publish them in gazebo and rviz.


Notes of the simulation:

To visualize the simulation we have to call the launch “demo_services_game” and run the 
node manager node. Once we have done, a menu is visualized, we have to choose the team 
and the action.
When we run only the rviz, the trajectory and the configurations are updates tanks to the rosservice setrobconf. When we run at the same time Rviz and Gazebo, the rviz subscribes from the Gazebo (that uses the rosservice “joint_trajectory_controller_with_gripper” to publish the trajectory) and at the same time when it is updated the rviz, it also check the collision with the rosservice “setrobconf”. We controll the gripper as it another joint of the robot. 





[Final Work Webpage](https://sir.upc.edu/projects/rostutorials/final_work)
