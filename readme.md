# Bug_planner
---
This package is modified from [motion_plan](https://bitbucket.org/theconstructcore/two-wheeled-robot-motion-planning/src/master/) to be compaitable with robot with 360 degrees laserscan in pybullet environment.

Modifications made:
- Used odometry for localisation, don't need to give initial values 
- subscribes to /goal topic (PoseStamped msg)
- Ability to change topic names and velocity limits in the launch file.

### Note:
Only bug2 algorithm extracted from original package. 

## Installation
---
Download this package in your workspace and catkin_make it.
	
	$ cd catkin_ws/src
	$ git clone
	$ cd .. && catkin_make

## Parameters
---
You can change parameters and topic names from the launch file by following the format in bug2.launch.

### Topic Names

- cmd_vel
- scan 
- goal
- odom

### Velocity Limits

- turn_vw: angular velocity of the robot when it is positioning itself to the goal
- go_vx: linear velocity of the robot when it is moving to the goal
- go_vw: angular velocity of the robot when it is re-positioning while moving to the goal

## Using bug_planner
---

	$ roslaunch bug_planning bug2.launch 

### Note:
If you encounter the following error when running bug2.launch: 
	
	ImportError: dynamic module does not define module export function (PyInit__tf2)

Refer to this [link](https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/) and follow the instructions. 

Do not make another catkin_ws, just start from **wstool init**.
