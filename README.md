# Robot Control in ROS and Gazebo

Two control projects from the course **Robotics II** at the National Technical University of Athens (NTUA), spring 2021. Both run in simulation with **ROS Noetic** and **Gazebo**, written in Python.

The course provided the simulation environment: the robot models, the Gazebo worlds and a skeleton for each ROS node (topics, publishers and subscribers). The control code in the two scripts below is my own work. The assignment descriptions are in `src/` (`Appendix-*.pdf`).

## 1. Redundant manipulator: trajectory tracking with obstacle avoidance

Package: `src/robo2_redundant` · Node: `scripts/controller.py` · Kinematics: `scripts/kinematics.py`

The 7-joint **xArm7** moves its end-effector back and forth between two points while two obstacles move nearby. The arm has more joints than the task needs, so the spare freedom is used to keep its body away from the obstacles.

- **Kinematics:** homogeneous transformation matrices for each joint and the arm's Jacobian.
- **Trajectory:** a cubic polynomial between the two points, so position and velocity are smooth and the arm starts and stops at rest.
- **Primary task, tracking:** joint velocities from the Jacobian pseudoinverse, with a feedback term on the end-effector position error.
- **Secondary task, obstacle avoidance:** a velocity that pushes joints 3 and 4 away from the obstacles, projected into the null space of the Jacobian. That way it never disturbs the end-effector's path.
- **Monitoring:** end-effector position and tracking error, and the distance of joints 3 and 4 from each obstacle, are published on their own topics for plotting.

Run:
```bash
roslaunch robo2_redundant redundant.launch
```

## 2. Mobile robot: wall following

Package: `src/robo2_mobile` · Node: `scripts/follower.py`

A differential-drive robot with five sonars (front, front-left, front-right, left, right) and an IMU has to find a wall and then follow it at a fixed distance.

- **State machine:** drive straight until the front sonar sees a wall, turn in place until the way ahead is clear, then follow the wall on the right. If a wall appears ahead again (a corner), it turns again.
- **PD controller:** during following, the angular velocity comes from the errors of the right and front-right sonars against their target distances, with a proportional and a derivative term. The output is limited, so the robot never turns too sharply.
- **Monitoring:** sonar errors and the commanded velocities are published for plotting.

Run:
```bash
roslaunch robo2_mobile mobile_partA.launch
```

## Setup

A catkin workspace on Ubuntu 20.04 with ROS Noetic and Gazebo, plus the course's robot description packages, which are not included here. Clone this repository into the workspace, then:

```bash
catkin_make
source devel/setup.bash
```

The build/ and devel/ folders are the pre-built workspace as provided by the course, kept unchanged.
