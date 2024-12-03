# ros2_differential_drive_line_following
A ROS2 package for differential drive vehicles containing an action server with time-optimal line-following dynamics.

## Overview
This package contains a ROS2 action server for differential drive vehicles using time-optimal line-following with VICON motion capture localization.

## License
This package is released under an [MIT License](https://github.com/marinarasauced/ros2_differential_drive_line_following/blob/main/LICENSE).

**Authors:** [Marina Nelson](https://github.com/marinarasauced) <br/>
**Affiliation:** [ACE Lab](https://rvcowlagi-research.owlstown.net/) <br/>
**Maintainer:** Marina Nelson, marinarasauced@outlook.com

This package has been tested with TurtleBot3s in ROS2 Humble on Ubuntu 22.04.

## Installation
### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- [mess2_msgs](https://github.com/marinarasauced/mess2_msgs)
- [mess2_plugins](https://github.com/marinarasauced/mess2_plugins)

These instructions assume that you meet the above prerequisites. Do not proceed if you have not already installed ROS2 Humble or Jazzy on Ubuntu 22.04 or 24.04, respectively, or if you have not created a ROS2 workspace.

### Building
Clone the repository into your ROS2 workspace's `src` directory:

```zsh
cd ~/your_ws/src
git clone https://github.com/marinarasauced/ros2_differential_drive_line_following.git
```

Clone the `mess2_msgs` and `mess2_plugins` packages into the workspace.

```zsh
cd ~/your_ws/src
git clone https://github.com/marinarasauced/mess2_msgs.git
git clone https://github.com/marinarasauced/mess2_plugins.git
```

Update your ROS2 dependencies:

```zsh
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

Compile the package using `colcon`:

```
cd ~/your_ws
colcon build --symlink-install --packages-select mess2_msgs mess2_plugins ros2_differential_drive_line_following
```

Source the setup script:

```
source ~/your_ws/install/setup.zsh
```

## Usage
The node is started onboard the TurtleBot3 using model-respective launch file. Since this package is designed for use with the Modular Experiment Software System 2, it is tailored for TurtleBot3s and VICON motion capture localization, as found in the ACE Lab.

## Launch Files

- **`burger.launch.py`**

    Launches the `ros2_differential_drive_line_following` action server node assuming default parameters `name:='burger1'` and `model:='burger'`.

- **`waffle.launch.py`**

    Launches the `ros2_differential_drive_line_following` action server node assuming default parameters `name:='waffle1'` and `model:='waffle'`.

## Nodes

### server

Creates an action server using the following action defintion:

```zsh
# UGVLineFollowing.action

# goal definition
# this action's goal contains the actors target point and action specific parameters pertaining to the vehicle's dynamics and control.

float64 k1          # the linear control input gain
float64 k2          # the angular control input gain
float64 v_ratio     # a ratio limiting the vehicle's maximum velocities; i.e., if the absolute maximum allowable linear velocity is 1.0 m/s and v_ratio is 0.7, the vehicle's maximum linear velocity during the action is 0.7 m/s.
geometry_msgs/Pose2D tolerances     # the allowable rotational and translational error toleranaces
geometry_msgs/Point boundaries_min  # the lower boundaries of the three-dimensional space the vehicle is allowed to operate within (based on localization x, y, z).
geometry_msgs/Point boundaries_max  # the upper boundaries of the three-dimensional space the vehicle is allowed to operate within (based on localization x, y, z).
geometry_msgs/Pose2D x_target       # the target state of the vehicle

---

# result definition
# this action's result contains a bool determining the success of the transition.

bool success

---

# feedback definition
# this action's feedback contains the current local error during a transition.

geometry_msgs/Pose2D error
```

The `k1` and `k2` values are control input gains used to calculate the angular control input values during rotations and translations. The linear model is as follows:

1. **Global state** (`x`):
   - \( X = [x_{global}, y_{global}, \theta_{global}] \)

By projecting the global coordinates of the vehicle onto the vector from the source pose to the target pose (assuming the vector is the x-axis of the newly defined local-coordinate frame), the linear model and control input are rewritten as an error-reduction model.

2. **Local error** (`e`):
   - \( E = [e_{y_{local}}, e_{\theta_{local}}] \)
   - Where:
     - \( e_{y} = y_{target} - y_{local} \)
     - \( e_{\theta} = \theta_{target} - \theta_{local} \)

3. **State derivatives** (`de/dt`):
   - \( \dot{E} = [-\dot{y_{local}}, -\dot{\theta_{local}}] \)
   - Where:
     - \( \dot{y} \) is the change in the local y-coordinate.
     - \( \dot{\theta} \) is the change in the local orientation.

The transition between the source and target states is split into three segments: rotation, translation, rotation. The control input are calculated as:

4. **Control inputs** (`u`):
   - \( U = [0.0, -`k2` * e_{\theta_{local}}] \) during rotations.
   - \( U = [max(u_{linear}), -`k1` * \theta_{target} -`k2` * e_{\theta_{local}}] \)
   - Where:
     - \( v_{linear} \) is the linear velocity.
     - \( v_{angular} \) is the angular velocity.

The linear velocity input controls the rate of the vehicle in its body-fixed x-axis, and the angular velocity input controls the rate about the vehicle's body-fixed z-axis. Control input are capped using the hardware-specified maximum allowable linear and angular velocities along with an artificial velocity ration `v_ratio` such that u_max = v_ratio * v_max.


#### Subscriptions

- **`/vicon/name/name`**

    The topic on which localization data is published from the ROS2 VICON driver node; `name` is replaced with the vehicle's name from the launch file.

#### Publishers

- **`/name/cmd_vel`**

    The topic on which the TurtleBot3 accepts control input; `name` is replaced with the vehicle's name from the launch file.

#### Parameters

- **`name`** (string, default: "burger1" or "waffle1)

	The name of the TurtleBot3 (this name must match the one in the VICON Tracker application).

- **`model`** (string, default: "burger" or "waffle")

	The TurtleBot3's model; valid options are "burger", "waffle", "waffle_pi", "wafflepi".
