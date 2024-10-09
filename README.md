# rrt_planner
The `rrt_planner` ROS package provides a framework to implement sampling based motion planners (`RRT` and its variants) as global planner plugin for `move_base`, a ROS framework. Please refer to http://wiki.ros.org/move_base, for a detailed description of the `move_base` framework.

## Demo
![](media/rrt_turtlebot_rviz.gif)

## Install Dependencies and Build
```bash
cd ~/catkin_ws/src
git clone https://github.com/irob-labs-ist/rrt_planner.git
cd .. && rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Usage
Within the [move_base](https://wiki.ros.org/move_base) node in your launch file, set the `base_global_planner` parameter to `rrt_planner/RRTPlannerROS` and load the required parameters.
```xml
<param name="base_global_planner" value="rrt_planner/RRTPlannerROS"/>
<rosparam file="$(find rrt_planner)/config/rrt_planner.yaml" command="load" />
```

After launching the system, when you set a `move_base/goal` using RViz's `2D Nav Goal` or with an action client, the `RRTPlannerROS` will be called. The global path will be published as a topic for visualization in RViz. Add a `Path` display and subscribe to `~/move_base/RRTPlannerROS/global_plan`.

## Examples
An example launch file for using `RRTPlannerROS` is located in [rrt_planner/launch](launch/). An example RViz config file for using `RRTPlannerROS` with the [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) is located in [rrt_planner/rviz](rviz/rrt_turtlebot_sim.rviz).

## Troubleshooting
If you find that the RRT planner continues to produce plans even after a solution has been found, try setting the `/move_base/planner_frequency` to 0. Zero is normally what the default value is (see `planner_frequency` on the [move_base Wiki](http://wiki.ros.org/move_base)), but that's not what the value is in the turtlebot examples. If you're running the turtlebot examples, you can change the parameter in `turtlebot3_navigation/param/move_base_params.yaml`.

If you see that the RRT planner repeatedly fails to find a solution, you may want to play around with the parameters in the [rrt_planner.yaml](config/rrt_planner.yaml) configuration file.

## ROS API
### Published Topics
`~/move_base/RRTPlannerROS/global_plan` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
- The global path constructed by the planner. Used for visualization purposes.

### Subscribed Topics
None.

### Parameters
`~/move_base/RRTPlannerROS/goal_tolerance` (`double`, default: 0.20)
- Goal tolerance (in meters) to be achieved by the global planner.

`~/move_base/RRTPlannerROS/rrt/step` (`double`, default: 0.20)
- Distance (in meters) to extend tree per iteration.

`~/move_base/RRTPlannerROS/rrt/min_num_nodes` (`int`, default: 200)
- Minumum number of iterations to attempt to find a plan.

`~/move_base/RRTPlannerROS/rrt/max_num_nodes` (`int`, default: 5,000)
- Maximum number of iterations to attempt to find a plan.


### Services
None.