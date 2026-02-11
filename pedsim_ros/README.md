# Pedestrian Simulator
<img src=https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/crowd1.png width=400/> | <img src=https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator/images/costmap.png width=400/>

ROS packages for a 2D pedestrian simulator based on social force
model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf). The implementation is based on an extended version of Christian Gloor's [libpedsim](http://pedsim.silmaril.org/) library which has been extended to include additional behaviors and activities. This packages is useful for robot navigation experiments with crowded scenes which are hard to acquire in practice.

### Features
- Individual walking using social force model for very large crowds in real time
- Group walking using the extended social force model
- Social activities simulation
- Sensors simulation (point clouds in robot frame for people and walls)
- XML based scene design
- Extensive visualization using Rviz
- Option to connect with gazebo for physics reasoning

### Requirements
- ROS2 (currently tested on `humble`). `
- C++14 compiler

### Installation

This installation guide is for ROS2. For ROS1 please check out the ROS1 branches in the official repo.

```
cd [workspace]/src
git clone https://github.com/AET-Automation-Engineer-Training/hippo-social-aware-planner.git
cd ..
colcon build
```

### Sample usage with Rurbot
```
ros2 launch pedsim_gazebo_plugin robot_test_launch.py
```

### Navigation
Terminal 1:
```
ros2 launch rur_navigation2 navigation2.launch.py
```
Terminal 2: If you want to obtain the necessary metrics for comparison
```
ros2 run data_processor navigation_metrics_logger.cpp
```
### Adding a simulation environment
- Configuration is in `/hippo-social-aware-planner/pedsim_ros/pedsim_gazebo_plugin/worlds`
- Adding this plugin in <world> tag:
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo_spawner</namespace>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>

### Adding a scenario
- Configuration is in `/hippo-social-aware-planner/pedsim_ros/pedsim_simulator/scenarios`
- Sample XML scenario file is `/hippo-social-aware-planner/pedsim_ros/pedsim_simulator/scenarios/office_env_large.xml`

### Licence
The core `libpedsim` is licensed under LGPL. The ROS integration and extensions are licensed under BSD.

The package is a **work in progress** mainly used in research prototyping. Pull requests and/or issues are highly encouraged.

### Acknowledgements
These packages have been developed in part during the EU FP7 project [SPENCER](spencer.eu)
