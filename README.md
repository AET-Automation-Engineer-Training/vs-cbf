# Virtual Social Control Barrier Function

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This repository implements VSCBF-MPPI, a socially compliant robot navigation framework that integrates predictive human comfort zones as soft, differentiable penalties within Model Predictive Path Integral (MPPI) control.

---

## 📺 Demo Video
[![Watch the video](https://img.youtube.com/vi/VOvwsm09DQU/0.jpg)](https://www.youtube.com/playlist?list=PLJVh23tSYeJ_heGxY1AITG5Npxdda1Fy_)
> [!NOTE]
> *Click the image above to watch our navigation framework in action.*

---

### Overview
Navigating effectively in human-populated environments requires robots to respect social norms and human comfort, not just avoid collisions. Traditional "hard" constraints often cause the "freezing robot" problem in dense crowds.

VSCBF addresses this by modeling personal space as anisotropic, uncertainty-aware potentials. Instead of rigid barriers, it reshapes the MPPI sampling distribution to balance social compliance with navigation efficiency.

### Requirements
- ROS2 (currently tested on `humble`). 
- C++14 compiler
- g2o: [20230806_git](https://github.com/RainerKuemmerle/g2o/releases/tag/20230806_git)

### Installation

#### g2o Install 
```bash
wget https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20230806_git.zip
unzip g2o-20230806_git.zip
sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
cd g2o-20230806_git
mkdir build && cd build
cmake ../
make
sudo make install 
```

#### Workspace Build
```bash
cd [workspace]/src
git clone https://github.com/AET-Automation-Engineer-Training/vs-cbf.git
cd ..
colcon build
```

### Sample usage with Rurbot
```bash
ros2 launch pedsim_gazebo_plugin robot_test_launch.py
```
Configure world and scenario in `/vs-cbf/pedsim_ros/pedsim_gazebo_plugin/launch/robot_test_launch.py`, line 18 to 21

### Navigation
For example, with MPPI_VSCBF:
```bash
ros2 launch rur_navigation2 navigation2_mppi_cbf.launch.py
```

### Adding a simulation environment
- Configuration is in `/vs-cbf/pedsim_ros/pedsim_gazebo_plugin/worlds`
- Adding this plugin in `<world>` tag:
```xml
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo_spawner</namespace>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>
```

### Adding a scenario
- Configuration is in `/vs-cbf/pedsim_ros/pedsim_simulator/scenarios`
- Sample XML scenario file is `/vs-cbf/pedsim_ros/pedsim_simulator/scenarios/office_env_large.xml`

```xml
  <welcome>
      <waypoint id="A" x="-5" y="-6" r="0.1" /> <!-- id: agent_id; x,y: agent coordinates -->
      <waypoint id="B" x="5" y="6" r="0.1" /> <!-- id: agent_id; x,y: agent coordinates -->
      <agent x="-5" y="-6" n="1" dx="0" dy="0" vmax="0.3">      <!-- agents coordinates setting, dx, dy, vmax  -->
       <addwaypoint id="A" />                                   <!-- agents move from A to B  -->
       <addwaypoint id="B" />
      </agent>
  </welcome>
```

### Acknowledgements
- [Pedestrian Simulator](https://github.com/srl-freiburg/pedsim_ros/blob/master/pedsim_simulator)
- [MPC-D-CBF](https://github.com/jianzhuozhuTHU/MPC-D-CBF)
- [TEB](https://github.com/rst-tu-dortmund/teb_local_planner)
