# leader_follower_simulation
Gazebo simulation of leader-follower formation control for multi-agent system.

## 1. Dependencies
- g2o
- jackal
- turtlebot3
- move_base
```bash
sudo apt-get install ros-noetic-libg2o ros-noetic-move-base*
git clone https://github.com/Daffan/nav-competition-icra2022.git
git clone https://github.com/jackal/jackal.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_simulator.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_desktop.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/utexas-bwi/eband_local_planner.git
git clone https://github.com/rst-tu-dortmund/teb_local_planner.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/rst-tu-dortmund/teb_local_planner_tutorials.git --branch <YOUR_ROS_VERSION>-devel
```

## 2. Build
```bash
catkin_make --cmake-args \
    -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3 \
    -DPYTHON_LIBRARY=/usr/lib/libpython3.so
```

## 3. Run Single Robot
```bash
source /devel/setup.sh
python3 run.py --gui --world_idx 0
```

## 4. Run Multi Robots
```bash
source /devel/setup.sh
roslaunch multi_agent_formation multi_jackal.launch
```