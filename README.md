# ros2_eval_task

## Dependencies (Native)
Make sure you have **ROS 2 Humble** and **Gazebo Classic** installed, then install the required packages:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
sudo apt install \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-visualization-msgs \
  ros-humble-gazebo-msgs
```

## Build the Workspace (Native)
```bash
mkdir -p ~/ros2_ws/src
# place this package inside ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install && source install/setup.bash
```

## Quickstart (Native)
1. Launch the simulator
```bash
source install/setup.bash && ros2 launch ros2_eval_task gazebo.launch.py
```
2. Run the model spawner node
```bash
source install/setup.bash && ros2 run ros2_eval_task model_spawner_node
```

---

## Docker
### 0) Install dependencies
```bash
sudo apt-get update && \
     apt-get -y install make
```
### 1) Build the image
```bash
# from the pkg (where the Dockerfile is)
make run_docker # this build the docker image and run it with the required image
```
### 2) Run the container (with GUI parameters)
```bash
make exec_container #this line must be run from host terminal
```
#### 1) inside the container
```bash
colcon build --symlink-install && \
  source install/setup.bash && \
  ros2 launch ros2_eval_task gazebo.launch.py
```
### 3) Run the spawner node inside the container
```bash
make exec_container #this line must be run from host terminal
```
#### 1) inside the container
```bash
  source install/setup.bash && \
  ros2 run ros2_eval_task model_spawner_node
```
