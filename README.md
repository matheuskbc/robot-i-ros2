# Install requirements

1. Execute:

2. Create virtual env
```
python -m venv robot-i-ros2-venv
```
3. Activate virtual env

```
source robot-i-ros2-venv/bin/activate
```

4. Install python dependencies 

```
pip3 install -r requirements/python-requirements.txt
```

5. Install ros dependecies
```
rosdep update
```

```
rosdep install --from-paths src --ignore-src -r -y
```

6. Install system dependecies

```
bash requirements/system-requirements.txt
```

7. Download repos

```
vcs import --input requirements/repos-requirements.txt
```

8. Catkin build
```
Source the ros2 installation folder
```

```
colcon build --symlink-install
```

# How to run

Run the following commands:

```
ros2 launch roomba_navigation roomba_map_server.launch.py

ros2 launch roomba_navigation roomba_amcl.launch.py

ros2 launch roomba_navigation roomba_navigation.launch.py

ros2 launch roomba_navigation roomba_rviz.launch.py
```

Other commands:

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel

ros2 launch roomba_robot_localization.py

ros2 launch roomba_navigation roomba_robot_localization.launch.py

ros2 launch roomba_navigation roomba_move_base.launch.py

ros2 launch roomba_navigation roomba_rviz.launch.py

ros2 launch slam_toolbox online_async_launch.py

ros2 launch roomba_navigation roomba_map_server.launch.py
```

# How to map

```
ros2 launch roomba_navigation roomba_move_base.launch.py use_slam:=True

ros2 launch roomba_navigation roomba_rviz.launch.py
```


```
Alternatively from a docker image

sudo docker run -it --rm --name mapping robot-i-ros2-roomba_navigation bash "scripts/start_mapping.bash"
```

How to save map

```
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "map_topic: 'map'
map_url: '/home/matheus/Projects/robot-i-ros2/src/roomba_navigation/map/room_updated'
image_format: 'pgm'
map_mode: 'raw'
free_thresh: 0.0
occupied_thresh: 0.0"
```

# New how to

- ros2 launch roomba_navigation roomba_sensors_static_transforms.py
- ros2 launch roomba_navigation roomba_all.launch.py
- ros2 launch roomba_navigation roomba_rviz.launch.py