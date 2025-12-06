# Carver_description

## How to install

1. **Colcon build and source** your workspace
```bash
cd ~/carver_ws
colcon build --symlink-install
source install/setup.bash
```
3. Display rviz
```bash
ros2 launch carver_description simple_display.launch.py
```