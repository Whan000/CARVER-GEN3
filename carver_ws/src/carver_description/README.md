# Carver_description

## How to install

1. Clone this git in your **workspace->src**.
```bash
git clone https://github.com/CARVER-NEXT-GEN/carver_description.git
```

2. **Colcon build and source** your workspace
```bash
colcon build && source ~/.bashrc
```
3. Display rviz
```bash
ros2 launch carver_description simple_display.launch.py
```