# Setup Instructions

1. Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_avoidance
source install/setup.bash
```

2.  Run the nodes in two terminals (ensure the Turtlesim terminal is started first):
```bash
# Terminal 1
ros2 run turtlesim turtlesim_node
# Terminal 2
ros2 run turtle_avoidance avoider
```
