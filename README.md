
# ros2_py_racecar

*F1TENTH RACECAR ROS2(rclpy) Binding.*

---
## Quickstart
1. Clone this repository in where you want workspace. 
    ```commandline
   cd ~/f1tenth_ws/
   git clone https://github.com/zygn/ros2_py_racecar
    ```
   
2. Building package with using [Colcon]() 
   ```commandline
   colcon build --packages-select ros2_py_racecar 
   . install/setup.bash 
   ```

3. Run racecar package.
   ```commandline
   ros2 run ros2_py_racecar racecar
   ```

If you want Hotfix, try this one-line command.
```commandline
colcon build --packages-select ros2_py_racecar && . install/setup.bash &&  ros2 run ros2_py_racecar racecar 
```

## Changing Planner 
TODO

## 