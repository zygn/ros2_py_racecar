
# ros2_py_racecar

F1TENTH Racecar driving algorithm planner binding for ROS2(rclpy)

---
## Quickstart
1. Clone this repository in where you want workspace. 
    ```commandline
   cd ~/f1tenth_ws/
   git clone https://github.com/zygn/ros2_py_racecar
    ```
   
2. Building package with using [Colcon](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) 
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