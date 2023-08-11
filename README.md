
# ros2_py_racecar

F1TENTH Racecar driving algorithm planner binding for ROS2(rclpy)

---
## Quickstart
1. Clone this repository in where you want workspace. 
    ```commandline
   cd ~/f1tenth_ws/src
   git clone https://github.com/zygn/ros2_py_racecar
    ```
   
2. Building package with using [Colcon](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) 
   ```commandline
   cd .. && colcon build --packages-select ros2_py_racecar 
   . install/setup.bash 
   ```

3. Run racecar package.
   ```commandline
   ros2 run ros2_py_racecar racecar
   ```

If you want Hotfix, try this one-line command.
```commandline
colcon build --packages-select ros2_py_racecar && . install/setup.bash && ros2 run ros2_py_racecar racecar 
```

## Changing Planner 
TODO

## Related Project

- [F1TENTH](https://f1tenth.org/)
- [F1TENTH Racecar](https://github.com/f1tenth/f1tenth_system)
- [F1TENTH gym environment ROS2 communication bridge](https://github.com/f1tenth/f1tenth_gym_ros)