class WallFollow:
    def __init__(self):
        self.robot_scale = 0.3302

    def drive(self, scan_data, odom_data):
        ranges = scan_data.ranges
        try:
            left = ranges[720]
            right = ranges[360]
            front = ranges[540]

            max_size = left+right
            center = max_size/2
        except IndexError:
            return 0.0, 0.0

        print(f"Left : {left}, Front: {front}, Right: {right}")
        if front < 0.5:
            steering_angle = 0.0
            speed = -1.0

        elif left > 2.1:  # If you move away from the left wall, you will get closer to the left wall.
            steering_angle = 0.15
            speed = 1.0
        elif left < 1.0:  # If you get too close to the left wall, you move to the right.
            steering_angle = -0.15
            speed = 1.0

        else:  # When you keep a certain distance from the wall, you drive at high speed.
            steering_angle = 0.0
            speed = 3.0


        return speed, steering_angle
