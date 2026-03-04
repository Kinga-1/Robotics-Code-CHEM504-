class ShakeModule:
    """
    Rotates wrist joint (joint 6, index 5) ±angle_deg to simulate shaking.
    Robot must be at a safe open-air position holding the vial before calling.

    Usage:
        shaker = ShakeModule(robot)
        shaker.shake(PositionList.shakePos, repetitions=15, angle_deg=90)
    """

    def __init__(self, robot):
        self.robot = robot

    def shake(self, base_joint_state, repetitions=15, angle_deg=90,
              speed=1.5, accel=1.5):
        """
        Args:
            base_joint_state : joint angles (radians) of safe shake position
            repetitions      : number of full left-right cycles
            angle_deg        : degrees to rotate each side
            speed / accel    : rad/s
        """
        import math
        angle_rad = math.radians(angle_deg)
        base  = list(base_joint_state)
        left  = base.copy(); left[5]  = base[5] + angle_rad
        right = base.copy(); right[5] = base[5] - angle_rad

        print(f"[Shake] Starting {repetitions} cycles (±{angle_deg}°)...")
        for i in range(repetitions):
            self.robot.move_joint_list(left,  speed, accel, 0.0)
            self.robot.move_joint_list(right, speed, accel, 0.0)
            if (i + 1) % 5 == 0:
                print(f"[Shake] {i+1}/{repetitions} cycles done.")

        self.robot.move_joint_list(base, speed, accel, 0.02)
        print("[Shake] Done.")
