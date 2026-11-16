# Results per robot

Copy `motion_template.csv` into `robot_1_motion.csv`, `robot_2_motion.csv`, etc. Use measured values only; one row per completed pulse.

For each robot also record:

- Test PWM: startup/sustaining duty for left/right, forward/reverse; observed wheel directions.
- Test speed: pulse command, actual time, energized travel, payload, measured battery voltage, repeated trials. Note curved-path arc length separately from endpoint displacement.
- Test IMU: stationary gyro range, yaw drift, yaw after externally measured left/right 90/180-degree turns.
- Test pallet: lower/lift angles for each servo, measured settling time, success count/trial count and docking observations.

Existing detailed profiles remain at `firmware/test/results/robot_N/profile.json`. Unmeasured entries stay null. Nothing in this folder is automatically applied to the running controller.
