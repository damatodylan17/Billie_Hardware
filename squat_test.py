'''
squat_test.py

Tests a squat motion for the biped BILLIE.
--> The desired height (z) is given, and inverse kinematics is used to 
    develop joint angles for each of the motors.
--> When power-cycling the system, remember to set_zero.py the 
'''

from cybergear import CyberGear
import numpy as np
import time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import os
from datetime import datetime

# Motor IDs
LH_MOTOR_ID = 0x3
LK_MOTOR_ID = 0x1

RH_MOTOR_ID = 0x4
RK_MOTOR_ID = 0x2

KP = 3.0
KD = 0.2

# Create CyberGear object
motors = CyberGear({LH_MOTOR_ID: "Cybergear", LK_MOTOR_ID: "Cybergear",
                RH_MOTOR_ID: "Cybergear", RK_MOTOR_ID: "Cybergear"})

STATE_IDX = {LK_MOTOR_ID: 1, LH_MOTOR_ID: 2, RH_MOTOR_ID: 3, RK_MOTOR_ID: 4}

# Link lengths
l1 = 0.183
l2 = 0.221

standing_height = l1 + l2   # 0.404 m

# Inverse kinematics
def hip_inverse_kinematics(z_des):
    # Check reachable range
    if z_des <= 0 or z_des > l1 + l2:
        raise ValueError(f"z_des={z_des} is outside reachable range")

    # Hip
    cos_theta1 = (l2**2 - l1**2 - z_des**2) / (-2 * l1 * z_des)
    theta1 = np.arccos(np.clip(cos_theta1, -1.0, 1.0))
    hip_angle = theta1

    # Knee
    cos_theta2 = (z_des**2 - l1**2 - l2**2) / (-2 * l1 * l2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))
    knee_angle = -(np.pi - theta2)

    return hip_angle, knee_angle


# Command both legs
def command_z(z_des):
    hip_angle, knee_angle = hip_inverse_kinematics(z_des)

    # Left leg
    motors.set_position(LH_MOTOR_ID,hip_angle,kp=KP,kd=KD)
    motors.set_position(LK_MOTOR_ID,knee_angle,kp=KP,kd=KD)

    # Right leg
    motors.set_position(RH_MOTOR_ID,-hip_angle,kp=KP,kd=KD)
    motors.set_position(RK_MOTOR_ID,-knee_angle,kp=KP,kd=KD)

# plots the squat, actual vs desired position
class SquatLogger:
    def __init__(self):
        self.time = []
        self.desired = []
        self.actual = []

        # Create timestamped folder
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.plot_dir = os.path.join("plots", f"squat_{timestamp}")
        os.makedirs(self.plot_dir, exist_ok=True)

        print(f"Saving squat plot to: {self.plot_dir}")

    def log(self, t, desired, actual):
        self.time.append(t)
        self.desired.append(np.asarray(desired).copy())
        self.actual.append(np.asarray(actual).copy())

    def plot(self):
        time_data = np.asarray(self.time)
        desired = np.asarray(self.desired)
        actual = np.asarray(self.actual)

        joint_names = ["Left Knee","Left Hip", "Right Hip","Right Knee"]

        for i, name in enumerate(joint_names):
            plt.figure(figsize=(10, 4))

            plt.plot(time_data,desired[:, i],label="Desired")

            plt.plot(time_data,actual[:, i],"--",label="Actual")

            plt.xlabel("Time [s]")
            plt.ylabel("Position [rad]")
            plt.title(f"{name} Position")
            plt.grid()
            plt.legend()

            filename = f"{name.lower().replace(' ', '_')}.png"
            filepath = os.path.join(self.plot_dir, filename)

            plt.savefig(filepath,dpi=150,bbox_inches="tight")

            plt.close()
            print(f"Saved: {filepath}")

def get_actual_positions():
    lk_data = motors.get_data(LK_MOTOR_ID)
    lh_data = motors.get_data(LH_MOTOR_ID)
    rh_data = motors.get_data(RH_MOTOR_ID)
    rk_data = motors.get_data(RK_MOTOR_ID)

    # Match STATE_IDX ordering:
    # [LK, LH, RH, RK]
    actual = np.array([
        lk_data.current_position,
        lh_data.current_position,
        rh_data.current_position,
        rk_data.current_position
    ])

    return actual

# Sinusoidal squat
def squat_sinusoid(duration=10,frequency=0.2,amplitude=0.02, logger=None):
    start_time = time.time()
    while time.time() - start_time < duration:
        t = time.time() - start_time

        # 0.404 m = standing
        # At sin = 1: z_des = 0.404  (standing)
        # At sin = -1: z_des = 0.364  (squat)

        z_des = (standing_height - amplitude + amplitude * np.sin(
                2 * np.pi * frequency * t))

        # Calculate desired joint angles
        hip_angle, knee_angle = hip_inverse_kinematics(z_des)

        desired = np.array([
            knee_angle,
            hip_angle,
            -hip_angle,
            -knee_angle
        ])

        command_z(z_des)

        # Get actual motor positions
        actual = get_actual_positions()

        if logger is not None:
            logger.log(t, desired, actual)

        print(f"z_des = {z_des:.4f} m")

        time.sleep(0.01)


# Main
logger = SquatLogger()
try:
    motors.enable_for_motion()

    print("CyberGear motors enabled.")

    print()
    input("Press ENTER to command a start position.")
    start_pos = [-0.29718542, 0.07431855, 0.41929060, 0.07520059]
    for i, motor_id in enumerate(STATE_IDX):
        motors.set_position(motor_id, start_pos[i], kp=KP,kd=KD)

    print()
    input("Press ENTER to start the squat.")

    # Start sinusoidal squat
    print("\nStarting sinusoidal squat...")

    squat_sinusoid(duration=15,frequency=0.2,amplitude=0.02, logger=logger)

    print("\nSinusoidal squat finished.")

    logger.plot()

    input("Press ENTER to disable the motors...")

finally:

    print("\nCleaning up...")
    motors.cleanup()