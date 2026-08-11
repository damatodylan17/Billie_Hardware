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

# Motor IDs
LH_MOTOR_ID = 0x3
LK_MOTOR_ID = 0x1

RH_MOTOR_ID = 0x4
RK_MOTOR_ID = 0x2

# Create CyberGear object
motors = CyberGear({LH_MOTOR_ID: "Cybergear", LK_MOTOR_ID: "Cybergear",
                RH_MOTOR_ID: "Cybergear", RK_MOTOR_ID: "Cybergear"})


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
    motors.set_position(LH_MOTOR_ID,hip_angle,kp=3,kd=0.2)
    motors.set_position(LK_MOTOR_ID,knee_angle,kp=3,kd=0.2)

    # Right leg
    motors.set_position(RH_MOTOR_ID,-hip_angle,kp=3,kd=0.2)
    motors.set_position(RK_MOTOR_ID,-knee_angle,kp=3,kd=0.2)

# Sinusoidal squat
def squat_sinusoid(duration=10,frequency=0.2,amplitude=0.02):
    start_time = time.time()
    while time.time() - start_time < duration:
        t = time.time() - start_time

        # 0.404 m = standing
        # At sin = 1: z_des = 0.404  (standing)
        # At sin = -1: z_des = 0.364  (squat)

        z_des = (standing_height - amplitude + amplitude * np.sin(
                2 * np.pi * frequency * t))

        command_z(z_des)
        print(f"z_des = {z_des:.4f} m")
        time.sleep(0.01)


# Main
try:
    motors.enable_for_motion()

    print("CyberGear motors enabled.")

    print()
    input("Press ENTER to start the squat.")

    # Start sinusoidal squat
    print("\nStarting sinusoidal squat...")

    squat_sinusoid(duration=15,frequency=0.2,amplitude=0.02)

    print("\nSinusoidal squat finished.")
    input("Press ENTER to disable the motors...")

finally:

    print("\nCleaning up...")
    motors.cleanup()