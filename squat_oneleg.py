from motor_controller import MotorController

import numpy as np
import time


# Motor IDs
HIP_MOTOR_ID = 0x03
KNEE_MOTOR_ID = 0x01


# Create motor controller
motors = MotorController([HIP_MOTOR_ID,KNEE_MOTOR_ID])


# Link lengths
l1 = 0.183
l2 = 0.213


def hip_inverse_kinematics(y_des):

    if y_des < 0 or y_des > l1 + l2:
        raise ValueError(
            "Desired hip position out of reachable range"
        )

    # Calculate theta2 using the law of cosines
    cos_theta2 = (y_des**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))  # Clip to avoid errors due to rounding

    # Calculate theta1 based on theta2 and y_des
    theta1 = np.arctan2(y_des, 0) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))

    # Send the calculated angles to the motors
    motors.set_position(HIP_MOTOR_ID, theta1, kp=3, kd=0.1)
    motors.set_position(KNEE_MOTOR_ID, theta2, kp=3, kd=0.1)


def track_sinusoid(duration=5, frequency=0.5, amplitude=0.05, offset=0.03):
    start_time = time.time()
    while time.time() - start_time < duration:
        t = time.time() - start_time
        hip_position = amplitude * np.cos(2 * np.pi * frequency * t) + (l1 + l2 - amplitude - offset)
        hip_inverse_kinematics(hip_position)
        time.sleep(0.01)


try:
    track_sinusoid()

finally:

    motors.cleanup()