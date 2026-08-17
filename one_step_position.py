'''
Position control for a one-step controller (from Julie's code)
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

# Change file to load here
# joint_angles = np.load("Xsol_without-height.npy")  
joint_angles = np.load("Xsol_full_0.2_periodic_0.2_wheight.npy")
# joint_angles = np.load("Xsol_0.3_periodic_0.4.npy")

joint_angles[:, 1] = - joint_angles[:, 1]
joint_angles[:, 2] = - joint_angles[:, 2]
joint_angles[:, 3] = - joint_angles[:, 3]
joint_angles[:, 4] = - joint_angles[:, 4]

def command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle):
    # Left leg
    motors.set_position(LH_MOTOR_ID, lhip_angle,kp=3,kd=0.2)
    motors.set_position(LK_MOTOR_ID, lknee_angle,kp=3,kd=0.2)

    # Right leg
    motors.set_position(RH_MOTOR_ID, rhip_angle,kp=3,kd=0.2)
    motors.set_position(RK_MOTOR_ID, rknee_angle,kp=3,kd=0.2)

# Reads the actual motor positions
def get_actual_positions():
    actual = {}
    for motor_id in [LK_MOTOR_ID,LH_MOTOR_ID,RH_MOTOR_ID,RK_MOTOR_ID]:
        data = motors.get_data(motor_id)

        if data is not None:
            actual[motor_id] = data.current_position
        else:
            actual[motor_id] = None
    return actual

# Main
def main():
    try:
        motors.enable_for_motion()
        print("CyberGear motors enabled.")

        print()
        input("Press ENTER to start the step.")

        row = joint_angles[0]
        lknee_angle = row[1]
        lhip_angle = row[2]
        rhip_angle = row[3]
        rknee_angle = row[4]
        command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle)

        actual = get_actual_positions()

        input("Press ENTER to start the step.")

        for step, row in enumerate(joint_angles, start=1):
            lknee_angle = row[1]
            lhip_angle = row[2]
            rhip_angle = row[3]
            rknee_angle = row[4]

            command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle)
            commanded = [lknee_angle, lhip_angle, rhip_angle, rknee_angle]
            actual = get_actual_positions()
            
            print(f"commanded right knee: {rknee_angle}")
            print(f"actual right knee: {actual.get(2)}")
            # time.sleep(0.01)
            input("enter")


        
    finally:
        print("\nCleaning up...")
        # command_z(0,0,0,0)

        # motors.cleanup()


if __name__=="__main__":
    main()