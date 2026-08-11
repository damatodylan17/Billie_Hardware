from cybergear import CyberGear
import time

# Motor IDs
LH_MOTOR_ID = 0x3
LK_MOTOR_ID = 0x1
RH_MOTOR_ID = 0x4
RK_MOTOR_ID = 0x2

motor_ids = [LH_MOTOR_ID,LK_MOTOR_ID,RH_MOTOR_ID,RK_MOTOR_ID]

# Create CyberGear object
motors = CyberGear({
    LH_MOTOR_ID: "Cybergear",
    LK_MOTOR_ID: "Cybergear",
    RH_MOTOR_ID: "Cybergear",
    RK_MOTOR_ID: "Cybergear"})

try:
    motors.enable_for_motion()

    print("CyberGear motors enabled.")
    print()
    print("Put the robot in the desired STANDING position.")
    print("Make sure all four legs are positioned correctly.")
    print()

    input("Press ENTER to set all four motors to ZERO.")

    for motor_id in motor_ids:
        print(f"Setting motor {hex(motor_id)} zero...")
        motors.set_zero_position(motor_id)
        time.sleep(0.1)

    print()
    print("All four motors zeroed!")
    print()
    input("Press ENTER to disable motors...")

finally:

    print("Cleaning up...")
    motors.cleanup()
    print("Motors disabled.")