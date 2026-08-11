'''
single_motor_slider.py 
Allows user to control each of the one specified motor 
on BILLIE with a keyboard.
W/S = +/- angle
Q = Quit

To be able to control all four motors at once, see multi_motor_slider.py
'''

import sys
import termios
import tty
import time
from cybergear import CyberGear

# Motor
MOTOR_ID = 0x3  # change motor here

motor = CyberGear({
    MOTOR_ID: "Cybergear"
})

# How much the position changes per key press
STEP = 0.01


# Read one keyboard character
def get_key():

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    return key


# Main
try:

    motor.enable_for_motion()

    print("======================================")
    print("     CyberGear Motor Position Test")
    print("======================================")
    print()
    print("Motor ID:", hex(MOTOR_ID))
    print()
    print("Controls:")
    print("  W = increase position")
    print("  S = decrease position")
    print("  Q = quit")
    print()
    print(f"Step size: {STEP} rad")
    print()

    # Read current motor position
    time.sleep(0.2)

    data = motor.get_data(MOTOR_ID)

    if data is None:
        raise RuntimeError("Could not read motor position.")

    position = data.current_position

    print(f"Current motor position: {position:.4f} rad")
    print()
    print("Starting from current position.")
    print()


    # Main control loop
    while True:
        key = get_key()

        # Increase position
        if key.lower() == "w":

            position += STEP

            motor.set_position(
                MOTOR_ID,
                position=position,
                kp=3,
                kd=0.2
            )

            print()
            print("W pressed")
            print(f"Commanded: {position:.4f} rad")

        # Decrease position
        elif key.lower() == "s":

            position -= STEP

            motor.set_position(
                MOTOR_ID,
                position=position,
                kp=3,
                kd=0.2
            )

            print()
            print("S pressed")
            print(f"Commanded: {position:.4f} rad")

        # Quit
        elif key.lower() == "q":

            print()
            print("Q pressed.")
            print("Stopping motor...")
            break

        # ----------------------------------------------------
        # Read motor state
        data = motor.get_data(MOTOR_ID)

        if data:

            print(
                f"Actual: {data.current_position:.4f} rad | "
                f"Velocity: {data.current_velocity:.4f} rad/s | "
                f"Torque: {data.current_torque:.4f} Nm"
            )

finally:

    print()
    print("Cleaning up...")
    motor.cleanup()
    print("Motor disabled.")