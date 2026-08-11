"""
multi_motor_slider.py

Allows user to control each of the four motors on BILLIE with a keyboard.
W/S = Motor 1 (Left Knee)
E/D = Motor 2 (Right Knee)
R/F = Motor 3 (Left Hip)
T/G = Motor 4 (Right Hip)
Q = Quit

"""
import curses
import time
from cybergear import CyberGear


# Motor configuration
MOTORS = {0x1: "LK", 0x2: "RK", 0x3: "LH", 0x4: "RH",} 

STEP = 0.01 # radians per key press
KP = 5
KD = 0.2

motor = CyberGear({motor_id: "Cybergear" for motor_id in MOTORS})


# Draw slider
def draw_slider(stdscr, row, name, commanded, actual):
    min_pos = -2.0
    max_pos = 2.0

    width = 50

    # Convert commanded position to slider location
    normalized = (commanded - min_pos) / (max_pos - min_pos)

    normalized = max(0.0, min(1.0, normalized))

    cursor = int(normalized * (width - 1))

    slider = ["-"] * width
    slider[cursor] = "O"

    stdscr.addstr(
        row,
        0,
        f"{name:>2}  "
        f"[{''.join(slider)}] "
        f"{commanded:+.3f}"
    )

    stdscr.addstr(
        row + 1,
        6,
        f"Actual: {actual:+.3f} rad"
    )


# Main
def main(stdscr):

    # Don't wait for keyboard input
    stdscr.nodelay(True)

    # Hide cursor
    curses.curs_set(0)

    # Enable motors
    motor.enable_for_motion()
    time.sleep(0.2)

    # Read current positions
    commanded = {}
    actual = {}

    for motor_id in MOTORS:
        data = motor.get_data(motor_id)

        if data is None:
            commanded[motor_id] = 0.0
            actual[motor_id] = 0.0

        else:
            commanded[motor_id] = data.current_position
            actual[motor_id] = data.current_position

    # Keyboard mapping
    key_map = {
        # Motor 1
        "w": (0x1, +STEP),
        "s": (0x1, -STEP),

        # Motor 2
        "e": (0x2, +STEP),
        "d": (0x2, -STEP),

        # Motor 3
        "r": (0x3, +STEP),
        "f": (0x3, -STEP),

        # Motor 4
        "t": (0x4, +STEP),
        "g": (0x4, -STEP),
    }

    # Main loop
    try:

        while True:

            # Read keyboard
            key = stdscr.getch()

            if key != -1:

                try:
                    char = chr(key).lower()
                except:
                    char = ""

                # Quit
                if char == "q":
                    break

                # Move one motor
                if char in key_map:

                    motor_id, change = key_map[char]

                    commanded[motor_id] += change

                    motor.set_position(
                        motor_id,
                        position=commanded[motor_id],
                        kp=KP,
                        kd=KD
                    )

            # Continuously send position commands
            for motor_id in MOTORS:

                motor.set_position(
                    motor_id,
                    position=commanded[motor_id],
                    kp=KP,
                    kd=KD
                )

                data = motor.get_data(motor_id)

                if data is not None:
                    actual[motor_id] = data.current_position

            # Draw screen
            stdscr.erase()

            stdscr.addstr(
                0,
                0,
                "=============================================="
            )

            stdscr.addstr(
                1,
                0,
                "       CyberGear 4-Motor Controller"
            )

            stdscr.addstr(
                2,
                0,
                "=============================================="
            )

            stdscr.addstr(
                4,
                0,
                "Controls:"
            )

            stdscr.addstr(
                5,
                0,
                "  W/S = Motor 1 (LK)"
            )

            stdscr.addstr(
                6,
                0,
                "  E/D = Motor 2 (RK)"
            )

            stdscr.addstr(
                7,
                0,
                "  R/F = Motor 3 (LH)"
            )

            stdscr.addstr(
                8,
                0,
                "  T/G = Motor 4 (RH)"
            )

            stdscr.addstr(
                9,
                0,
                "  Q   = Quit"
            )

            row = 11

            for motor_id, name in MOTORS.items():

                draw_slider(
                    stdscr,
                    row,
                    f"{name}",
                    commanded[motor_id],
                    actual[motor_id]
                )

                row += 3

            stdscr.addstr(
                row + 1,
                0,
                f"Step: {STEP:.3f} rad    "
                f"Kp: {KP}    Kd: {KD}"
            )

            stdscr.refresh()

            time.sleep(0.01)

    finally:

        motor.cleanup()


# ============================================================
# Run
# ============================================================

if __name__ == "__main__":

    curses.wrapper(main)