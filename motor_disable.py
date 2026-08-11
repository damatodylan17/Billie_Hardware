from cybergear import CyberGear
import time

MOTOR_IDS = [0x1, 0x2, 0x3, 0x4]

motor = CyberGear(
    {motor_id: "Cybergear" for motor_id in MOTOR_IDS},
    verbose=True
)

time.sleep(0.5)

motor.cleanup()

print("All four motors disabled")