# import can
# import time

# # figures out what the motor ID is

# bus = can.interface.Bus(
#     channel="can0",
#     interface="socketcan"
# )

# print("Scanning...")

# for motor_id in range(1, 128):

#     # MIT read parameter command
#     can_id = (0x11 << 24) | (0 << 8) | motor_id

#     msg = can.Message(
#         arbitration_id=can_id,
#         is_extended_id=True,
#         data=[
#             0x20, 0x10,
#             0,0,0,0,0,0
#         ]
#     )

#     try:
#         bus.send(msg)
#     except:
#         pass

#     rx = bus.recv(timeout=0.01)

#     if rx:
#         print(
#             "FOUND:",
#             hex(rx.arbitration_id),
#             rx.data.hex()
#         )

# print("Done")

import can
import time

bus = can.interface.Bus(
    channel="can0",
    interface="socketcan"
)

print("=== CyberGear CAN Scanner ===")

# Flush any old frames
while True:
    msg = bus.recv(timeout=0.01)
    if msg is None:
        break

print("Bus cleared.\n")

for motor_id in range(1, 5):

    can_id = (0x11 << 24) | (0 << 8) | motor_id

    tx = can.Message(
        arbitration_id=can_id,
        is_extended_id=True,
        data=[
            0x20, 0x10,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        ]
    )

    print(f"Trying motor ID {motor_id:3d}...", end=" ")

    try:
        bus.send(tx)
        print("TX OK", end="")
    except Exception as e:
        print(f"TX FAILED: {e}")
        continue

    found = False
    start = time.time()

    while time.time() - start < 0.1:
        rx = bus.recv(timeout=0.01)

        if rx is None:
            continue

        found = True

        print(
            f"\n    RX:"
            f"\n      ID       = {hex(rx.arbitration_id)}"
            f"\n      Extended= {rx.is_extended_id}"
            f"\n      DLC      = {rx.dlc}"
            f"\n      Data     = {rx.data.hex()}"
        )

    if not found:
        print("  --> No response")

print("\nFinished.")