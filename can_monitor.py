import can

bus = can.interface.Bus(
    channel="can0",
    interface="socketcan"
)

print("Listening on CAN...")

while True:
    msg = bus.recv()

    if msg:
        print(
            f"ID: {hex(msg.arbitration_id)} "
            f"EXT:{msg.is_extended_id} "
            f"DATA:{msg.data.hex()}"
        )