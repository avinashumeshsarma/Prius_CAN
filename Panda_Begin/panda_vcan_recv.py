import can
from panda import Panda

# Initialize Panda and vCAN interface
panda = Panda()
bus = can.interface.Bus('vcan0', bustype='socketcan')

try:
    while True:
        # Read CAN messages from Panda
        messages = panda.can_recv()
        for msg in messages:
            arbitration_id, extended, data, bus_num = msg
            # Send to virtual CAN (vcan0)
            can_msg = can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=bool(extended)
            )
            bus.send(can_msg)
            print(f"Sent to vCAN: {can_msg}")

        # (Optional) Relay messages from vCAN back to Panda
        msg_from_vcan = bus.recv(timeout=0.1)
        if msg_from_vcan:
            panda.can_send(
                msg_from_vcan.arbitration_id,
                msg_from_vcan.data,
                bus=0  # Specify which Panda CAN bus to use (e.g., 0, 1, or 2)
            )
            print(f"Sent to Panda: {msg_from_vcan}")
except KeyboardInterrupt:
    print("Bridge stopped.")
