import pandas as pd
import cantools
import time
import can

# Load the CSV file (Replace 'your_file.csv' with the actual filename)
df = pd.read_csv("/home/avinashumeshsarma/panda/examples/Data_Log/Jan 28/01.csv")

# Load the DBC file (Replace 'your_file.dbc' with the actual DBC file)
dbc_file = "prius_dbc/opendbc/toyota_prius_2010_pt.dbc"
db = cantools.database.load_file(dbc_file)

# Initialize CAN Bus (Virtual SocketCAN for simulation)
bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

# Convert MessageID from Hexadecimal to Decimal
df["MessageID_Decimal"] = df["MessageID"].apply(lambda x: int(x, 16))

# Simulate playing the CAN data
print("🚗 Playing CAN Data...")

for index, row in df.iterrows():
    message_id = row["MessageID_Decimal"]
    data_bytes = bytes.fromhex(row["Message"][2:])  # Convert hex data to bytes
    timestamp = row["Time"]

    # Send CAN message
    can_msg = can.Message(arbitration_id=message_id, data=data_bytes, is_extended_id=False)

    if len(data_bytes) != 8:
        print(f"⚠️ Skipping ID {message_id} due to non-standard length ({len(data_bytes)})")
        continue

    bus.send(can_msg)

    # Decode the message using the DBC file
    try:
        decoded_msg = db.decode_message(message_id, data_bytes)
        print(f"⏱ {timestamp:.6f}s | ID: {message_id} | {decoded_msg}")
    except KeyError:
        print(f"⏱ {timestamp:.6f}s | ID: {message_id} | ❌ No matching DBC entry")

    # Simulate real-time playback delay
    time.sleep(0.01)  # Adjust based on real-time requirements

print("✅ Playback Finished!")
