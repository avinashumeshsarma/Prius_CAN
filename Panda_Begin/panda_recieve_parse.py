import cantools
from panda import Panda

# Load the DBC file
DBC_FILE = "prius_dbc/opendbc/toyota_prius_2010_pt.dbc"  # Change this to your DBC file
try:
    dbc = cantools.database.load_file(DBC_FILE)
    print(f"Loaded DBC file: {DBC_FILE}")
except Exception as e:
    print("Error loading DBC file:", e)
    exit()

# Connect to the Panda device
try:
    panda = Panda()
    print("Connected to Panda:", panda.get_serial())
except Exception as e:
    print("Error connecting to Panda:", e)
    exit()

# Set up the CAN interface
try:
    CAN_BUS = 0  # Select CAN bus 0 (change to 1 or 2 if needed)
    panda.set_safety_mode(Panda.SAFETY_TOYOTA)  # Use Toyota safety mode
    panda.set_power_save(False)  # Disable power saving mode

    bus0_msg_cnt = 0
    bus1_msg_cnt = 0
    bus2_msg_cnt = 0
except Exception as e:
    print("Error configuring CAN interface:", e)
    exit()

# Read and parse CAN messages
try:
    print("Listening for CAN messages... (Press Ctrl+C to stop)")
    while True:
        messages = panda.can_recv()

        for address, data, src in messages:
            # Increment bus message count
            # if len(data)<8:
            #     print(f"Skipping message since the data length is less than 8")
            #     continue

            if src == 0:
                bus0_msg_cnt += 1
            elif src == 1:
                bus1_msg_cnt += 1
            elif src == 2:
                bus2_msg_cnt += 1

            # Try to decode the CAN message using the DBC file
            try:
                message = dbc.decode_message(address, data)
                print(f"Decoded Message: {message}")
            except KeyError:
                print(f"Unknown Message ID: {hex(address)}")
                print(f"Raw Data: {data.hex()}")

            print("\n")

        # Print message count
        print(f"Message Counts... Bus 0: {bus0_msg_cnt} Bus 1: {bus1_msg_cnt} Bus 2: {bus2_msg_cnt}", end='\r')

except KeyboardInterrupt:
    print("\nStopping CAN message reception.")

except Exception as e:
    print("Error receiving CAN messages:", e)

finally:
    panda.close()
    print("Panda connection closed.")
