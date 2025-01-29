from panda import Panda

# Connect to the Panda device
try:
    panda = Panda()
    print("Connected to Panda:", panda.get_serial())
except Exception as e:
    print("Error connecting to Panda:", e)
    exit()

# Set up the CAN interface
try:
    CAN_BUS = 0  # Select CAN bus 0 (you can change to 1 or 2 for other buses)
    panda.set_safety_mode(Panda.SAFETY_TOYOTA)  # Allow all CAN messages
    # print(f"CAN bus {CAN_BUS} configured.")
    bus0_msg_cnt = 0
    bus1_msg_cnt = 0
    bus2_msg_cnt = 0
except Exception as e:
    print("Error configuring CAN interface:", e)
    exit()

# Read CAN messages
try:
    print("Listening for CAN messages... (Press Ctrl+C to stop)")
    while True:
        # Receive messages from the selected CAN bus
        messages = panda.can_recv()

        # print(messages)
        for address, dat, src in messages:
            if src == 0:
                bus0_msg_cnt += 1
            elif src == 1:
                bus1_msg_cnt += 1
            elif src == 2:
                bus2_msg_cnt += 1
            print(f"ID : {hex(address)}\t Data : {dat.hex()}\t Bus : {src}")
            print("\n")

    print(f"Message Counts... Bus 0: {bus0_msg_cnt} Bus 1: {bus1_msg_cnt} Bus 2: {bus2_msg_cnt}", end='\r')

        # for message in messages:
        #     if len(message) == 4:
        #         bus, message_id, data, timestamp = message
        #         print(f"Bus: {bus}, ID: {hex(message_id)}, Data: {data.hex()}, Timestamp: {timestamp}")
        #     elif len(message) == 3:  # If only 3 elements are returned
        #         bus, message_id, data = message
        #         print(f"Bus: {bus}, ID: {hex(message_id)}, Data: {data.hex()}")
        #     else:
                # print(f"Unexpected message format: {message}")

except KeyboardInterrupt:
    print("\nStopping CAN message reception.")

except Exception as e:
    print("Error receiving CAN messages:", e)

finally:
    # Close the Panda connection
    panda.close()
    print("Panda connection closed.")
