from panda import Panda

# Connect to the Panda device
panda = Panda()

# print(Panda.list())

# Flash the firmware
print("Flashing Panda firmware...")
panda.flash()
print("Firmware flashed successfully!")
