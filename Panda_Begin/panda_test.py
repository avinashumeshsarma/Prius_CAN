import sys
sys.path.insert(0, "/home/avinashumeshsharma/panda")

from panda import Panda
# import lanelet2
panda = Panda()
# panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)  # Allow all CAN messages
# panda.set_power_save(False)
# panda.can_recv()
# print(type(panda.health()["safety_rx_invalid"]))
health_data = panda.health()
print("\nPanda Health Data Types:\n")
    
for key, value in health_data.items():
    print(f"{key}: {type(value).__name__}")