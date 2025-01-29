from panda import Panda
panda = Panda()
# panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)  # Allow all CAN messages
# panda.set_power_save(False)
# panda.can_recv()
print(panda.health())