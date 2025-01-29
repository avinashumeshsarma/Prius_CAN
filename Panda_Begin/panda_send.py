import os


from panda import Panda

# Simulate a Panda connection
panda = Panda()
panda.set_safety_mode(Panda.SAFETY_TOYOTA)
panda.can_send(0x1aa, b'message', 0)
