from panda import Panda

class CANInterface:
    def __init__(self):
        try:
            self.panda = Panda()
            self.panda.set_power_save(False)
        except Exception as e:
            print(f"Error initializing Panda: {e}")
    def read_messages(self):
        return self.panda.can_recv()