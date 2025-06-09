class FSM(object):
    def __init__(self, initial_state):
        self.state = initial_state

    def transition(self, new_state):
        self.state = new_state

    def get_state(self):
        return self.state

    def __str__(self):
        return f"FSM(current state: {self.state})"