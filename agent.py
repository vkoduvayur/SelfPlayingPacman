from constants import *
class Agent():
    def __init__(self):
        self.initial_state = None
        self.goal_state = None
        self.action_costs = {}
        self.transition_model = [] # path from initial state to goal state
        # Actions are up, down, left, right, which are in constants.py

    def search(self, init_state):
        # only conduct a search if the path is empty
        if len(self.transition_model) == 0:
            self.initial_state = init_state
            self.transition_model = [LEFT]
        return self.transition_model

    # retrieve and remove the next step from the transition model
    def transitionStep(self):
        nextStep = self.transition_model.pop()
        return nextStep
