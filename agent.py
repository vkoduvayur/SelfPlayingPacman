from constants import *
import queue
class Agent():
    def __init__(self):
        self.initial_state = None
        self.goal_state = None
        self.action_costs = {}
        self.transition_model = [] # path from initial state to goal state
        # holds all nodes Pacman has ever visited
        self.visited = set()

        # Actions are up, down, left, right, which are in constants.py
        # for node to be a goal, we need to know if it contains a pellet
    def goal(self, node):
        return False

    def search(self, init_state):
        # only conduct a search if the path is empty
        print(f"initial state: {init_state}")
        if len(self.transition_model) == 0:
            self.initial_state = init_state
            # self.transition_model = [LEFT]
            frontier = queue.Queue()
            # direction is stored in init_state and set to None by default
            frontier.put(init_state)
            self.visited.add(init_state)
            reached = set()
            reached.add(init_state)
            while not frontier.empty():
                current = frontier.get()
                print(f"current: {current}")
                # neighbors is a dictionary, direction = key, next = value
                for direction, next in current.neighbors.items():
                    print(f"next: {next}")
                    # put opposite direction in the next node before putting it into frontier
                    if next not in reached and next != None and next not in self.visited:
                        next.direction = -direction
                        self.transition_model.append(direction)
                        self.goal = direction
                        frontier.put(next)
                        reached.add(next)
                        break
        return self.transition_model

    # retrieve and remove the next step from the transition model
    def transitionStep(self):
        if len(self.transition_model) > 0:
            nextStep = self.transition_model.pop()
        else:
            nextStep = self.goal
        return nextStep
