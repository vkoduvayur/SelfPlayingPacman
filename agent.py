from constants import *
import queue
from datetime import datetime
from nodes import Node

class Agent():
    def __init__(self):
        self.initial_state = None
        self.direction = None
        self.action_costs = {}
        self.transition_model = [] # path from initial state to goal state
        # holds all nodes Pacman has ever visited
        self.visited = set()
        self.goal = None

        # Actions are up, down, left, right, which are in constants.py
        # for node to be a goal, we need to know if it contains a pellet

        self.f = open("path.txt", "a")
        self.f.write("init agent\n")
        self.f.close()

    def makeGoal(self):
        self.goal = Node(16, 512)

    def astarSearch(self, init_state):
        now = self.startRecord("astarSearch")
        self.f.write(f"initial state: {init_state}\n")
        self.makeGoal()
        count = 0
        frontier = queue.PriorityQueue()
        frontier.put((0, count, init_state))
        count += 1
        came_from = dict()
        cost_so_far = dict()
        came_from[init_state] = None
        cost_so_far[init_state] = 0

        while not frontier.empty():
            current = frontier.get()[2]
            self.f.write(f"current: {current}\n")

            if current.position == self.goal.position:
                self.goal = current
                break

            for direction, next in current.neighbors.items():
                if next is not None:
                    new_cost = cost_so_far[current] + self.manhattanDistance(current, next)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        priority = new_cost + self.manhattanDistance(self.goal, next)
                        self.f.write(f"next node: {next}\n")
                        self.f.write(f"priority: {priority}\n")
                        frontier.put((priority, count, next))
                        count += 1
                        came_from[next] = -direction
        self.calculatePath(init_state, came_from, cost_so_far)
        self.stopRecord("astarSearch", now)

    def calculatePath(self, init_state, came_from, cost_so_far):
        current = self.goal
        self.f.write(f" cost to goal: {cost_so_far[self.goal]}\n")
        path = []
        while current.position != init_state.position:
            dir = -came_from[current]
            path.append(dir)
            current = current.neighbors[came_from[current]] # came_from holds directions, came_from[current] holds direction taken to get to current
        path.reverse() # path from Pacman's position to goal
        msg = f"path: {path}\n"
        msg = self.directions(msg)
        self.f.write(msg)

    '''STOP = 0
    UP = 1
    DOWN = -1
    LEFT = 2
    RIGHT = -2
    PORTAL = 3'''
    # use string replace function to replace numbers with names of constants
    def directions(self, msg):
        # if positive numbers come first, then negative replaced with positive with negative in front
        # e.g. DOWN -> -UP, RIGHT -> -LEFT
        msg = msg.replace("-1", "DOWN")
        msg = msg.replace("1", "UP")
        msg = msg.replace("-2", "RIGHT")
        msg = msg.replace("2", "LEFT")
        msg = msg.replace("3", "PORTAL")
        return msg

    def search(self, init_state):
        now = self.startRecord("search")
        # only conduct a search if the path is empty
        self.f.write(f"initial state: {init_state}\n")
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
                self.f.write(f"current: {current}\n")
                # neighbors is a dictionary, direction = key, next = value
                for direction, next in current.neighbors.items():
                    self.f.write(f"next: {next}\n")
                    self.f.write(f"direction: {direction}\n")
                    #print(self.visited)
                    # self.direction = direction
                    neighbors = queue.PriorityQueue()
                    # put opposite direction in the next node before putting it into frontier
                    if next not in reached and next not in self.visited and next != None:
                        self.f.write("inside if\n")
                        m_dist = self.manhattanDistance(current, next)
                        self.f.write(f"Manhattan Distance: {m_dist}\n")
                        neighbors.put((-m_dist, (direction, next)))
                        next.direction = -direction
                        # if direction isn't opposite of previous direction, then add new direction to transition model and goal
                        # if direction != 0 - self.direction:
                        # self.transition_model.append(direction)  # shouldn't be opposite of previous direction
                        self.direction = direction  # shouldn't be opposite of previous direction
                        frontier.put(next)
                        reached.add(next)
                        break
                    else:
                        self.direction = direction

                while neighbors.qsize() > 0:
                    direction_tuple = neighbors.get()
                    dir = direction_tuple[1][0]
                    nxt = direction_tuple[1][1]
                    if nxt not in self.visited and current.neighbors[self.direction] == None:
                        self.transition_model.append(dir)
                        self.direction = dir
                    elif current.neighbors[self.direction] != None:
                        self.transition_model.append(self.direction)
        self.stopRecord("search", now)
        return self.transition_model

    # open up file and return time-stamp information
    def startRecord(self, name):
        self.f = open("path.txt", "a")
        now = datetime.now()
        self.f.write(f"start {name}: {now}\n")
        return now

    # take variable 'now' that gets returned from startRecord
    def stopRecord(self, name, now):
        now2 = datetime.now()
        self.f.write(f"stop {name}: {now2}\n")
        duration = now2 - now
        self.f.write(f"total time for {name}: {duration}\n")
        self.f.close()

    # return the distance travelled between two points travelling only vertically and/or horizontally
    # start and end are of type node
    def manhattanDistance(self, start, end):
        return abs(end.position.x - start.position.x) + abs(end.position.y - start.position.y)

    def addDirection(self, next, direction, frontier, reached):
        pass

    # retrieve and remove the next step from the transition model
    def transitionStep(self):
        nextStep = None
        if len(self.transition_model) > 0:
            nextStep = self.transition_model.pop()
        else:
            nextStep = self.direction
        return nextStep
