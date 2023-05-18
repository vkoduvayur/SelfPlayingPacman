import random

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
        self.maze_nodes = None
        self.pellets = None
        self.ghosts = None

        # Actions are up, down, left, right, which are in constants.py
        # for node to be a goal, we need to know if it contains a pellet

        self.f = open("path.txt", "a")
        self.f.write("init agent\n")
        self.f.close()

    # makes a static goal
    def makeGoal(self, node):
        node = self.maze_nodes.nodesLUT[(node.position.x, node.position.y)]
        self.goal = node
        return self.goal

    # gets a random goal based on pellets
    # TODO: add code to choose a node that avoids the ghosts
    def getRandNodeGoal(self, init_state):
        # print("Entering getRandNodeGoal()")
        goal = None
        if self.goal is None or self.goal.position == init_state.position:
            # no 'None' inside maze nodes
            if self.goal is None:
                key, goal = random.choice(list(self.maze_nodes.nodesLUT.items()))
                # print(f"if none goal: {goal}")
                self.goal = goal
                # print(f"if none self.goal: {self.goal}")
            # could randomly choose same goal position, do a loop until it produces a different goal position
            while self.goal.position == init_state.position:
                key, goal = random.choice(list(self.maze_nodes.nodesLUT.items()))
                # print(f"if none goal: {goal}")
                self.goal = goal
        goal = self.goal
        return goal

    def astarSearch(self, init_state, goal):
        self.initial_state = init_state
        self.goal = goal
        now = self.startRecord("astarSearch")
        self.f.write(f"initial state: {init_state}\n")
        self.f.write(f"goal state: {self.goal}\n")
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
            # self.f.write(f"current: {current}\n")

            if current.position == self.goal.position:
                break

            for direction, next in current.neighbors.items():
                if next is not None:
                    new_cost = cost_so_far[current] + self.manhattanDistance(current, next)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        priority = new_cost + self.manhattanDistance(self.goal, next)
                        # self.f.write(f"next node: {next}\n")
                        # self.f.write(f"priority: {priority}\n")
                        frontier.put((priority, count, next))
                        count += 1
                        came_from[next] = -direction
        self.calculatePath(init_state, came_from, cost_so_far)
        self.stopRecord("astarSearch", now)

    def calculatePath(self, init_state, came_from, cost_so_far):
        current = self.goal
        self.f.write(f" cost to goal: {cost_so_far[self.goal]}\n")
        path = []
        self.f.write(f"nodes on path (from goal to init_state):\n")
        while current.position != init_state.position:
            self.f.write(f"current: {current}\n")
            dir = -came_from[current]
            path.append(dir)
            current = current.neighbors[came_from[current]] # came_from holds directions, came_from[current] holds direction taken to get to current
        self.f.write(f"current: {current}\n")
        path.reverse() # path from Pacman's position to goals
        msg = f"path (from init_state to goal): {path}\n"
        msg = self.directions(msg)
        self.f.write(msg)
        # transition model returns the path taken to reach the goal
        self.transition_model = path

    # gets the maze nodes
    def getMazeNodes(self, maze_nodes):
        self.maze_nodes = maze_nodes

    def getPellets(self, pellets):
        self.pellets = pellets

    def getGhosts(self, ghosts):
        self.ghosts = ghosts

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
                    ## print(self.visited)
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
        # print("Enter transitionStep()")
        if len(self.transition_model) > 0:
            self.direction = self.transition_model.pop(0)
            msg = f"self.direction: {self.direction}"
            msg = self.directions(msg)
            # print(msg)
        return self.direction # self.direction saves direction in agent
