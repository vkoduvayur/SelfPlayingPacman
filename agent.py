from constants import *
import queue
class Agent():
    def __init__(self):
        self.initial_state = None
        self.goal = None
        self.action_costs = {}
        self.transition_model = [] # path from initial state to goal state
        # holds all nodes Pacman has ever visited
        self.visited = set()

        # Actions are up, down, left, right, which are in constants.py
        # for node to be a goal, we need to know if it contains a pellet

        self.f = open("path.txt", "a")
        self.f.write("init agent\n")


    def search(self, init_state):
        # self.f.open("path.txt", "a")
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
                    print(f"direction: {direction}")
                    #print(self.visited)
                    # self.goal = direction
                    neighbors = queue.PriorityQueue()
                    # put opposite direction in the next node before putting it into frontier
                    if next not in reached and next not in self.visited and next != None:
                        print("inside if")
                        m_dist = self.ManhattanDistance(current, next)
                        print(f"Manhattan Distance: {m_dist}")
                        neighbors.put((-m_dist, (direction, next)))
                        next.direction = -direction
                        # if direction isn't opposite of previous direction, then add new direction to transition model and goal
                        # if direction != 0 - self.goal:
                        # self.transition_model.append(direction)  # shouldn't be opposite of previous direction
                        self.goal = direction  # shouldn't be opposite of previous direction
                        frontier.put(next)
                        reached.add(next)
                        break
                    else:
                        self.goal = direction

                while neighbors.qsize() > 0:
                    direction_tuple = neighbors.get()
                    dir = direction_tuple[1][0]
                    nxt = direction_tuple[1][1]
                    if nxt not in self.visited and current.neighbors[self.goal] == None:
                        self.transition_model.append(dir)
                        self.goal = dir
                    elif current.neighbors[self.goal] != None:
                        self.transition_model.append(self.goal)
        # self.f.close()
        return self.transition_model

    # return the distance travelled between two points travelling only vertically and/or horizontally
    # start and end are of type node
    def ManhattanDistance(self, start, end):
        return abs(end.position.x - start.position.x) + abs(end.position.y - start.position.y)

    def addDirection(self, next, direction, frontier, reached):
        pass

    # retrieve and remove the next step from the transition model
    def transitionStep(self):
        nextStep = None
        if len(self.transition_model) > 0:
            nextStep = self.transition_model.pop()
        else:
            nextStep = self.goal
        return nextStep
