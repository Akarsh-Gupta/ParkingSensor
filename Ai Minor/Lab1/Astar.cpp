from queue import PriorityQueue

class Graph:
    def _init_(self):
        self.edges = {
            'A': {'B': 2, 'C': 1},
            'B': {'D': 7, 'E': 3},
            'C': {'F': 5},
            'D': {'G': 1, 'H': 2},
            'E': {'H': 4},
            'F': {'I': 3, 'J': 2},
            'G': {'J': 4},
            'H': {'J': 1},
            'I': {'J': 6},
            'J': {}
        }
         # define weights for each vertex
        self.weights = {'A': 19.07, 'B': 20, 'C': 12.21, 'D': 14.32, 'E': 5.42, 'F': 6.55, 'G': 7.56, 'H': 18.46, 'I': 15.34, 'J': 12.34}
    
    def neighbors(self, node):
        return self.edges[node].keys()
    
    def cost(self, current, next):
        return self.edges[current][next]
    
    def heuristic(self, current, goal):
        # use the weights dictionary to get the weight for each vertex
        current_weight = self.weights[current]
        goal_weight = self.weights[goal]
        # use the absolute difference between the weights as the heuristic value
        return abs(current_weight - goal_weight)

def astar(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + graph.heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path

def astar(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + graph.heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path

def main():
    graph = Graph()
    start = 'A'
    goal = 'G'

    path = astar(graph, start, goal)

    print("Shortest path from {} to {} is: {}".format(start, goal, path))

if _name_ == "_main_":
    main()