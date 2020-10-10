import sys
import math


class Node:
    def __init__(self, pos: (), parent: ()):
        self.pos = pos
        self.parent = parent

        self.g = 0  # Actual distance to goal
        self.h = 0  # Heuristic distance to goal
        self.f = 0  # Total cost

    # Compares nodes
    def __eq__(self, other):
        return self.pos == other.pos

    # Sort nodes
    def __lt__(self, other):
        return self.f < other.f

    # Print node
    def __repr__(self):
        return "({0},{1})".format(self.pos, self.f)


# draw enviroment of agent through a grid
def draw_grid(map, width, height, spacing=2, **kwargs):
    for y in range(height):
        for x in range(width):
            print("%%-%ds" % spacing % draw_tile(map, (x, y), kwargs), end="")
        print()


# define tile
def draw_tile(map, pos, kwargs):

    # Assign value from specific tile from the positions specified in sample maze
    v = map.get(pos)
    # Check the current status of that tile and return corresponding character
    if "path" in kwargs and pos in kwargs["path"]:
        v = "|"
    if "start" in kwargs and pos == kwargs["start"]:
        v = "S"
    if "goal" in kwargs and pos == kwargs["goal"]:
        v = "G"

    return v


# Depth-first search
def depthfirst_search(map, start, end):

    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    nstart = Node(start, None)
    ngoal = Node(end, None)
    # Add the start node
    open.append(nstart)

    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        ncurrent = open.pop(0)
        # Add the current node to the closed list
        closed.append(ncurrent)

        # Check if we have reached the goal, return the path
        if ncurrent == ngoal:
            path = []
            while ncurrent != nstart:
                path.append(ncurrent.pos)
                ncurrent = ncurrent.parent
            # path.append(start)
            # Return reversed path
            return path[::-1]
        # Unzip the current node position
        (x, y) = ncurrent.pos
        # Get neighbors
        closeneighbor = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        # Loop neighbors
        for next in closeneighbor:
            # Get value from map
            map_value = map.get(next)
            # Check if the node is a wall
            if map_value == "%":
                continue
            # Create a neighbor node
            neighbor = Node(next, ncurrent)
            # Check if the neighbor is in the closed list
            if neighbor in closed:
                continue
            # Generate heuristics (Manhattan distance)
            neighbor.g = abs(neighbor.pos[0] - nstart.pos[0]) + abs(
                neighbor.pos[1] - nstart.pos[1]
            )
            neighbor.h = abs(neighbor.pos[0] - ngoal.pos[0]) + abs(
                neighbor.pos[1] - ngoal.pos[1]
            )
            neighbor.f = neighbor.g + neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if add_to_open(open, neighbor) == True:
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None


# Breath-first search
def breadthfirst_search(map, start, end):
    # Create a list for the open and close nodes
    open = []
    closed = []

    # Create a start node and a goal nose and initialize it
    nstart = Node(start, None)
    ngoal = Node(end, None)

    open.append(nstart)  # add start node to open list of nodes

    # while the open list is empty, sort the open list to get the node with the lowest cost first
    while len(open) > 0:
        open.sort()

        # remove current and lowest costign node from the open list and "move" to closed list of nodes.
        ncurrent = open.pop(0)
        closed.append(ncurrent)

        # Check wether we have reached the goal, and return the path taken
        if ncurrent == ngoal:
            path = []
            while ncurrent != nstart:
                path.append(ncurrent.pos)
                ncurrent = ncurrent.parent
            return path[::-1]

        # Assign current node position x and y values to position on the map
        (x, y) = ncurrent.pos
        # Get neighbor tiles position
        closeneighbor = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]

        # get value of each neighbor's tile
        for next in closeneighbor:
            map_value = map.get(next)

            if map_value == "%":
                continue
            # Create a neighbor node if no wall is found and path is clear
            neighbor = Node(next, ncurrent)

            # Check if the neighbor is in the closed list
            if neighbor in closed:
                continue

            # Generate heuristics using Euclidean distance
            neighbor.g = math.sqrt(
                math.pow(neighbor.pos[0] - nstart.pos[0], 2)
                + math.pow(neighbor.pos[1] - nstart.pos[1], 2)
            )
            neighbor.h = math.sqrt(
                math.pow(neighbor.pos[0] - ngoal.pos[0], 2)
                + math.pow(neighbor.pos[1] - ngoal.pos[1], 2)
            )
            neighbor.f = neighbor.g + neighbor.h

            # Check if neighbor is in the open list and if it has a lower f value and add neighbor to the list
            if add_to_open(open, neighbor) == True:
                open.append(neighbor)
    return None


# A* search implementation
def astar_search(map, start, end):

    # Create a list for the open and close nodes
    open = []
    closed = []

    # Create a start node and a goal nose and initialize it
    nstart = Node(start, None)
    ngoal = Node(end, None)

    open.append(nstart)  # add start node to open list of nodes

    # while the open list is empty, sort the open list to get the node with the lowest cost first
    while len(open) > 0:
        open.sort()

        # remove current and lowest costign node from the open list and "move" to closed list of nodes.
        ncurrent = open.pop(0)
        closed.append(ncurrent)

        # Check wether we have reached the goal, and return the path taken
        if ncurrent == ngoal:
            path = []
            while ncurrent != nstart:
                path.append(ncurrent.pos)
                ncurrent = ncurrent.parent
            return path[::-1]

        # Assign current node position x and y values to position on the map
        (x, y) = ncurrent.pos
        # Get neighbor tiles position
        closeneighbor = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]

        # get value of each neighbor's tile
        for next in closeneighbor:
            map_value = map.get(next)

            if map_value == "%":
                continue
            # Create a neighbor node if no wall is found and path is clear
            neighbor = Node(next, ncurrent)

            # Check if the neighbor is in the closed list
            if neighbor in closed:
                continue

            # Generate heuristics using Euclidean distance
            neighbor.g = math.sqrt(
                math.pow(neighbor.pos[0] - nstart.pos[0], 2)
                + math.pow(neighbor.pos[1] - nstart.pos[1], 2)
            )
            neighbor.h = math.sqrt(
                math.pow(neighbor.pos[0] - ngoal.pos[0], 2)
                + math.pow(neighbor.pos[1] - ngoal.pos[1], 2)
            )
            neighbor.f = neighbor.g + neighbor.h

            # Check if neighbor is in the open list and if it has a lower f value and add neighbor to the list
            if add_to_open(open, neighbor) == True:
                open.append(neighbor)
    return None


# Check wether a neighbor should be added to the open list
def add_to_open(open, neighbor):
    for node in open:
        if neighbor == node and neighbor.f >= node.f:
            return False
    return True


def main():

    # Initialize a grid
    map = {}
    chars = ["c"]
    start = None
    end = None
    width = 0
    height = 0

    # Open a file
    fp = open(sys.argv[3], "r")

    # Loop through file lines
    while len(chars) > 0:
        chars = [str(i) for i in fp.readline().strip()]

        # calculate width
        width = len(chars) if width == 0 else width

        # recreate file map
        for x in range(len(chars)):
            map[(x, height)] = chars[x]
            if chars[x] == "S":
                start = (x, height)
            elif chars[x] == "G":
                end = (x, height)

        # Increase the height of the map
        if len(chars) > 0:
            height += 1

    # Close the file
    fp.close()

    # Find and display the closest path from start(S) to the Goal(G)
    print(sys.argv[2])
    if sys.argv[2] == "astar":
        path = astar_search(map, start, end)
    if sys.argv[2] == "depth":
        path = depthfirst_search(map, start, end)
    if sys.argv[2] == "breadth":
        path = breadthfirst_search(map, start, end)

    ##path = depthfirst_search(map, start, end)

    ##path = astar_search(map, start, end)

    print()
    print("Path points: ", path)
    print()
    draw_grid(map, width, height, spacing=1, path=path, start=start, goal=end)
    print()
    print("Solution cost: {0}".format(len(path)))
    print()


if __name__ == "__main__":
    main()
