from enum import Enum
from queue import PriorityQueue
import numpy as np

class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    NE = (-1, 1, np.sqrt(2))
    SE = (1, 1, np.sqrt(2))
    SW = (1, -1, np.sqrt(2))
    NW = (-1, -1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN, Action.NE, Action.SE, Action.SW, Action.NW]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)

    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid.remove(Action.NE)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid.remove(Action.SE)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid.remove(Action.SW)
    if x - 1 < 0 or y - 1 <  0 or grid[x - 1, y - 1] == 1:
        valid.remove(Action.NW)

    return valid


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        None
    return path[::-1], path_cost, found


