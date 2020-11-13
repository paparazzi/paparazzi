import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from grid import create_grid_opt
from planning import a_star

#from bresenham import bresenham

#plt.rcParams['figure.figsize'] = 7, 5

# This is the same obstacle data from the previous lesson.
#filename = 'colliders.csv'
filename = 'map_2D_voliere.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

# Static drone altitude (meters)
drone_altitude = 2

# Minimum distance stay away from obstacle (cm)
safe_distance = 20

# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
grid, grid_wall = create_grid_opt(data, drone_altitude, safe_distance)

# equivalent to
# plt.imshow(np.flip(grid, 0))
#plt.imshow(grid, cmap='Greys', origin='lower') 

#plt.xlabel('EAST')

#plt.ylabel('NORTH')
#plt.show()

start_ne = (450, 75)
goal_ne = (400, 450)

def heuristic_func(position, goal_position):
    # TODO: write a heuristic!
    h = np.sqrt((position[0] - goal_position[0])**2 + (position[1] - goal_position[1])**2)
    return h

# TODO: use `a_star` to compute the lowest cost path
path, cost, found = a_star(grid, heuristic_func, start_ne, goal_ne)
print(path, cost)

grid_wall = np.ma.masked_where(grid_wall < 0.95, grid_wall)
fig, ax = plt.subplots()
ax.imshow(grid, cmap=cm.coolwarm, origin='lower')
ax.imshow(grid_wall, cmap='Greys_r', origin='lower')

# For the purposes of the visual the east coordinate lay along
# the x-axis and the north coordinates long the y-axis.
plt.plot(start_ne[1], start_ne[0], 'c*')
plt.plot(goal_ne[1], goal_ne[0], 'r*')

if path is not None:
    pp = np.array(path)
    plt.plot(pp[:, 1], pp[:, 0], 'r')
    plt.plot(pp[:, 1], pp[:, 0], 'wo')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=20):   
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    pruned_path = [p for p in path]
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        
        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path

pruned_path = prune_path(path)
print(pruned_path)

grid_wall = np.ma.masked_where(grid_wall < 0.95, grid_wall)
fig, ax = plt.subplots()
ax.imshow(grid, cmap=cm.coolwarm, origin='lower')
ax.imshow(grid_wall, cmap='Greys_r', origin='lower')

if pruned_path is not None:
    pp = np.array(pruned_path)
    plt.plot(pp[:, 1], pp[:, 0], 'r')
    plt.plot(pp[:, 1], pp[:, 0], 'wo')

plt.plot(start_ne[1], start_ne[0], 'c*')
plt.plot(goal_ne[1], goal_ne[0], 'r*')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()






