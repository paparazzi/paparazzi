import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from grid import create_grid_opt
from planning import a_star

#from bresenham import bresenham

plt.rcParams['figure.figsize'] = 7, 5

# This is the same obstacle data from the previous lesson.
#filename = 'colliders.csv'
filename = 'create.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

# Static drone altitude (meters)
drone_altitude = 4

# Minimum distance stay away from obstacle (meters)
safe_distance = 20

# TODO: Use `create_grid` to create a grid configuration space of
# the obstacle data.
grid, grid_wall = create_grid_opt(data, drone_altitude, safe_distance)

grid_wall = np.ma.masked_where(grid_wall < 0.95, grid_wall)

# equivalent to
# plt.imshow(np.flip(grid, 0))

#plt.imshow(grid, cmap='Greys', origin='lower') 

fig, ax = plt.subplots()
ax.imshow(grid, cmap=cm.coolwarm, origin='lower')
ax.imshow(grid_wall, cmap='Greys_r', origin='lower')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()
