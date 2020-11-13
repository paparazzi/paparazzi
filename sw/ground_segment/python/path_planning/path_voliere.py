import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from planning import a_star
from scipy.spatial import distance

waypoints_xy = [(450, 75), (419, 106), (247, 106), (246, 133), (266, 155), (282, 191), (304, 232), (315, 264), (323, 280), (246, 357), (246, 383), (278, 394), (317, 412), (354, 429), (390, 445), (400, 450)]

# Adding altitude to waypoints and scaling
drone_altitude = 2

x_voliere_offset = 250
y_voliere_offset = 250
scale = 100

waypoints = []
for j in range(len(waypoints_xy)):
    x_pos_scaled = (waypoints_xy[j][0] - x_voliere_offset)/scale
    y_pos_scaled = (waypoints_xy[j][1] - y_voliere_offset)/scale
    waypoints.append( [x_pos_scaled, y_pos_scaled, drone_altitude] )

print(waypoints)
next_waypoint = waypoints[0]
print('Next waypoint:', next_waypoint)

for k in range(len(waypoints)):
    # Measure actual point
    actual_point = [499, 100, 2]
    dist_to_waypoint = distance.euclidean(waypoints[k], actual_point)
    #print(dist_to_waypoint)
        
    if dist_to_waypoint <= 10:
        if k+1 < len(waypoints):
            next_waypoint = waypoints[k+1]
            print('Next waypoint:', next_waypoint)

    
