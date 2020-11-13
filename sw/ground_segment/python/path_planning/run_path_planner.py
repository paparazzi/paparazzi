#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from grid import create_grid_opt, heuristic_func, point, collinearity_check, prune_path
from planning import a_star

import sys
from os import path, getenv

from math import radians
from time import sleep
from numpy import linalg as la
from scipy.spatial import distance
import queue

import time

# Paparazzi guidance api
from guidance_common import Rotorcraft , Guidance


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Guided mode example")
    parser.add_argument("-ti", "--target_id", dest='target_id', default=0, type=int, help="Target aircraft ID")
    
    args = parser.parse_args()

    interface = None
    target_id = args.target_id
    follower_id = 10
    
    x_voliere_offset = 250
    y_voliere_offset = 250
    scale = 100
    
    
    g = Guidance(interface=interface, quad_ids=[target_id, follower_id])
    for rc in g.rotorcrafts:
        if rc.id == target_id:
            start_x = rc.X[0]
            start_y = rc.X[1]
    print(start_x)
    print(start_y)
    
    start_point = ( int(start_x*scale + x_voliere_offset), int(start_y*scale + y_voliere_offset) )
    print(start_point)
        
    # Define goal points in the grid (cm)
    goal_point = (400, 450)

    # This file describe the obstacles "walls" in the voliere.
    filename = 'map_2D_voliere.csv'
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

    # Static drone altitude (meters)
    drone_altitude = 1.5

    # Minimum distance stay away from obstacle (cm)
    safe_distance = 20

    # Use `create_grid` to create a grid configuration space of the obstacle data.
    grid, grid_wall = create_grid_opt(data, drone_altitude, safe_distance)

    # Use `a_star` to compute the lowest cost path
    path, cost, found_path = a_star(grid, heuristic_func, start_point, goal_point)

    if found_path == True:
        print('*************')
        print('Found a path!')
        print('*************')
        
        g.step = 0.1
        sleep(0.5)
        g.jump_to_block(2) # start engines
        sleep(0.5)
        g.jump_to_block(3) # takeoff
        sleep(2)
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    pruned_path = prune_path(path)

    # Adding static altitude to waypoints and scaling
    static_altitude = 1.5

    waypoints = []
    for j in range(len(pruned_path)):
        x_pos_scaled = (pruned_path[j][0] - x_voliere_offset)/scale
        y_pos_scaled = (pruned_path[j][1] - y_voliere_offset)/scale
        waypoints.append( [x_pos_scaled, y_pos_scaled, static_altitude] )
    
    try:
        g.set_guided_mode(quad_id=target_id)
        last_target_yaw = 0.0
        total_time = 0.0
        i = 0
        for p_des in waypoints:
            point_not_reached = True
            while point_not_reached:
                # TODO: make better frequency managing
                sleep(g.step)
                total_time = total_time + g.step
                # print('G IDS : ',g.ids) # debug....
                # policy_input = np.zeros(Settings.TOTAL_STATE_SIZE) # initializing policy input
                for rc in g.rotorcrafts:
                    rc.timeout = rc.timeout + g.step
     
                    if rc.id == target_id: # we've found the target
                        # print('Position : ',rc.X)
                        # print('Velocity : ',rc.V)
                        err_p = p_des - rc.X
                        V_des = err_p * 0.4
                        err_V = V_des - rc.V
                        print("X: %.2f; Y: %.2f; Z: %.2f; Xerr-norm: %.2f; Vdes: %.2f; Verr %.2f" %(rc.X[0], rc.X[1], rc.X[2], abs(err_p[0]+err_p[1]+err_p[2]), V_des[0], err_V[0]))
                        # print(err_p)
                        # print(p_des)
                        # print(err_V)
                        pos_x_sp = p_des[0]
                        pos_y_sp = p_des[1]
                        pos_z_sp = -p_des[2]
 
                #g.accelerate(north = err_V[0]*0.9, east = err_V[1]*0.9, down = drone_altitude, quad_id=target_id)
                g.goto_ned(pos_x_sp, pos_y_sp, pos_z_sp, heading=0.0)
                #if distance.euclidean([p_des[0], p_des[1]], [rc.X[0],rc.X[1]]) < 0.25: 
                if abs(err_p[0]+err_p[1]+err_p[2]) < 0.25: 
                    i +=1
                    print('Passing to the next point', i)
                    point_not_reached = False
                #print("Deep guidance command: a_x: %.2f; a_y: %.2f; a_z: %.2f" %( deep_guidance[1], deep_guidance[2], deep_guidance[3]))
                # print("Time: %.2f; X: %.2f; Vx: %.2f; Ax: %.2f" %(total_time, ))
        g.set_nav_mode(quad_id = target_id)
        sleep(2)
        g.jump_to_block(4) # do to standby point    
        sleep(5)
        g.jump_to_block(13) # land here
        grid_wall = np.ma.masked_where(grid_wall < 0.95, grid_wall)
        fig, ax = plt.subplots()
        ax.imshow(grid, cmap=cm.coolwarm, origin='lower')
        ax.imshow(grid_wall, cmap='Greys_r', origin='lower')

        if pruned_path is not None:
            pp = np.array(pruned_path)
            plt.plot(pp[:, 1], pp[:, 0], 'r')
            plt.plot(pp[:, 1], pp[:, 0], 'wo')

        plt.plot(start_point[1], start_point[0], 'c*')
        plt.plot(goal_point[1], goal_point[0], 'r*')

        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()
        
        exit()


    except (KeyboardInterrupt, SystemExit):
        print('Shutting down...')
        # g.set_nav_mode()
        g.shutdown()
        sleep(0.6)
        exit()


if __name__ == '__main__':
    main()

#EOF
