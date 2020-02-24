#include "firmwares/rotorcraft/navigation.h"
#include "modules/nav/nav_survey_rectangle_rotorcraft.h"
#include "modules/nav/nav_survey_rectangle_avoid_rotorcraft.h"
#include <stdio.h>
#include <stdlib.h>

struct Waypoint p1,p2,p3,p4;
float sx1, sx2, sy1, sy2, ax1, ax2, ay1, ay2;
int type, setup, survey_num;
uint8_t util_wp1, util_wp2;
float survey_avoid_grid;
survey_orientation_t survey_avoid_orientation;

void nav_survey_rectangle_avoid_rotorcraft_init(){}


void nav_survey_rectangle_avoid_rotorcraft_setup(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4, uint8_t wp5, uint8_t wp6, float grid, survey_orientation_t so) {
	sx1 = WaypointX(wp1);
	sx2 = WaypointX(wp2);
	sy1 = WaypointY(wp1);
	sy2 = WaypointY(wp2);
	ax1 = WaypointX(wp3);
	ax2 = WaypointX(wp4);
	ay1 = WaypointY(wp3);
	ay2 = WaypointY(wp4);
	util_wp1 = wp5;
	util_wp2 = wp6;
	setup = true;
	survey_num = 0;
	survey_avoid_grid = grid;
	survey_avoid_orientation = so;
	if (sx1 == ax1 && sy1 == ay1) { // top left corner
 		type = 1;
	} else if (abs(sx1 - ax1) < 1.0 && abs(sy2 - ay2) < 1.0) { // bottom left corner
		type = 2;
	} else if (abs(sx2 - ax2) < 1.0 && abs(sy1 - ay1) < 1.0) { // top right corner
		type = 3;
	} else if (abs(sx2 - ax2) < 1.0 && abs(sy2 -ay2) < 1.0) { // bottom right corner
		type = 4;
	} else if (abs(sx1 - ax1) < 1.0) { // against left side, but not on corner
		type = 5;
	} else if (abs(sx2 - ax2) < 1.0) { // against right side, but not on corner
		type = 6;
	} else if (abs(sy1 - ay1) < 1.0) { // against top side
		type = 7;
	} else if (abs(sy2 - ay2) < 1.0) { // against bottom side
		type = 8;
	} else { // fully contained inside of rectangle
		type = 9;
	}
}



bool nav_survey_rectangle_avoid_rotorcraft_run() {
	switch (type) {
		case 1 : 
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 1) survey_num = 0;
		 	}
			switch (survey_num) {
				case 0 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax2, ay1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 1 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax1, ay2, 10};
						struct EnuCoor_f temp_coor_2 = {ax1, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		case 2 :
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 1) survey_num = 0;
	 		}
			switch (survey_num) {
				case 0 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {ax2, ay1, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 1: 
					if (setup) { 
						struct EnuCoor_f temp_coor_1 = {ax2, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		case 3 :
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 1) survey_num = 0;
	 		}
			switch (survey_num) {
				case 0 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {ax1, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2); 
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 1: 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax1, ay2, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2); 
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		case 4 :
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 1) survey_num = 0;
	 		}
			switch (survey_num) {
				case 0 : 
					if (setup) { 
						struct EnuCoor_f temp_coor_1 = {sx1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {ax1, ay2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation); 
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 1 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {ax2, ay1, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2); 
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation); 
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		case 5:
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 2) survey_num = 0;
	 		}
			switch (survey_num) {
				case 0 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, ay1, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation); 
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 1 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax1, ay1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, ay2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 2 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax1, ay2, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		case 6 :
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 2) survey_num = 0;
	 		}
			switch (survey_num) {
				case 0 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, ay1, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 1 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, ay1, 10};
						struct EnuCoor_f temp_coor_2 = {ax2, ay2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 2 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, ay2, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		case 7 :
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 2) survey_num = 0;
	 		}
			switch (survey_num) {
				case 0 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {ax2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run (util_wp1, util_wp2);
					break;
				case 1 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax1, ay2, 10};
						struct EnuCoor_f temp_coor_2 = {ax2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run (util_wp1, util_wp2);
					break;
				case 2 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax2, ay1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run (util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		case 8 :
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 2) survey_num = 0;
	 		}
			switch (survey_num) {
				case 0 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {ax1, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2); 
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 1 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {ax2, ay1, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 2 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax2, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		case 9 :
			if (rectangle_survey_sweep_num == 1) {
				survey_num++;
				setup = true;
				if (survey_num > 3) survey_num = 0;
	 		}
			switch (survey_num) {
				case 0 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, sy1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, ay1, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 1 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, ay1, 10};
						struct EnuCoor_f temp_coor_2 = {ax1, ay2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2);
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 2 : 
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {sx1, ay2, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, sy2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2); 
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				case 3 :
					if (setup) {
						struct EnuCoor_f temp_coor_1 = {ax2, ay1, 10};
						struct EnuCoor_f temp_coor_2 = {sx2, ay2, 10};
						waypoint_set_enu(util_wp1, &temp_coor_1);
						waypoint_set_enu(util_wp2, &temp_coor_2);
						waypoint_globalize (util_wp1);
						waypoint_globalize (util_wp2); 
						nav_survey_rectangle_rotorcraft_setup(util_wp1,util_wp2, survey_avoid_grid, survey_avoid_orientation);
						setup=false;
					}
					return nav_survey_rectangle_rotorcraft_run(util_wp1, util_wp2);
					break;
				default : break;
			}
			break;
		default : break;
	}
	return true; //No survey called, so continue to next iteration
}

