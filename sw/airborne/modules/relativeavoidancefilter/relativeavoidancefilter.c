/*
 * Copyright (C) Mario Coppola
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/relativeavoidancefilter/relativeavoidancefilter.c"
 * @author Mario Coppola
 * Relative Localization Filter for collision avoidance between drones
 */

#include "relativeavoidancefilter.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/multi/traffic_info.h"
#include "modules/stdma/stdma.h"
#include "subsystems/datalink/bluegiga.h"

#include "pprzlink/pprz_transport.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

#define NUAVS 5				// Maximum expected number of drones

#ifndef INS_INT_VEL_ID
#define INS_INT_VEL_ID ABI_BROADCAST
#endif

ekf_filter ekf[NUAVS-1]; 	// EKF structure
btmodel model[NUAVS-1];  	// Bluetooth model structure 
int IDarray[NUAVS-1]; 		// Array of IDs of other MAVs
int8_t srcstrength[NUAVS-1];// Source strength
uint32_t now_ts[NUAVS-1]; 	// Time of last received message from each MAV
int nf;						// Number of filters registered
float RSSIarray[NUAVS-1];	// Recorded RSSI values (so they can all be sent)

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi);

static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi)
{ 
	int i = -1; // Initialize the index of all tracked drones (-1 for null assumption of no drone found).

	// Check if a new aircraft ID is present, if it's a new ID we start a new EKF for it.
	if (( !array_find_int(NUAVS-1, IDarray, ac_id, &i))  // If yes, a new drone is found.
		   && (nf < NUAVS-1))  // If yes, the amount of drones does not exceed the maximum.
	{
		IDarray[nf] = ac_id; 				// Store ID in an array (logging purposes)
		srcstrength[nf] = source_strength;  // Store source strength in an array (logging purposes)
		ekf_filter_new(&ekf[nf]); 			// Initialize an EKF filter for the newfound drone

		// Set up the Q and R matrices and all the rest
		// Weights are based on:
		// Coppola et al, "On-board Communication-based Relative Localization for Collision Avoidance in Micro Air Vehicle teams", 2017
		fmat_scal_mult(EKF_N,EKF_N, ekf[nf].Q, pow(0.5,2.0), ekf[nf].Q);
		fmat_scal_mult(EKF_M,EKF_M, ekf[nf].R, pow(SPEEDNOISE,2.0), ekf[nf].R);
		ekf[nf].Q[0]   	   = 0.01; // Reccomended 0.01 to give this process a high level of trust
		ekf[nf].Q[EKF_N+1] = 0.01;
		ekf[nf].R[0]   = pow(RSSINOISE,2.0);
			
		// Initialize the states
		// Initial position cannot be zero or the filter will divide by zero on initialization
		ekf[i].X[0] = 1.0; // Relative position North
		ekf[i].X[1] = 1.0; // Relative position East
		// The other variables can be initialized at 0
		ekf[i].X[2] = 0.0; // Own Velocity North
		ekf[i].X[3] = 0.0; // Own Velocity East
		ekf[i].X[4] = 0.0; // Relative velocity North
		ekf[i].X[5] = 0.0; // Relative velocity East
		ekf[i].X[6] = 0.0; // Height difference

		ekf[nf].dt       = 0.2;  // Initial assumption for time difference between messages (STDMA code runs at 5Hz)
		model[nf].Pn     = -63;  // Expected RSSI at 1m (based on experience)
		model[nf].gammal = 2.0;	 // Expected Space-loss parameter (based on free space assumption)
		nf++; 					 // Number of filter is present is increased
	}
	// Else, if we do recognize the ID, then we can update the measurement message data
	else if ((i != -1) || (nf == (NUAVS-1)) )
	{
		RSSIarray[i] = (float)rssi; // Store RSSI in array (for logging purposes)

		// Get own velocities
		float ownVx = stateGetSpeedEnu_f()->y; // Velocity North in NED
		float ownVy = stateGetSpeedEnu_f()->x; // Velocity East in NED
		// Bind to realistic amounts to avoid occasional spikes/NaN/inf errors
		keepbounded(&ownVx,-2.0,2.0);
		keepbounded(&ownVy,-2.0,2.0);

		// Make the filter only in Guided mode (flight).
		// This is because it is best for the filter should only start once the drones are in motion, 
		// otherwise it might diverge while drones are not moving.
		if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
		{
			ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6); // Update the time between messages
			
			// Get the velocity in NED for the tracked aircraft
			float trackedVx, trackedVy;
			polar2cart(acInfoGetGspeed(ac_id), acInfoGetCourse(ac_id), &trackedVx, &trackedVy); // get North and East velocities (m/s)
			// As for own velocity, bind to realistic amounts to avoid occasional spikes/NaN/inf errors
			keepbounded(&trackedVx,-2.0,2.0);
			keepbounded(&trackedVy,-2.0,2.0);

			// Construct measurement vector Y for EKF using the latest data obtained.
			// Y = [RSSI owvVx ownVy trackedVx trackedVy dh], EKF_M = 6 as defined in discreteekf.h
			float Y[EKF_M];
			Y[0] = (float)rssi; 	//RSSI measurement
			Y[1] = ownVx; 	   		// Own velocity North (NED frame)
			Y[2] = ownVy;			// Own velocity East  (NED frame)
			Y[3] = trackedVx;  		// Velocity of other drone Norht (NED frame)
			Y[4] = trackedVy;		// Velocity of other drone East  (NED frame)
			Y[5] = acInfoGetPositionUtm_f(ac_id)->alt - stateGetPositionEnu_f()->z;  // Height difference
			
			// Run the steps of the EKF, but only if velocity difference is significant (to filter out minimal noise)
			if (  sqrt( pow(Y[1]-Y[3],2) + pow(Y[2]-Y[4],2) ) > 0.05 )
			{
				ekf_filter_predict(&ekf[i], &model[i]); // Prediction step of the EKF 
				ekf_filter_update(&ekf[i], Y);	// Update step of the EKF
			}
		}

		now_ts[i] = get_sys_time_usec();  // Store latest time

	}
};

// bool alternate;
// static void send_rafilterdata(struct transport_tx *trans, struct link_device *dev)
// {	
// 	// Store the relative localization data
// 	uint8_t i;
// 	// To avoid overflowing, it is best to send the data of each tracked drone separately.
// 	// To do so, we can cycle through the filters at each new timestep.
// 	// cnt++;
// 	// if (cnt == nf)
// 	// 	cnt = 0;
	
// 	// TODO: MAKE THIS SWITCHING NOT LAZY BUT PROPER FOR UNLIMITED MAVS
// 	// array_shiftleft(vec, nf, 1);
// 	// vec is a vector of 0 to nf defined each time nf increases
// 	if (nf == 2) {
// 		if (alternate) {
// 			alternate = false;
// 			i = 1;
// 		}
// 		else {
// 			alternate = true;
// 			i = 0;
// 		}
// 	}
// 	else { //only data on first
// 		i = 0;
// 	}

// 	int8_t id = (int8_t)IDarray[i]; // Extract ID

// 	pprz_msg_send_RLFILTER(
// 		trans, dev, AC_ID,			 // Standard stuff
// 		&id,			     		 // ID of the tracked UAV in question
// 		&RSSIarray[i], 		    	 // Received ID and RSSI
// 		&srcstrength[i],		     // Source strength
// 		&ekf[i].X[0], &ekf[i].X[1],  // Relative position [North, East]
// 		&ekf[i].X[2], &ekf[i].X[3],  // Own velocity [North, East]
// 		&ekf[i].X[4], &ekf[i].X[5],  // Relative velocity of other drone [North, East]
// 		&ekf[i].X[6]				 // Height separation [Down]
// 		);  				 
// };

void relativeavoidancefilter_init(void)
{
	array_make_zeros_int(NUAVS-1, IDarray); // Clear out the known IDs
	nf = 0; // Number of active filters upon initialization

	AbiBindMsgRSSI(ABI_BROADCAST, &rssi_ev, bluetoothmsg_cb); // Subscribe to the ABI RSSI messages
	// register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RLFILTER, send_rafilterdata); // Send out the filter data
};

void relativeavoidancefilter_periodic(void)
{	
	/*********************************************
		Sending speed directly between drones
	 *********************************************/
	// Convert course to the proper format (NED)
	float spd, crs;
	cart2polar(stateGetSpeedEnu_f()->y, stateGetSpeedEnu_f()->x, &spd, &crs); // Get the total speed and course
	wrapTo2Pi(&crs); 

	int32_t course = (int32_t)(crs*(1e7)); // Typecast crs into a int32_t type integer with proper unit (see gps.course in gps.h)
	uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(course) / 1e7) / 2)) & 0x7FF) <<
			21; 									  // bits 31-21 course (where the magnitude is pointed at)
	multiplex_speed |= (((uint32_t)(spd*100)) & 0x7FF) << 10;         // bits 20-10 speed in cm/s
	multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);        // bits 9-0 z velocity in cm/s
	int16_t alt = (int16_t)(stateGetPositionEnu_f()->z*100.0);

	// Use this for communication via the AR drones + Bluetooth dongle
	DOWNLINK_SEND_GPS_SMALL(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &multiplex_speed, &gps.lla_pos.lat, &gps.lla_pos.lon, &alt);
	
	// Message through USB bluetooth dongle to other drones
	// DOWNLINK_SEND_GPS_SMALL(stdma_trans, bluegiga_p, &multiplex_speed, &gps.lla_pos.lat, &gps.lla_pos.lon, &alt);
}
