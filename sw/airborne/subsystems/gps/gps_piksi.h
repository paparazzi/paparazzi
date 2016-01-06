/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file subsystems/gps/gps_piksi.h
 *
 * Driver for Piksi modules from Swift-Nav
 *
 * http://docs.swiftnav.com/wiki/Piksi_Integration_Tutorial
 * https://github.com/swift-nav/sbp_tutorial
 */

#ifndef GPS_PIKSI_H
#define GPS_PIKSI_H

// #define GPS_NB_CHANNELS 10

#define PIKSI_HEARTBEAT_MSG

#if GPS_SECONDARY_PIKSI
#define PIKSI_GPS_LINK GPS_SECONDARY_PORT
#define SecondaryGpsImpl piksi
#else
#define PrimaryGpsImpl piksi
#endif
#if GPS_PRIMARY_PIKSI
#define PIKSI_GPS_LINK GPS_PRIMARY_PORT
#endif

extern void piksi_gps_event(void);
extern void piksi_gps_impl_init(void);
extern void piksi_gps_register(void);

/*
 * Reset base station position
 */
extern void gps_piksi_set_base_pos(void);

/*
 * The GPS event
 */
//#define GpsEvent gps_piksi_event

#endif /* GPS_PIKSI_H */
