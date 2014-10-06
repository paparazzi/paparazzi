/*
 * Copyright (C) 2014 Christophe De Wagter
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file fbw_datalink.h
 * Handling of messages coming from ground in FTD
 */

#ifndef FBW_DATALINK_H
#define FBW_DATALINK_H

extern void fbw_datalink_periodic(void);
extern void fbw_datalink_event(void);

#endif /* FBW_DATALINK_H */
