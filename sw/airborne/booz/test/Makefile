##
# $Id$
#  
# Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
# 
# This file is part of paparazzi.
# 
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
# 
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA. 
##

CC = gcc
CFLAGS = -std=gnu99 -Wall -I.. -I../.. -I../../test/ -I../../../include -I../../booz_priv
LDFLAGS = -lm

CFLAGS += -I../../../../var/BOOZ2_A1P
test_mlkf: test_mlkf.c ../booz_ahrs.c ../../booz_priv/ahrs/booz_ahrs_mlkf.c ../../booz_priv/ahrs/booz_ahrs_mlkf_opt.c  ../booz_imu.c ../../math/pprz_trig_int.c ./imu_dummy.c ../ahrs/booz_ahrs_aligner.c
	$(CC) $(CFLAGS) -DBOOZ_IMU_TYPE_H=\"test/imu_dummy.h\" -o $@ $^ $(LDFLAGS)

test_vg_ref: test_vg_ref.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_vg_adpt: test_vg_adpt.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)


test_deadband: test_deadband.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_scaling: test_scaling.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

TEST_ATT_CFLAGS =  -DSTABILISATION_ATTITUDE_TYPE_INT \
	           -DSTABILISATION_ATTITUDE_H=\"stabilization/booz_stabilization_attitude_int.h\" \
	           -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/booz_stabilization_attitude_ref_quat_int.h\"

test_att_ref: test_att_ref.c ../stabilization/booz_stabilization_attitude_ref_quat_int.c
	$(CC) $(CFLAGS) $(TEST_ATT_CFLAGS) -I/home/poine/work/savannah/paparazzi3/trunk/var/BOOZ2_A1 -o $@ $^ $(LDFLAGS)


clean:
	rm -f *~ test_att_ref
