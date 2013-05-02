/*
 * electrical_arch.c
 *
 *  Created on: Nov 23, 2012
 *      Author: dhensen
 */


#include "subsystems/electrical/electrical_arch.h"
#include "vbat.h"

struct Electrical electrical;
vbat_struct vbat;

void electrical_init(void) {
//	int status;
//	status = vbat_init(&vbat);
//	printf("Status returned on vbat_init: %d", status);
}

void electrical_periodic(void) {
//	vbat_read(&vbat);
	electrical.vsupply = 120;
}
