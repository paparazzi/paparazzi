/*********************************************************************
 * avi_isp_drop register map
 *
 * Vendor:   Parrot
 * Library:  AVI
 * Version:  P7R3
 * Gen-date: (Date of generation of this C code, not the IP-Xact file)
 *           2014-02-28
 *
 * WARNING: This code is automatically generated from the hardware
 * IP-Xact XML files. Do not edit directly.
 *********************************************************************/

#ifndef _AVI_ISP_DROP_H_
#define _AVI_ISP_DROP_H_

#define AVI_ISP_DROP_OFFSET_X                     0x00
#define AVI_ISP_DROP_OFFSET_Y                     0x04

union avi_isp_drop_offset_x
{
	struct
	{
		uint32_t offset_x        : 16;
	};
	uint32_t _register;
};

union avi_isp_drop_offset_y
{
	struct
	{
		uint32_t offset_y        : 16;
	};
	uint32_t _register;
};

struct avi_isp_drop_regs
{
	union avi_isp_drop_offset_x              offset_x;                   /* 0x000 */
	union avi_isp_drop_offset_y              offset_y;                   /* 0x004 */
};

#endif /* _AVI_ISP_DROP_H_ */
