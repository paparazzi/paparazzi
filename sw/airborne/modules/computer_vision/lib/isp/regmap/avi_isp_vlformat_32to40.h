/*********************************************************************
 * avi_isp_vlformat_32to40 register map
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

#ifndef _AVI_ISP_VLFORMAT_32TO40_H_
#define _AVI_ISP_VLFORMAT_32TO40_H_

#define AVI_ISP_VLFORMAT_32TO40_FORMAT                0x00

union avi_isp_vlformat_32to40_format
{
	struct
	{
		uint32_t format          :  3;
	};
	uint32_t _register;
};

struct avi_isp_vlformat_32to40_regs
{
	union avi_isp_vlformat_32to40_format         format;                     /* 0x000 */
};

#endif /* _AVI_ISP_VLFORMAT_32TO40_H_ */
