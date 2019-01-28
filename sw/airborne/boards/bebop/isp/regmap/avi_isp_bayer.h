/*********************************************************************
 * avi_isp_bayer register map
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

#ifndef _AVI_ISP_BAYER_H_
#define _AVI_ISP_BAYER_H_

#define AVI_ISP_BAYER_CFA                         0x00
#define AVI_ISP_BAYER_THRESHOLD_1                 0x04
#define AVI_ISP_BAYER_THRESHOLD_2                 0x08

union avi_isp_bayer_cfa
{
	struct
	{
		uint32_t cfa             :  2;
	};
	uint32_t _register;
};

union avi_isp_bayer_threshold_1
{
	struct
	{
		uint32_t threshold_1     : 13;
	};
	uint32_t _register;
};

union avi_isp_bayer_threshold_2
{
	struct
	{
		uint32_t threshold_2     : 13;
	};
	uint32_t _register;
};

struct avi_isp_bayer_regs
{
	union avi_isp_bayer_cfa                  cfa;                        /* 0x000 */
	union avi_isp_bayer_threshold_1          threshold_1;                /* 0x004 */
	union avi_isp_bayer_threshold_2          threshold_2;                /* 0x008 */
};

#endif /* _AVI_ISP_BAYER_H_ */
