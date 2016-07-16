/*********************************************************************
 * avi_isp_pedestal register map
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

#ifndef _AVI_ISP_PEDESTAL_H_
#define _AVI_ISP_PEDESTAL_H_

#define AVI_ISP_PEDESTAL_CFA                      0x00
#define AVI_ISP_PEDESTAL_SUB_R                    0x04
#define AVI_ISP_PEDESTAL_SUB_GB                   0x08
#define AVI_ISP_PEDESTAL_SUB_GR                   0x0c
#define AVI_ISP_PEDESTAL_SUB_B                    0x10

union avi_isp_pedestal_cfa
{
	struct
	{
		uint32_t cfa             :  2;
	};
	uint32_t _register;
};

union avi_isp_pedestal_sub_r
{
	struct
	{
		uint32_t sub_r           : 10;
	};
	uint32_t _register;
};

union avi_isp_pedestal_sub_gb
{
	struct
	{
		uint32_t sub_gb          : 10;
	};
	uint32_t _register;
};

union avi_isp_pedestal_sub_gr
{
	struct
	{
		uint32_t sub_gr          : 10;
	};
	uint32_t _register;
};

union avi_isp_pedestal_sub_b
{
	struct
	{
		uint32_t sub_b           : 10;
	};
	uint32_t _register;
};

struct avi_isp_pedestal_regs
{
	union avi_isp_pedestal_cfa               cfa;                        /* 0x000 */
	union avi_isp_pedestal_sub_r             sub_r;                      /* 0x004 */
	union avi_isp_pedestal_sub_gb            sub_gb;                     /* 0x008 */
	union avi_isp_pedestal_sub_gr            sub_gr;                     /* 0x00c */
	union avi_isp_pedestal_sub_b             sub_b;                      /* 0x010 */
};

#endif /* _AVI_ISP_PEDESTAL_H_ */
