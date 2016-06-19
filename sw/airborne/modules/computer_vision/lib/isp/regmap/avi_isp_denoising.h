/*********************************************************************
 * avi_isp_denoising register map
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

#ifndef _AVI_ISP_DENOISING_H_
#define _AVI_ISP_DENOISING_H_

#define AVI_ISP_DENOISING_CFA                     0x00
#define AVI_ISP_DENOISING_LUMOCOEFF_R_03_00       0x10
#define AVI_ISP_DENOISING_LUMOCOEFF_R_07_04       0x14
#define AVI_ISP_DENOISING_LUMOCOEFF_R_11_08       0x18
#define AVI_ISP_DENOISING_LUMOCOEFF_R_13_12       0x1c
#define AVI_ISP_DENOISING_LUMOCOEFF_B_03_00       0x20
#define AVI_ISP_DENOISING_LUMOCOEFF_B_07_04       0x24
#define AVI_ISP_DENOISING_LUMOCOEFF_B_11_08       0x28
#define AVI_ISP_DENOISING_LUMOCOEFF_B_13_12       0x2c
#define AVI_ISP_DENOISING_LUMOCOEFF_G_03_00       0x30
#define AVI_ISP_DENOISING_LUMOCOEFF_G_07_04       0x34
#define AVI_ISP_DENOISING_LUMOCOEFF_G_11_08       0x38
#define AVI_ISP_DENOISING_LUMOCOEFF_G_13_12       0x3c

union avi_isp_denoising_cfa
{
	struct
	{
		uint32_t cfa             :  2;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_r_03_00
{
	struct
	{
		uint32_t lumocoeff_r_00  :  8;
		uint32_t lumocoeff_r_01  :  8;
		uint32_t lumocoeff_r_02  :  8;
		uint32_t lumocoeff_r_03  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_r_07_04
{
	struct
	{
		uint32_t lumocoeff_r_04  :  8;
		uint32_t lumocoeff_r_05  :  8;
		uint32_t lumocoeff_r_06  :  8;
		uint32_t lumocoeff_r_07  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_r_11_08
{
	struct
	{
		uint32_t lumocoeff_r_08  :  8;
		uint32_t lumocoeff_r_09  :  8;
		uint32_t lumocoeff_r_10  :  8;
		uint32_t lumocoeff_r_11  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_r_13_12
{
	struct
	{
		uint32_t lumocoeff_r_12  :  8;
		uint32_t lumocoeff_r_13  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_b_03_00
{
	struct
	{
		uint32_t lumocoeff_b_00  :  8;
		uint32_t lumocoeff_b_01  :  8;
		uint32_t lumocoeff_b_02  :  8;
		uint32_t lumocoeff_b_03  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_b_07_04
{
	struct
	{
		uint32_t lumocoeff_b_04  :  8;
		uint32_t lumocoeff_b_05  :  8;
		uint32_t lumocoeff_b_06  :  8;
		uint32_t lumocoeff_b_07  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_b_11_08
{
	struct
	{
		uint32_t lumocoeff_b_08  :  8;
		uint32_t lumocoeff_b_09  :  8;
		uint32_t lumocoeff_b_10  :  8;
		uint32_t lumocoeff_b_11  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_b_13_12
{
	struct
	{
		uint32_t lumocoeff_b_12  :  8;
		uint32_t lumocoeff_b_13  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_g_03_00
{
	struct
	{
		uint32_t lumocoeff_g_00  :  8;
		uint32_t lumocoeff_g_01  :  8;
		uint32_t lumocoeff_g_02  :  8;
		uint32_t lumocoeff_g_03  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_g_07_04
{
	struct
	{
		uint32_t lumocoeff_g_04  :  8;
		uint32_t lumocoeff_g_05  :  8;
		uint32_t lumocoeff_g_06  :  8;
		uint32_t lumocoeff_g_07  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_g_11_08
{
	struct
	{
		uint32_t lumocoeff_g_08  :  8;
		uint32_t lumocoeff_g_09  :  8;
		uint32_t lumocoeff_g_10  :  8;
		uint32_t lumocoeff_g_11  :  8;
	};
	uint32_t _register;
};

union avi_isp_denoising_lumocoeff_g_13_12
{
	struct
	{
		uint32_t lumocoeff_g_12  :  8;
		uint32_t lumocoeff_g_13  :  8;
	};
	uint32_t _register;
};

struct avi_isp_denoising_regs
{
	union avi_isp_denoising_cfa              cfa;                        /* 0x000 */
	unsigned                                 /*unused*/ : 32;            /* 0x004 */
	unsigned                                 /*unused*/ : 32;            /* 0x008 */
	unsigned                                 /*unused*/ : 32;            /* 0x00c */
	union avi_isp_denoising_lumocoeff_r_03_00 lumocoeff_r_03_00;          /* 0x010 */
	union avi_isp_denoising_lumocoeff_r_07_04 lumocoeff_r_07_04;          /* 0x014 */
	union avi_isp_denoising_lumocoeff_r_11_08 lumocoeff_r_11_08;          /* 0x018 */
	union avi_isp_denoising_lumocoeff_r_13_12 lumocoeff_r_13_12;          /* 0x01c */
	union avi_isp_denoising_lumocoeff_b_03_00 lumocoeff_b_03_00;          /* 0x020 */
	union avi_isp_denoising_lumocoeff_b_07_04 lumocoeff_b_07_04;          /* 0x024 */
	union avi_isp_denoising_lumocoeff_b_11_08 lumocoeff_b_11_08;          /* 0x028 */
	union avi_isp_denoising_lumocoeff_b_13_12 lumocoeff_b_13_12;          /* 0x02c */
	union avi_isp_denoising_lumocoeff_g_03_00 lumocoeff_g_03_00;          /* 0x030 */
	union avi_isp_denoising_lumocoeff_g_07_04 lumocoeff_g_07_04;          /* 0x034 */
	union avi_isp_denoising_lumocoeff_g_11_08 lumocoeff_g_11_08;          /* 0x038 */
	union avi_isp_denoising_lumocoeff_g_13_12 lumocoeff_g_13_12;          /* 0x03c */
};

#endif /* _AVI_ISP_DENOISING_H_ */
