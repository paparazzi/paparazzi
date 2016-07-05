/*********************************************************************
 * avi_isp_chromatic_aberration register map
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

#ifndef _AVI_ISP_CHROMATIC_ABERRATION_H_
#define _AVI_ISP_CHROMATIC_ABERRATION_H_

#define AVI_ISP_CHROMATIC_ABERRATION_RADIUS_SQUARED  0x00
#define AVI_ISP_CHROMATIC_ABERRATION_DISPLACEMENT_COEFFS  0x04
#define AVI_ISP_CHROMATIC_ABERRATION_CIRCLE_POS_X_CENTER  0xa0
#define AVI_ISP_CHROMATIC_ABERRATION_CIRCLE_POS_X_SQUARED  0xa4
#define AVI_ISP_CHROMATIC_ABERRATION_CIRCLE_POS_Y_CENTER  0xa8
#define AVI_ISP_CHROMATIC_ABERRATION_CIRCLE_POS_Y_SQUARED  0xac
#define AVI_ISP_CHROMATIC_ABERRATION_CFA          0xb0
#define AVI_ISP_CHROMATIC_ABERRATION_GREEN_VARIATION  0xb4
#define AVI_ISP_CHROMATIC_ABERRATION_INCREMENTS_LOG2  0xb8

union avi_isp_chromatic_aberration_radius_squared
{
	struct
	{
		uint32_t radius_squared  : 24;
	};
	uint32_t _register;
};

union avi_isp_chromatic_aberration_displacement_coeffs
{
	struct
	{
		uint32_t displacement_blue : 16;
		uint32_t displacement_red : 16;
	};
	uint32_t _register;
};

union avi_isp_chromatic_aberration_circle_pos_x_center
{
	struct
	{
		uint32_t x_center        : 14;
	};
	uint32_t _register;
};

union avi_isp_chromatic_aberration_circle_pos_x_squared
{
	struct
	{
		uint32_t x_squared       : 26;
	};
	uint32_t _register;
};

union avi_isp_chromatic_aberration_circle_pos_y_center
{
	struct
	{
		uint32_t y_center        : 14;
	};
	uint32_t _register;
};

union avi_isp_chromatic_aberration_circle_pos_y_squared
{
	struct
	{
		uint32_t y_squared       : 26;
	};
	uint32_t _register;
};

union avi_isp_chromatic_aberration_cfa
{
	struct
	{
		uint32_t cfa             :  2;
	};
	uint32_t _register;
};

union avi_isp_chromatic_aberration_green_variation
{
	struct
	{
		uint32_t green_var       :  1;
	};
	uint32_t _register;
};

union avi_isp_chromatic_aberration_increments_log2
{
	struct
	{
		uint32_t x_log2_inc      :  3;
		unsigned /*unused */     : 13;
		uint32_t y_log2_inc      :  3;
	};
	uint32_t _register;
};

struct avi_isp_chromatic_aberration_regs
{
	union avi_isp_chromatic_aberration_radius_squared radius_squared;             /* 0x000 */
	union avi_isp_chromatic_aberration_displacement_coeffs displacement_coeffs;        /* 0x004 */
	unsigned                                 /*unused*/ : 32;            /* 0x008 */
	unsigned                                 /*unused*/ : 32;            /* 0x00c */
	unsigned                                 /*unused*/ : 32;            /* 0x010 */
	unsigned                                 /*unused*/ : 32;            /* 0x014 */
	unsigned                                 /*unused*/ : 32;            /* 0x018 */
	unsigned                                 /*unused*/ : 32;            /* 0x01c */
	unsigned                                 /*unused*/ : 32;            /* 0x020 */
	unsigned                                 /*unused*/ : 32;            /* 0x024 */
	unsigned                                 /*unused*/ : 32;            /* 0x028 */
	unsigned                                 /*unused*/ : 32;            /* 0x02c */
	unsigned                                 /*unused*/ : 32;            /* 0x030 */
	unsigned                                 /*unused*/ : 32;            /* 0x034 */
	unsigned                                 /*unused*/ : 32;            /* 0x038 */
	unsigned                                 /*unused*/ : 32;            /* 0x03c */
	unsigned                                 /*unused*/ : 32;            /* 0x040 */
	unsigned                                 /*unused*/ : 32;            /* 0x044 */
	unsigned                                 /*unused*/ : 32;            /* 0x048 */
	unsigned                                 /*unused*/ : 32;            /* 0x04c */
	unsigned                                 /*unused*/ : 32;            /* 0x050 */
	unsigned                                 /*unused*/ : 32;            /* 0x054 */
	unsigned                                 /*unused*/ : 32;            /* 0x058 */
	unsigned                                 /*unused*/ : 32;            /* 0x05c */
	unsigned                                 /*unused*/ : 32;            /* 0x060 */
	unsigned                                 /*unused*/ : 32;            /* 0x064 */
	unsigned                                 /*unused*/ : 32;            /* 0x068 */
	unsigned                                 /*unused*/ : 32;            /* 0x06c */
	unsigned                                 /*unused*/ : 32;            /* 0x070 */
	unsigned                                 /*unused*/ : 32;            /* 0x074 */
	unsigned                                 /*unused*/ : 32;            /* 0x078 */
	unsigned                                 /*unused*/ : 32;            /* 0x07c */
	unsigned                                 /*unused*/ : 32;            /* 0x080 */
	unsigned                                 /*unused*/ : 32;            /* 0x084 */
	unsigned                                 /*unused*/ : 32;            /* 0x088 */
	unsigned                                 /*unused*/ : 32;            /* 0x08c */
	unsigned                                 /*unused*/ : 32;            /* 0x090 */
	unsigned                                 /*unused*/ : 32;            /* 0x094 */
	unsigned                                 /*unused*/ : 32;            /* 0x098 */
	unsigned                                 /*unused*/ : 32;            /* 0x09c */
	union avi_isp_chromatic_aberration_circle_pos_x_center circle_pos_x_center;        /* 0x0a0 */
	union avi_isp_chromatic_aberration_circle_pos_x_squared circle_pos_x_squared;       /* 0x0a4 */
	union avi_isp_chromatic_aberration_circle_pos_y_center circle_pos_y_center;        /* 0x0a8 */
	union avi_isp_chromatic_aberration_circle_pos_y_squared circle_pos_y_squared;       /* 0x0ac */
	union avi_isp_chromatic_aberration_cfa   cfa;                        /* 0x0b0 */
	union avi_isp_chromatic_aberration_green_variation green_variation;            /* 0x0b4 */
	union avi_isp_chromatic_aberration_increments_log2 increments_log2;            /* 0x0b8 */
};

#endif /* _AVI_ISP_CHROMATIC_ABERRATION_H_ */
