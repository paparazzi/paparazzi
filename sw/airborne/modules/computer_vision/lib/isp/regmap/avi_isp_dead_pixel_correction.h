/*********************************************************************
 * avi_isp_dead_pixel_correction register map
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

#ifndef _AVI_ISP_DEAD_PIXEL_CORRECTION_H_
#define _AVI_ISP_DEAD_PIXEL_CORRECTION_H_

#define AVI_ISP_DEAD_PIXEL_CORRECTION_LIST_MEM  0x00

union avi_isp_dead_pixel_correction_list_mem
{
	struct
	{
		uint32_t op_code         :  3;
		uint32_t coord_x         : 11;
		uint32_t coord_y         : 12;
	};
	uint32_t _register;
};

struct avi_isp_dead_pixel_correction_list_mem_regs
{
	union avi_isp_dead_pixel_correction_list_mem list_mem[256];
};

#define AVI_ISP_DEAD_PIXEL_CORRECTION_CFA  0x400
#define AVI_ISP_DEAD_PIXEL_CORRECTION_BYPASS  0x404
#define AVI_ISP_DEAD_PIXEL_CORRECTION_THRESHOLD  0x408
#define AVI_ISP_DEAD_PIXEL_CORRECTION_RGRIM_CONF  0x40c
#define AVI_ISP_DEAD_PIXEL_CORRECTION_RGRIM_GAIN  0x410

union avi_isp_dead_pixel_correction_cfa
{
	struct
	{
		uint32_t cfa             :  2;
	};
	uint32_t _register;
};

union avi_isp_dead_pixel_correction_bypass
{
	struct
	{
		uint32_t list            :  1;
		uint32_t auto_detection  :  1;
		uint32_t rgrim           :  1;
	};
	uint32_t _register;
};

union avi_isp_dead_pixel_correction_threshold
{
	struct
	{
		uint32_t threshold       : 10;
	};
	uint32_t _register;
};

union avi_isp_dead_pixel_correction_rgrim_conf
{
	struct
	{
		uint32_t tim_1           :  8;
		uint32_t tim_2           :  8;
		uint32_t tsat            :  8;
		uint32_t tcon            :  8;
	};
	uint32_t _register;
};

union avi_isp_dead_pixel_correction_rgrim_gain
{
	struct
	{
		uint32_t gain            :  7;
	};
	uint32_t _register;
};

struct avi_isp_dead_pixel_correction_regs
{
	union avi_isp_dead_pixel_correction_cfa cfa;                        /* 0x400 */
	union avi_isp_dead_pixel_correction_bypass bypass;                     /* 0x404 */
	union avi_isp_dead_pixel_correction_threshold threshold;                  /* 0x408 */
	union avi_isp_dead_pixel_correction_rgrim_conf rgrim_conf;                 /* 0x40c */
	union avi_isp_dead_pixel_correction_rgrim_gain rgrim_gain;                 /* 0x410 */
};

#endif /* _AVI_ISP_DEAD_PIXEL_CORRECTION_H_ */
