/*********************************************************************
 * avi_isp_chroma register map
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

#ifndef _AVI_ISP_CHROMA_H_
#define _AVI_ISP_CHROMA_H_

#define AVI_ISP_CHROMA_COEFF_01_00                0x00
#define AVI_ISP_CHROMA_COEFF_10_02                0x04
#define AVI_ISP_CHROMA_COEFF_12_11                0x08
#define AVI_ISP_CHROMA_COEFF_21_20                0x0c
#define AVI_ISP_CHROMA_COEFF_22                   0x10
#define AVI_ISP_CHROMA_OFFSET_RY                  0x14
#define AVI_ISP_CHROMA_CLIP_RY                    0x18
#define AVI_ISP_CHROMA_OFFSET_GU                  0x1c
#define AVI_ISP_CHROMA_CLIP_GU                    0x20
#define AVI_ISP_CHROMA_OFFSET_BV                  0x24
#define AVI_ISP_CHROMA_CLIP_BV                    0x28

union avi_isp_chroma_coeff_01_00
{
	struct
	{
		uint32_t coeff_00        : 14;
		unsigned /*unused */     :  2;
		uint32_t coeff_01        : 14;
	};
	uint32_t _register;
};

union avi_isp_chroma_coeff_10_02
{
	struct
	{
		uint32_t coeff_02        : 14;
		unsigned /*unused */     :  2;
		uint32_t coeff_10        : 14;
	};
	uint32_t _register;
};

union avi_isp_chroma_coeff_12_11
{
	struct
	{
		uint32_t coeff_11        : 14;
		unsigned /*unused */     :  2;
		uint32_t coeff_12        : 14;
	};
	uint32_t _register;
};

union avi_isp_chroma_coeff_21_20
{
	struct
	{
		uint32_t coeff_20        : 14;
		unsigned /*unused */     :  2;
		uint32_t coeff_21        : 14;
	};
	uint32_t _register;
};

union avi_isp_chroma_coeff_22
{
	struct
	{
		uint32_t coeff_22        : 14;
	};
	uint32_t _register;
};

union avi_isp_chroma_offset_ry
{
	struct
	{
		uint32_t offset_in       : 10;
		unsigned /*unused */     :  6;
		uint32_t offset_out      : 10;
	};
	uint32_t _register;
};

union avi_isp_chroma_clip_ry
{
	struct
	{
		uint32_t clip_min        : 10;
		unsigned /*unused */     :  6;
		uint32_t clip_max        : 10;
	};
	uint32_t _register;
};

union avi_isp_chroma_offset_gu
{
	struct
	{
		uint32_t offset_in       : 10;
		unsigned /*unused */     :  6;
		uint32_t offset_out      : 10;
	};
	uint32_t _register;
};

union avi_isp_chroma_clip_gu
{
	struct
	{
		uint32_t clip_min        : 10;
		unsigned /*unused */     :  6;
		uint32_t clip_max        : 10;
	};
	uint32_t _register;
};

union avi_isp_chroma_offset_bv
{
	struct
	{
		uint32_t offset_in       : 10;
		unsigned /*unused */     :  6;
		uint32_t offset_out      : 10;
	};
	uint32_t _register;
};

union avi_isp_chroma_clip_bv
{
	struct
	{
		uint32_t clip_min        : 10;
		unsigned /*unused */     :  6;
		uint32_t clip_max        : 10;
	};
	uint32_t _register;
};

struct avi_isp_chroma_regs
{
	union avi_isp_chroma_coeff_01_00         coeff_01_00;                /* 0x000 */
	union avi_isp_chroma_coeff_10_02         coeff_10_02;                /* 0x004 */
	union avi_isp_chroma_coeff_12_11         coeff_12_11;                /* 0x008 */
	union avi_isp_chroma_coeff_21_20         coeff_21_20;                /* 0x00c */
	union avi_isp_chroma_coeff_22            coeff_22;                   /* 0x010 */
	union avi_isp_chroma_offset_ry           offset_ry;                  /* 0x014 */
	union avi_isp_chroma_clip_ry             clip_ry;                    /* 0x018 */
	union avi_isp_chroma_offset_gu           offset_gu;                  /* 0x01c */
	union avi_isp_chroma_clip_gu             clip_gu;                    /* 0x020 */
	union avi_isp_chroma_offset_bv           offset_bv;                  /* 0x024 */
	union avi_isp_chroma_clip_bv             clip_bv;                    /* 0x028 */
};

#endif /* _AVI_ISP_CHROMA_H_ */
