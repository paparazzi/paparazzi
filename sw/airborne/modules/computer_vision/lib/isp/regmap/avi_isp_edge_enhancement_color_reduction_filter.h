/*********************************************************************
 * avi_isp_edge_enhancement_color_reduction_filter register map
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

#ifndef _AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_H_
#define _AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_H_

#define AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_EE_LUT  0x00

union avi_isp_edge_enhancement_color_reduction_filter_ee_lut
{
	struct
	{
		uint32_t ee_lut          :  6;
	};
	uint32_t _register;
};

struct avi_isp_edge_enhancement_color_reduction_filter_ee_lut_regs
{
	union avi_isp_edge_enhancement_color_reduction_filter_ee_lut ee_lut[256];
};

#define AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_EE_KERNEL_COEFF  0x400
#define AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_CRF_KERNEL_COEFF  0x420
#define AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_M_COEFF  0x440
#define AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_D_COEFF  0x460

union avi_isp_edge_enhancement_color_reduction_filter_ee_kernel_coeff
{
	struct
	{
		uint32_t ee_kernel_coeff : 11;
	};
	uint32_t _register;
};

union avi_isp_edge_enhancement_color_reduction_filter_crf_kernel_coeff
{
	struct
	{
		uint32_t crf_kernel_coeff : 11;
	};
	uint32_t _register;
};

union avi_isp_edge_enhancement_color_reduction_filter_m_coeff
{
	struct
	{
		uint32_t m_coeff         :  8;
	};
	uint32_t _register;
};

union avi_isp_edge_enhancement_color_reduction_filter_d_coeff
{
	struct
	{
		uint32_t d_coeff         : 11;
	};
	uint32_t _register;
};

struct avi_isp_edge_enhancement_color_reduction_filter_regs
{
	union avi_isp_edge_enhancement_color_reduction_filter_ee_kernel_coeff ee_kernel_coeff[6];         /* 0x400 - 0x414 */
	unsigned                                 /*unused*/ : 32;            /* 0x018 */
	unsigned                                 /*unused*/ : 32;            /* 0x01c */
	union avi_isp_edge_enhancement_color_reduction_filter_crf_kernel_coeff crf_kernel_coeff[6];        /* 0x420 - 0x434 */
	unsigned                                 /*unused*/ : 32;            /* 0x038 */
	unsigned                                 /*unused*/ : 32;            /* 0x03c */
	union avi_isp_edge_enhancement_color_reduction_filter_m_coeff m_coeff;                    /* 0x440 */
	unsigned                                 /*unused*/ : 32;            /* 0x044 */
	unsigned                                 /*unused*/ : 32;            /* 0x048 */
	unsigned                                 /*unused*/ : 32;            /* 0x04c */
	unsigned                                 /*unused*/ : 32;            /* 0x050 */
	unsigned                                 /*unused*/ : 32;            /* 0x054 */
	unsigned                                 /*unused*/ : 32;            /* 0x058 */
	unsigned                                 /*unused*/ : 32;            /* 0x05c */
	union avi_isp_edge_enhancement_color_reduction_filter_d_coeff d_coeff;                    /* 0x460 */
};

#endif /* _AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_H_ */
