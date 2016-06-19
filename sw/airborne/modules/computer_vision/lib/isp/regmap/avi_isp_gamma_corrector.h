/*********************************************************************
 * avi_isp_gamma_corrector register map
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

#ifndef _AVI_ISP_GAMMA_CORRECTOR_H_
#define _AVI_ISP_GAMMA_CORRECTOR_H_

#define AVI_ISP_GAMMA_CORRECTOR_CONF              0x00

union avi_isp_gamma_corrector_conf
{
	struct
	{
		uint32_t bypass          :  1;
		uint32_t palette         :  1;
		uint32_t comp_width      :  1;
	};
	uint32_t _register;
};

struct avi_isp_gamma_corrector_regs
{
	union avi_isp_gamma_corrector_conf       conf;                       /* 0x000 */
};

#define AVI_ISP_GAMMA_CORRECTOR_RY_LUT            0x1000

union avi_isp_gamma_corrector_ry_lut
{
	struct
	{
		uint32_t ry_value        :  8;
	};
	uint32_t _register;
};

struct avi_isp_gamma_corrector_ry_lut_regs
{
	union avi_isp_gamma_corrector_ry_lut ry_lut[1024];
};

#define AVI_ISP_GAMMA_CORRECTOR_GU_LUT            0x2000

union avi_isp_gamma_corrector_gu_lut
{
	struct
	{
		uint32_t gu_value        :  8;
	};
	uint32_t _register;
};

struct avi_isp_gamma_corrector_gu_lut_regs
{
	union avi_isp_gamma_corrector_gu_lut gu_lut[1024];
};

#define AVI_ISP_GAMMA_CORRECTOR_BV_LUT            0x3000

union avi_isp_gamma_corrector_bv_lut
{
	struct
	{
		uint32_t bv_value        :  8;
	};
	uint32_t _register;
};

struct avi_isp_gamma_corrector_bv_lut_regs
{
	union avi_isp_gamma_corrector_bv_lut bv_lut[1024];
};

#endif /* _AVI_ISP_GAMMA_CORRECTOR_H_ */
