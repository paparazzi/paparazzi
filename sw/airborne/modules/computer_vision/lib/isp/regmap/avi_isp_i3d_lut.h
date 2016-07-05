/*********************************************************************
 * avi_isp_i3d_lut register map
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

#ifndef _AVI_ISP_I3D_LUT_H_
#define _AVI_ISP_I3D_LUT_H_

#define AVI_ISP_I3D_LUT_LUT_OUTSIDE               0x00

union avi_isp_i3d_lut_lut_outside
{
	struct
	{
		uint32_t bv_value        :  8;
		uint32_t gu_value        :  8;
		uint32_t ry_value        :  8;
	};
	uint32_t _register;
};

struct avi_isp_i3d_lut_lut_outside_regs
{
	union avi_isp_i3d_lut_lut_outside lut_outside[125];
};

#define AVI_ISP_I3D_LUT_LUT_INSIDE                0x200

union avi_isp_i3d_lut_lut_inside
{
	struct
	{
		uint32_t bv_value        :  8;
		uint32_t gu_value        :  8;
		uint32_t ry_value        :  8;
	};
	uint32_t _register;
};

struct avi_isp_i3d_lut_lut_inside_regs
{
	union avi_isp_i3d_lut_lut_inside lut_inside[125];
};

#define AVI_ISP_I3D_LUT_CLIP_MODE                 0x400

union avi_isp_i3d_lut_clip_mode
{
	struct
	{
		uint32_t clip_en         :  1;
	};
	uint32_t _register;
};

struct avi_isp_i3d_lut_regs
{
	union avi_isp_i3d_lut_clip_mode          clip_mode;                  /* 0x400 */
};

#endif /* _AVI_ISP_I3D_LUT_H_ */
