#ifndef _LIBISP_H
#define _LIBISP_H

#include "reg_avi.h"
#include "modules/computer_vision/lib/v4l/v4l2.h"

#define AVI_DEFINE_NODE(EXPANDER) \
	EXPANDER(chain_bayer_inter)					\
	EXPANDER(vlformat_32to40)					\
	EXPANDER(pedestal)						\
	EXPANDER(green_imbalance)					\
	EXPANDER(green_imbalance_green_red_coeff_mem)			\
	EXPANDER(green_imbalance_green_blue_coeff_mem)			\
	EXPANDER(dead_pixel_correction)					\
	EXPANDER(dead_pixel_correction_list_mem)			\
	EXPANDER(denoising)						\
	EXPANDER(statistics_bayer)					\
	EXPANDER(lens_shading_correction)				\
	EXPANDER(lens_shading_correction_red_coeff_mem)			\
	EXPANDER(lens_shading_correction_green_coeff_mem)		\
	EXPANDER(lens_shading_correction_blue_coeff_mem)		\
	EXPANDER(chromatic_aberration)					\
	EXPANDER(bayer)							\
	EXPANDER(color_correction)					\
	EXPANDER(vlformat_40to32)					\
	EXPANDER(gamma_corrector)					\
	EXPANDER(gamma_corrector_ry_lut)				\
	EXPANDER(gamma_corrector_gu_lut)				\
	EXPANDER(gamma_corrector_bv_lut)				\
	EXPANDER(chroma)						\
	EXPANDER(statistics_yuv)					\
	EXPANDER(statistics_yuv_ae_histogram_y)				\
	EXPANDER(chain_yuv_inter)					\
	EXPANDER(edge_enhancement_color_reduction_filter)		\
	EXPANDER(edge_enhancement_color_reduction_filter_ee_lut)	\
	EXPANDER(i3d_lut)						\
	EXPANDER(i3d_lut_lut_outside)					\
	EXPANDER(i3d_lut_lut_inside)					\
	EXPANDER(drop)

#define EXPAND_AS_ENUM(_node) \
	_node,

enum {
	AVI_DEFINE_NODE(EXPAND_AS_ENUM)
	ISP_NODE_NR,
};

/* ISP Context */
struct libisp_context
{
	int           devmem;
	unsigned long avi_base;
	unsigned      offsets[ISP_NODE_NR];
};

int configure_isp(struct v4l2_device *dev);

/* Registers access */
#define EXPAND_AS_PROTOTYPE(_node)                                        \
	void avi_isp_ ## _node ## _set_registers(struct libisp_context *, \
			struct avi_isp_ ## _node ## _regs const *regs);   \
	void avi_isp_ ## _node ## _get_registers(struct libisp_context *, \
			struct avi_isp_ ## _node ## _regs *regs);

AVI_DEFINE_NODE(EXPAND_AS_PROTOTYPE)

#endif /* _LIBISP_H */
