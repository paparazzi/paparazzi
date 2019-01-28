#ifndef _REG_AVI_H
#define _REG_AVI_H

#include <stdint.h>

/* Chain Bayer */
#include "regmap/avi_isp_vlformat_32to40.h"
#include "regmap/avi_isp_chain_bayer_inter.h"
#include "regmap/avi_isp_pedestal.h"
#include "regmap/avi_isp_green_imbalance.h"
#include "regmap/avi_isp_dead_pixel_correction.h"
#include "regmap/avi_isp_statistics_bayer.h"
#include "regmap/avi_isp_denoising.h"
#include "regmap/avi_isp_lens_shading_correction.h"
#include "regmap/avi_isp_chromatic_aberration.h"
#include "regmap/avi_isp_bayer.h"
#include "regmap/avi_isp_color_correction.h"
#include "regmap/avi_isp_vlformat_40to32.h"

/* Gamma + CSC */
#include "regmap/avi_isp_gamma_corrector.h"
#include "regmap/avi_isp_chroma.h"
#include "regmap/avi_isp_statistics_yuv.h"

/* Chain YUV */
#include "regmap/avi_isp_chain_yuv_inter.h"
#include "regmap/avi_isp_edge_enhancement_color_reduction_filter.h"
#include "regmap/avi_isp_i3d_lut.h"
#include "regmap/avi_isp_drop.h"

/* Sub modules of chain Bayer */
#define AVI_ISP_CHAIN_BAYER_INTER                       0x0000
#define AVI_ISP_VLFORMAT_32TO40                         0x1000
#define AVI_ISP_PEDESTAL                                0x2000
#define AVI_ISP_GREEN_IMBALANCE                         0x4000
#define AVI_ISP_DEAD_PIXEL_CORRECTION                   0x6000
#define AVI_ISP_DENOISING                               0x7000
#define AVI_ISP_STATISTICS_BAYER                        0x8000
#define AVI_ISP_LENS_SHADING_CORRECTION                 0xA000
#define AVI_ISP_CHROMATIC_ABERRATION                    0xC000
#define AVI_ISP_BAYER                                   0xD000
#define AVI_ISP_COLOR_CORRECTION                        0xE000
#define AVI_ISP_VLFORMAT_40TO32                         0xF000

/* Sub modules of chain YUV */
#define AVI_ISP_CHAIN_YUV_INTER                         0x0000
#define AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER 0x1000
#define AVI_ISP_I3D_LUT                                 0x2000
#define AVI_ISP_DROP                                    0x3000

#endif /* _REG_AVI_H */
