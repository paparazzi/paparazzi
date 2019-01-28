#ifndef _LIBISP_H
#define _LIBISP_H

#include "reg_avi.h"
#include "modules/computer_vision/lib/v4l/v4l2.h"

#define AVI_DEFINE_NODE(EXPANDER) \
  EXPANDER(chain_bayer_inter)         \
  EXPANDER(vlformat_32to40)         \
  EXPANDER(pedestal)            \
  EXPANDER(green_imbalance)         \
  EXPANDER(green_imbalance_green_red_coeff_mem)     \
  EXPANDER(green_imbalance_green_blue_coeff_mem)      \
  EXPANDER(dead_pixel_correction)         \
  EXPANDER(dead_pixel_correction_list_mem)      \
  EXPANDER(denoising)           \
  EXPANDER(statistics_bayer)          \
  EXPANDER(lens_shading_correction)       \
  EXPANDER(lens_shading_correction_red_coeff_mem)     \
  EXPANDER(lens_shading_correction_green_coeff_mem)   \
  EXPANDER(lens_shading_correction_blue_coeff_mem)    \
  EXPANDER(chromatic_aberration)          \
  EXPANDER(bayer)             \
  EXPANDER(color_correction)          \
  EXPANDER(vlformat_40to32)         \
  EXPANDER(gamma_corrector)         \
  EXPANDER(gamma_corrector_ry_lut)        \
  EXPANDER(gamma_corrector_gu_lut)        \
  EXPANDER(gamma_corrector_bv_lut)        \
  EXPANDER(chroma)            \
  EXPANDER(statistics_yuv)          \
  EXPANDER(statistics_yuv_ae_histogram_y)       \
  EXPANDER(chain_yuv_inter)         \
  EXPANDER(edge_enhancement_color_reduction_filter)   \
  EXPANDER(edge_enhancement_color_reduction_filter_ee_lut)  \
  EXPANDER(i3d_lut)           \
  EXPANDER(i3d_lut_lut_outside)         \
  EXPANDER(i3d_lut_lut_inside)          \
  EXPANDER(drop)

#define EXPAND_AS_ENUM(_node) \
  _node,

enum {
  AVI_DEFINE_NODE(EXPAND_AS_ENUM)
  ISP_NODE_NR,
};

/* ISP Context */
struct libisp_context {
  int           devmem;
  unsigned long avi_base;
  unsigned      offsets[ISP_NODE_NR];
};

/* Configuration of ISP */
struct libisp_config {
  struct avi_isp_vlformat_32to40_regs vlformat_32to40;    ///< Conversion factor (10bit to 10bit default)
  struct avi_isp_chain_bayer_inter_regs bayer_inter;      ///< Enable or disable bayer ISP functions by bypassing them
  struct avi_isp_pedestal_regs pedestal;                  ///< Pedestral parameters (substract from pixels)
  struct avi_isp_green_imbalance_regs green_imbalance;    ///< Green imbalance correction
  struct avi_isp_green_imbalance_green_red_coeff_mem_regs grim_gr;  ///< Green imbalance GR coefficients
  struct avi_isp_green_imbalance_green_blue_coeff_mem_regs grim_gb; ///< Green imbalance GB coefficients
  struct avi_isp_dead_pixel_correction_regs dead_pixel_correction;  ///< Dead pixel correction (disabled)
  struct avi_isp_denoising_regs denoising;                ///< Denoising parameters
  struct avi_isp_statistics_bayer_regs statistics_bayer;  ///< Statistics bayer parameters
  struct avi_isp_lens_shading_correction_regs lens_shading_correction;          ///< Lens shade correction
  struct avi_isp_lens_shading_correction_red_coeff_mem_regs
      lsc_red_coeffs;     ///< Lens shade correction red coefficients
  struct avi_isp_lens_shading_correction_green_coeff_mem_regs
      lsc_green_coeffs; ///< Lens shade correction green coefficients
  struct avi_isp_lens_shading_correction_blue_coeff_mem_regs
      lsc_blue_coeffs;   ///< Lens shade correction blue coefficients
  /*struct avi_isp_chromatic_aberration_regs chromatic_aberration;*/ ///< Chromatic abberation (Disabled for now)
  struct avi_isp_bayer_regs bayer;                        ///< Demosaicking parameters
  struct avi_isp_color_correction_regs color_correction;  ///< Color correction parameters
  struct avi_isp_vlformat_40to32_regs vlformat_40to32;    ///< Conversion factor (10bit to 10bit default)
  struct avi_isp_gamma_corrector_regs gamma_corrector;    ///< Gamma corrector (Curves)
  struct avi_isp_gamma_corrector_ry_lut_regs gc_ry_lut;   ///< Gamma corrector RY lut
  struct avi_isp_gamma_corrector_gu_lut_regs gc_gu_lut;   ///< Gamma corrector GU lut
  struct avi_isp_gamma_corrector_bv_lut_regs gc_bv_lut;   ///< Gamma corrector BV lut
  struct avi_isp_chroma_regs chroma;                      ///< Color space conversion
  struct avi_isp_statistics_yuv_regs statistics_yuv;      ///< YUV statistics parameters
  struct avi_isp_edge_enhancement_color_reduction_filter_regs eecrf;            ///< Edge enhancement + Color reduction
  struct avi_isp_edge_enhancement_color_reduction_filter_ee_lut_regs
      eecrf_lut; ///< Edge enhancement + Color correction lut
  struct avi_isp_chain_yuv_inter_regs chain_yuv_inter;    ///< YUV chain bypass configuration (enable/disable features)
};

/* Output YUV statistics */
struct isp_yuv_stats_t {
  uint32_t awb_sum_Y;
  uint32_t awb_sum_U;
  uint32_t awb_sum_V;
  uint32_t awb_nb_grey_pixels;
  uint32_t nb_valid_Y;
  uint32_t ae_histogram_Y[256];
};

extern int configure_isp(struct v4l2_device *dev);
extern int isp_get_statistics_yuv(struct isp_yuv_stats_t *yuv_stats);
extern int isp_request_statistics_yuv_window(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end,
                                      uint16_t x_odd_inc, uint16_t y_odd_inc);

/* Registers access */
#define EXPAND_AS_PROTOTYPE(_node)                                        \
  void avi_isp_ ## _node ## _set_registers(struct libisp_context *, \
      struct avi_isp_ ## _node ## _regs const *regs);   \
  void avi_isp_ ## _node ## _get_registers(struct libisp_context *, \
      struct avi_isp_ ## _node ## _regs *regs);

AVI_DEFINE_NODE(EXPAND_AS_PROTOTYPE)

#endif /* _LIBISP_H */
