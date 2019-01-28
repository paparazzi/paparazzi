#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "libisp.h"
#include "libisp_config.h"

#define PRINT(string,...) fprintf(stderr, "[libisp->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if MT9F002_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#define AVI_BASE 0x400000
#define AVI_SIZE 0x100000
#define AVI_MASK (AVI_SIZE - 1)

struct avi_isp_offsets {
  uint32_t chain_bayer;
  uint32_t gamma_corrector;
  uint32_t chroma;
  uint32_t statistics_yuv;
  uint32_t chain_yuv;
};

#define AVI_ISP_STAT_YUV_MAX_WAIT 3
uint8_t curWait = 0;

/* IOCTL implemented in AVI drivers */
#define AVI_ISP_IOGET_OFFSETS _IOR('F', 0x33, struct avi_isp_offsets)

/* Raw accesses */
#define readl(_addr)            (*((volatile uint32_t *)(_addr)))
#define writel(_val, _addr)     (*((volatile uint32_t *)(_addr)) = _val)

/* ISP context */
static struct libisp_context isp_ctx = {
  .devmem = -1
};

uint16_t requestWindow[6] = {0};

static const unsigned isp_bases[] = {
  AVI_ISP_CHAIN_BAYER_INTER,
  AVI_ISP_VLFORMAT_32TO40,
  AVI_ISP_PEDESTAL,
  AVI_ISP_GREEN_IMBALANCE,
  AVI_ISP_GREEN_IMBALANCE + AVI_ISP_GREEN_IMBALANCE_GREEN_RED_COEFF_MEM,
  AVI_ISP_GREEN_IMBALANCE + AVI_ISP_GREEN_IMBALANCE_GREEN_BLUE_COEFF_MEM,
  AVI_ISP_DEAD_PIXEL_CORRECTION + AVI_ISP_DEAD_PIXEL_CORRECTION_CFA,
  AVI_ISP_DEAD_PIXEL_CORRECTION + AVI_ISP_DEAD_PIXEL_CORRECTION_LIST_MEM,
  AVI_ISP_DENOISING,
  AVI_ISP_STATISTICS_BAYER,
  AVI_ISP_LENS_SHADING_CORRECTION,
  AVI_ISP_LENS_SHADING_CORRECTION + AVI_ISP_LENS_SHADING_CORRECTION_RED_COEFF_MEM,
  AVI_ISP_LENS_SHADING_CORRECTION + AVI_ISP_LENS_SHADING_CORRECTION_GREEN_COEFF_MEM,
  AVI_ISP_LENS_SHADING_CORRECTION + AVI_ISP_LENS_SHADING_CORRECTION_BLUE_COEFF_MEM,
  AVI_ISP_CHROMATIC_ABERRATION,
  AVI_ISP_BAYER,
  AVI_ISP_COLOR_CORRECTION,
  AVI_ISP_VLFORMAT_40TO32,
  0,  /* GAMMA conf */
  AVI_ISP_GAMMA_CORRECTOR_RY_LUT,
  AVI_ISP_GAMMA_CORRECTOR_GU_LUT,
  AVI_ISP_GAMMA_CORRECTOR_BV_LUT,
  0,  /* CHROMA */
  0,  /* STATS YUV*/
  AVI_ISP_STATISTICS_YUV_AE_HISTOGRAM_Y,
  AVI_ISP_CHAIN_YUV_INTER,
  AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER + AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_EE_KERNEL_COEFF,
  AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER + AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_EE_LUT,
  AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_CLIP_MODE,
  AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_LUT_OUTSIDE,
  AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_LUT_INSIDE,
  AVI_ISP_DROP,
};

/**
 * This is taken from libisp. Original function takes the videodev name and re-opens it,
 * causing undefined behaviour. Here we reuse the already-opened file descriptor
 */
static int avi_isp_get_offsets_fd(int fd, struct avi_isp_offsets *off)
{
  if (ioctl(fd, AVI_ISP_IOGET_OFFSETS, off) < 0) {
    VERBOSE_PRINT("sizeof: %d, %X\n", sizeof(struct avi_isp_offsets), AVI_ISP_IOGET_OFFSETS);
    perror("ioctl(AVI_ISP_IOGET_OFFSETS) failed");
    return -1;
  }

  return 0;
}

/**
 * This is taken from libisp. Original function takes the videodev name and re-opens it,
 * causing undefined behaviour. Here we reuse the already-opened file descriptor
 */
static int open_isp_fd(struct libisp_context *ctx, int fd)
{
  struct avi_isp_offsets off;
  int                    i;

  ctx->devmem = open("/dev/mem", O_RDWR);

  if (ctx->devmem < 0) {
    perror("Can't open /dev/mem");
    goto open_failed;
  }

  ctx->avi_base = (unsigned long) mmap(NULL, AVI_SIZE,
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED, ctx->devmem, AVI_BASE & ~AVI_MASK);

  if (ctx->avi_base == (unsigned long) MAP_FAILED) {
    perror("mmap failed");
    goto mmap_failed;
  }

  if (avi_isp_get_offsets_fd(fd, &off) < 0) {
    goto get_offsets_failed;
  }

  /* Compute all the sub-modules offsets */
  /* Chain Bayer */
  for (i = chain_bayer_inter ; i < gamma_corrector ; i++) {
    ctx->offsets[i] = ctx->avi_base + isp_bases[i] + off.chain_bayer;
  }

  ctx->offsets[gamma_corrector]        = ctx->avi_base + isp_bases[i++] + off.gamma_corrector;
  ctx->offsets[gamma_corrector_ry_lut] = ctx->avi_base + isp_bases[i++] + off.gamma_corrector;
  ctx->offsets[gamma_corrector_gu_lut] = ctx->avi_base + isp_bases[i++] + off.gamma_corrector;
  ctx->offsets[gamma_corrector_bv_lut] = ctx->avi_base + isp_bases[i++] + off.gamma_corrector;
  ctx->offsets[chroma]                 = ctx->avi_base + isp_bases[i++] + off.chroma;
  ctx->offsets[statistics_yuv]         = ctx->avi_base + isp_bases[i++] + off.statistics_yuv;
  ctx->offsets[statistics_yuv_ae_histogram_y] = ctx->avi_base + isp_bases[i++] + off.statistics_yuv;

  /* Chain YUV */
  for (i = chain_yuv_inter ; i < ISP_NODE_NR ; i++) {
    ctx->offsets[i] = ctx->avi_base + isp_bases[i] + off.chain_yuv;
  }

  return 0;

get_offsets_failed:
  munmap((void *) ctx->avi_base, AVI_SIZE);

mmap_failed:
  close(ctx->devmem);
  ctx->devmem = -1;

open_failed:
  return -1;
}

/*static int close_isp(struct libisp_context *ctx)
{
  int ret = 0;

  if (munmap((void *) ctx->avi_base, AVI_SIZE) == -1) {
    perror("munmap failed");
    ret = -1;
  }

  close(ctx->devmem);
  ctx->devmem = -1;

  return ret;
}*/

int configure_isp(struct v4l2_device *dev)
{
  if (open_isp_fd(&isp_ctx, dev->fd) < 0) {
    return -1;
  }

  avi_isp_vlformat_32to40_set_registers(&isp_ctx, &isp_config.vlformat_32to40);
  avi_isp_chain_bayer_inter_set_registers(&isp_ctx, &isp_config.bayer_inter);
  avi_isp_pedestal_set_registers(&isp_ctx, &isp_config.pedestal);
  avi_isp_green_imbalance_set_registers(&isp_ctx, &isp_config.green_imbalance);
  avi_isp_green_imbalance_green_red_coeff_mem_set_registers(&isp_ctx, &isp_config.grim_gr);
  avi_isp_green_imbalance_green_blue_coeff_mem_set_registers(&isp_ctx, &isp_config.grim_gb);
  avi_isp_dead_pixel_correction_set_registers(&isp_ctx, &isp_config.dead_pixel_correction);
  avi_isp_denoising_set_registers(&isp_ctx, &isp_config.denoising);
  avi_isp_statistics_bayer_set_registers(&isp_ctx, &isp_config.statistics_bayer);
  avi_isp_lens_shading_correction_set_registers(&isp_ctx, &isp_config.lens_shading_correction);
  avi_isp_lens_shading_correction_red_coeff_mem_set_registers(&isp_ctx, &isp_config.lsc_red_coeffs);
  avi_isp_lens_shading_correction_green_coeff_mem_set_registers(&isp_ctx, &isp_config.lsc_green_coeffs);
  avi_isp_lens_shading_correction_blue_coeff_mem_set_registers(&isp_ctx, &isp_config.lsc_blue_coeffs);
  avi_isp_bayer_set_registers(&isp_ctx, &isp_config.bayer);
  avi_isp_color_correction_set_registers(&isp_ctx, &isp_config.color_correction);
  avi_isp_vlformat_40to32_set_registers(&isp_ctx, &isp_config.vlformat_40to32);
  avi_isp_gamma_corrector_set_registers(&isp_ctx, &isp_config.gamma_corrector);
  avi_isp_gamma_corrector_ry_lut_set_registers(&isp_ctx, &isp_config.gc_ry_lut);
  avi_isp_gamma_corrector_gu_lut_set_registers(&isp_ctx, &isp_config.gc_gu_lut);
  avi_isp_gamma_corrector_bv_lut_set_registers(&isp_ctx, &isp_config.gc_bv_lut);
  avi_isp_chroma_set_registers(&isp_ctx, &isp_config.chroma);
  avi_isp_statistics_yuv_set_registers(&isp_ctx, &isp_config.statistics_yuv);
  avi_isp_edge_enhancement_color_reduction_filter_set_registers(&isp_ctx, &isp_config.eecrf);
  avi_isp_edge_enhancement_color_reduction_filter_ee_lut_set_registers(&isp_ctx, &isp_config.eecrf_lut);
  avi_isp_chain_yuv_inter_get_registers(&isp_ctx, &isp_config.chain_yuv_inter);

  //close_isp(&isp_ctx);

  return 0;
}

int isp_request_statistics_yuv_window(uint16_t x_start, uint16_t x_end, uint16_t y_start, uint16_t y_end,
                                      uint16_t x_odd_inc, uint16_t y_odd_inc)
{
  requestWindow[0] = x_start;
  requestWindow[1] = x_end;
  requestWindow[2] = y_start;
  requestWindow[3] = y_end;
  requestWindow[4] = x_odd_inc;
  requestWindow[5] = y_odd_inc;

  VERBOSE_PRINT("[YUV-STAT] Requesting window: [%d %d],[%d %d], [%d %d]\n",
      requestWindow[0], requestWindow[1], requestWindow[2], requestWindow[3], requestWindow[4], requestWindow[5]);

  return 0;
}

static int isp_set_statistics_yuv_window(void)
{
  VERBOSE_PRINT("[YUV-STAT] Setting window: [%d %d],[%d %d]\n",
      requestWindow[0], requestWindow[1], requestWindow[2], requestWindow[3]);

  isp_config.statistics_yuv.window_pos_x.window_x_start   = requestWindow[0];
  isp_config.statistics_yuv.window_pos_x.window_x_end     = requestWindow[1];
  isp_config.statistics_yuv.window_pos_y.window_y_start   = requestWindow[2];
  isp_config.statistics_yuv.window_pos_y.window_y_end     = requestWindow[3];
  isp_config.statistics_yuv.increments_log2.x_log2_inc    = requestWindow[4];
  isp_config.statistics_yuv.increments_log2.y_log2_inc    = requestWindow[5];

  VERBOSE_PRINT("[YUV-STAT] Current settings: [%d %d] [%d %d] [%d %d] [%d %d] [%d] [%d %d] [%d]\n",
          isp_config.statistics_yuv.window_pos_x.window_x_start,
          isp_config.statistics_yuv.window_pos_x.window_x_end,
          isp_config.statistics_yuv.window_pos_y.window_y_start,
          isp_config.statistics_yuv.window_pos_y.window_y_end,
          isp_config.statistics_yuv.circle_pos_x_center.x_center,
          isp_config.statistics_yuv.circle_pos_x_squared.x_squared,
          isp_config.statistics_yuv.circle_pos_y_center.y_center,
          isp_config.statistics_yuv.circle_pos_y_squared.y_squared,
          isp_config.statistics_yuv.circle_radius_squared.radius_squared,
          isp_config.statistics_yuv.increments_log2.x_log2_inc,
          isp_config.statistics_yuv.increments_log2.y_log2_inc,
          isp_config.statistics_yuv.awb_threshold.awb_threshold
  );

  return 0;
}

/* Get YUV statistics */
int isp_get_statistics_yuv(struct isp_yuv_stats_t *yuv_stats)
{
  uint16_t i;

  if (isp_ctx.devmem < 0) {
    fprintf(stderr, "[YUV-STAT] Error isp_ctx.devmem < 0\n");
    return -1;
  }

  struct avi_isp_statistics_yuv_regs stats_yuv;
  avi_isp_statistics_yuv_get_registers(&isp_ctx, &stats_yuv);

  if (!stats_yuv.measure_status.done) {
    VERBOSE_PRINT("[YUV-STAT] Waiting for YUV stats\n");
    curWait++;
    if (curWait <= AVI_ISP_STAT_YUV_MAX_WAIT) {
      isp_config.statistics_yuv.measure_req.clear = 0; // Clear current results
    } else {
      isp_config.statistics_yuv.measure_req.clear = 1; // Clear current results
      isp_set_statistics_yuv_window();
      curWait = 0;
    }
    avi_isp_statistics_yuv_set_registers(&isp_ctx, &isp_config.statistics_yuv);
    return -1;
  } else if (stats_yuv.measure_status.error) {
    fprintf(stderr, "[YUV-STAT] Error requesting YUV stats\n");
    isp_config.statistics_yuv.measure_req.clear = 1; // Clear current results?
    curWait = 0;
    isp_set_statistics_yuv_window();
    avi_isp_statistics_yuv_set_registers(&isp_ctx, &isp_config.statistics_yuv);
    return -1;
  } else {
    isp_config.statistics_yuv.measure_req.clear = 1; // Clear current results?
  }
  curWait = 0;
  yuv_stats->awb_sum_Y = stats_yuv.awb_sum_y.awb_sum_y;
  yuv_stats->awb_sum_U = stats_yuv.awb_sum_u.awb_sum_u;
  yuv_stats->awb_sum_V = stats_yuv.awb_sum_v.awb_sum_v;
  yuv_stats->awb_nb_grey_pixels = stats_yuv.awb_nb_grey_pixels.nb_grey_pixels;
  yuv_stats->nb_valid_Y = stats_yuv.ae_nb_valid_y.nb_valid_y;

  // Histogram
  struct avi_isp_statistics_yuv_ae_histogram_y_regs histogram;
  avi_isp_statistics_yuv_ae_histogram_y_get_registers(&isp_ctx, &histogram);

  for (i = 0; i < 256; ++i) {
    yuv_stats->ae_histogram_Y[i] = histogram.ae_histogram_y[i].histogram_y;
  }
  isp_set_statistics_yuv_window();

  avi_isp_statistics_yuv_set_registers(&isp_ctx, &isp_config.statistics_yuv);
  return 0;
}

static inline void memcpy_to_registers(unsigned long addr,
                                       const void *reg_base,
                                       size_t s)
{
  const uint32_t *reg = reg_base;
  unsigned i;

  s /= sizeof(uint32_t); /* we write one register at a time */

  for (i = 0; i < s; i++) {
    writel(reg[i], addr + i * sizeof(uint32_t));
  }
}

static inline void memcpy_from_registers(void *reg_base,
    unsigned long addr,
    size_t s)
{
  uint32_t *reg = reg_base;
  unsigned i;

  s /= sizeof(uint32_t); /* we read one register at a time */

  for (i = 0; i < s; i++) {
    reg[i] = readl(addr + i * sizeof(uint32_t));
  }
}

#define EXPAND_AS_FUNCTION(_node)                                              \
  void avi_isp_ ## _node ## _set_registers(struct libisp_context *c,     \
      struct avi_isp_ ## _node ## _regs const *regs)         \
  {                                                                      \
    memcpy_to_registers(c->offsets[_node], regs, sizeof(*regs));   \
  }                                                                      \
  \
  void avi_isp_ ## _node ## _get_registers(struct libisp_context *c,     \
      struct avi_isp_ ## _node ## _regs *regs)               \
  {                                                                      \
    memcpy_from_registers(regs, c->offsets[_node], sizeof(*regs)); \
  }

AVI_DEFINE_NODE(EXPAND_AS_FUNCTION)
