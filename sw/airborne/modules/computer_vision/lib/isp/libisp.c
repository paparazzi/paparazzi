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

#define AVI_BASE 0x400000
#define AVI_SIZE 0x100000
#define AVI_MASK (AVI_SIZE - 1)

struct avi_isp_offsets
{
	uint32_t chain_bayer;
	uint32_t gamma_corrector;
	uint32_t chroma;
	uint32_t statistics_yuv;
	uint32_t chain_yuv;
};

/* IOCTL implemented in AVI drivers */
#define AVI_ISP_IOGET_OFFSETS _IOR('F', 0x33, struct avi_isp_offsets)

/* Raw accesses */
#define readl(_addr)            (*((volatile uint32_t *)(_addr)))
#define writel(_val, _addr)     (*((volatile uint32_t *)(_addr)) = _val)

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
	0,	/* GAMMA conf */
	AVI_ISP_GAMMA_CORRECTOR_RY_LUT,
	AVI_ISP_GAMMA_CORRECTOR_GU_LUT,
	AVI_ISP_GAMMA_CORRECTOR_BV_LUT,
	0,	/* CHROMA */
	0,	/* STATS YUV*/
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
		printf("sizeof: %d, %X\n", sizeof(struct avi_isp_offsets), AVI_ISP_IOGET_OFFSETS);
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

	if (avi_isp_get_offsets_fd(fd, &off) < 0)
		goto get_offsets_failed;

	/* Compute all the sub-modules offsets */
	/* Chain Bayer */
	for (i = chain_bayer_inter ; i < gamma_corrector ; i++)
		ctx->offsets[i] = ctx->avi_base + isp_bases[i] + off.chain_bayer;

	ctx->offsets[gamma_corrector]        = ctx->avi_base + isp_bases[i++] + off.gamma_corrector;
	ctx->offsets[gamma_corrector_ry_lut] = ctx->avi_base + isp_bases[i++] + off.gamma_corrector;
	ctx->offsets[gamma_corrector_gu_lut] = ctx->avi_base + isp_bases[i++] + off.gamma_corrector;
	ctx->offsets[gamma_corrector_bv_lut] = ctx->avi_base + isp_bases[i++] + off.gamma_corrector;
	ctx->offsets[chroma]                 = ctx->avi_base + isp_bases[i++] + off.chroma;
	ctx->offsets[statistics_yuv]         = ctx->avi_base + isp_bases[i++] + off.statistics_yuv;
	ctx->offsets[statistics_yuv_ae_histogram_y] = ctx->avi_base + isp_bases[i++] + off.statistics_yuv;

	/* Chain YUV */
	for (i = chain_yuv_inter ; i < ISP_NODE_NR ; i++)
		ctx->offsets[i] = ctx->avi_base + isp_bases[i] + off.chain_yuv;

	return 0;

get_offsets_failed:
	munmap((void *) ctx->avi_base, AVI_SIZE);

mmap_failed:
	close(ctx->devmem);

open_failed:
	return -1;
}

static int close_isp(struct libisp_context *ctx)
{
	int ret = 0;

	if (munmap((void *) ctx->avi_base, AVI_SIZE) == -1) {
		perror("munmap failed");
		ret = -1;
	}

	close(ctx->devmem);

	return ret;
}

int configure_isp(struct v4l2_device *dev)
{
	struct libisp_context isp_ctx;

	struct avi_isp_chain_bayer_inter_regs bayer_inter = {{
		.pedestal_bypass     = 1,
		.grim_bypass         = 1,
		.rip_bypass          = 1,
		.denoise_bypass      = 0,
		.lsc_bypass          = 1,
		.chroma_aber_bypass  = 1,
		.bayer_bypass        = 0,
		.color_matrix_bypass = 0,
	}};

	struct avi_isp_vlformat_32to40_regs vlf_32to40 = {{
		.format = 0x0,
	}};

	struct avi_isp_bayer_regs bay = {
		.cfa         = { ._register = 0x03 },
		.threshold_1 = { ._register = 0x19 },
		.threshold_2 = { ._register = 0xc8 },
	};

	struct avi_isp_color_correction_regs cc = {
		.coeff_01_00 = { ._register = 0xF3811477 },
		.coeff_10_02 = { ._register = 0xFDF0021d },
		.coeff_12_11 = { ._register = 0xFF9E0A33 },
		.coeff_21_20 = { ._register = 0xF4DFFE25 },
		.coeff_22    = { ._register = 0x00001B83 },
		.offset_ry   = { ._register = 0x00000000 },
		.clip_ry     = { ._register = 0x03FF0000 },
		.offset_gu   = { ._register = 0x00000000 },
		.clip_gu     = { ._register = 0x03FF0000 },
		.offset_bv   = { ._register = 0x00000000 },
		.clip_bv     = { ._register = 0x03FF0000 },
	};

	struct avi_isp_vlformat_40to32_regs vlf_40to32 = {{
		.format = 0x4,
	}};

#define COMPLEMENT_2(i, r) (((i) >= 0) ? (r) : (~(r) + 1) & 0x3fff)
#define Q311(i) (COMPLEMENT_2(i, (unsigned)(((ABS(i)) * (1 << 11)) + 0.5)))

/*
 * Chroma converter parameters to convert input stream from a given
 * color space to another.
 * See http://www.fourcc.org/fccyvrgb.php
 */

#define AVI_CONV_MATRIX(_c00, _c01, _c02,                                    \
                        _c10, _c11, _c12,                                    \
                        _c20, _c21, _c22)                                    \
        .coeff_01_00 = {{ .coeff_00 = Q311(_c00), .coeff_01 = Q311(_c01) }}, \
        .coeff_10_02 = {{ .coeff_02 = Q311(_c02), .coeff_10 = Q311(_c10) }}, \
        .coeff_12_11 = {{ .coeff_11 = Q311(_c11), .coeff_12 = Q311(_c12) }}, \
        .coeff_21_20 = {{ .coeff_20 = Q311(_c20), .coeff_21 = Q311(_c21) }}, \
        .coeff_22    = {{ .coeff_22 = Q311(_c22) }}

#define AVI_CONV_OFFSETS(_ryin, _ryout,                                 \
                         _guin, _guout,                                 \
                         _bvin, _bvout)                                 \
        .offset_ry = {{ .offset_in = _ryin, .offset_out = _ryout }},    \
        .offset_gu = {{ .offset_in = _guin, .offset_out = _guout }},    \
        .offset_bv = {{ .offset_in = _bvin, .offset_out = _bvout }}

#define AVI_CONV_CLIPS(_rymin, _rymax,                                  \
                       _gumin, _gumax,                                  \
                       _bvmin, _bvmax)                                  \
        .clip_ry = {{ .clip_min = _rymin, .clip_max = _rymax }},        \
        .clip_gu = {{ .clip_min = _gumin, .clip_max = _gumax }},        \
        .clip_bv = {{ .clip_min = _bvmin, .clip_max = _bvmax }}


	struct avi_isp_chroma_regs chr = {
		AVI_CONV_MATRIX(  0.213,  0.715,  0.072,
		                 -0.100, -0.336,  0.436,
		                  0.615, -0.515, -0.100),

		AVI_CONV_OFFSETS(0, 16,
		                 0, 128,
		                 0, 128),

		AVI_CONV_CLIPS(16, 235,
		               16, 240,
		               16, 240),
	};

	if(open_isp_fd(&isp_ctx, dev->fd) < 0)
		return -1;

	avi_isp_chain_bayer_inter_set_registers(&isp_ctx, &bayer_inter);
	avi_isp_vlformat_32to40_set_registers(&isp_ctx, &vlf_32to40);
	avi_isp_bayer_set_registers(&isp_ctx, &bay);
	avi_isp_color_correction_set_registers(&isp_ctx, &cc);
	avi_isp_vlformat_40to32_set_registers(&isp_ctx, &vlf_40to32);
	avi_isp_chroma_set_registers(&isp_ctx, &chr);

	close_isp(&isp_ctx);

	return 0;
}


static inline void memcpy_to_registers(unsigned long addr,
                                       const void *reg_base,
                                       size_t s)
{
		const uint32_t *reg = reg_base;
		unsigned i;

		s /= sizeof(uint32_t); /* we write one register at a time */

		for (i = 0; i < s; i++)
			writel(reg[i], addr + i * sizeof(uint32_t));
}

static inline void memcpy_from_registers(void *reg_base,
                                         unsigned long addr,
                                         size_t s)
{
	uint32_t *reg = reg_base;
	unsigned i;

	s /= sizeof(uint32_t); /* we read one register at a time */

	for (i = 0; i < s; i++)
		reg[i] = readl(addr + i * sizeof(uint32_t));
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
