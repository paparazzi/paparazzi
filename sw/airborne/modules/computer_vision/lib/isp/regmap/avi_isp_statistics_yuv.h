/*********************************************************************
 * avi_isp_statistics_yuv register map
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

#ifndef _AVI_ISP_STATISTICS_YUV_H_
#define _AVI_ISP_STATISTICS_YUV_H_

#define AVI_ISP_STATISTICS_YUV_MEASURE_REQ            0x00
#define AVI_ISP_STATISTICS_YUV_MEASURE_STATUS         0x04
#define AVI_ISP_STATISTICS_YUV_WINDOW_POS_X           0x08
#define AVI_ISP_STATISTICS_YUV_WINDOW_POS_Y           0x0c
#define AVI_ISP_STATISTICS_YUV_CIRCLE_POS_X_CENTER    0x10
#define AVI_ISP_STATISTICS_YUV_CIRCLE_POS_X_SQUARED   0x14
#define AVI_ISP_STATISTICS_YUV_CIRCLE_POS_Y_CENTER    0x18
#define AVI_ISP_STATISTICS_YUV_CIRCLE_POS_Y_SQUARED   0x1c
#define AVI_ISP_STATISTICS_YUV_CIRCLE_RADIUS_SQUARED  0x20
#define AVI_ISP_STATISTICS_YUV_INCREMENTS_LOG2        0x24
#define AVI_ISP_STATISTICS_YUV_AE_NB_VALID_Y          0x30
#define AVI_ISP_STATISTICS_YUV_AWB_THRESHOLD          0x40
#define AVI_ISP_STATISTICS_YUV_AWB_SUM_Y              0x44
#define AVI_ISP_STATISTICS_YUV_AWB_SUM_U              0x48
#define AVI_ISP_STATISTICS_YUV_AWB_SUM_V              0x4c
#define AVI_ISP_STATISTICS_YUV_AWB_NB_GREY_PIXELS     0x50

union avi_isp_statistics_yuv_measure_req
{
	struct
	{
		uint32_t measure_req     :  1;
		uint32_t clear           :  1;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_measure_status
{
	struct
	{
		uint32_t done            :  1;
		uint32_t error           :  1;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_window_pos_x
{
	struct
	{
		uint32_t window_x_start  : 13;
		unsigned /*unused */     :  3;
		uint32_t window_x_end    : 13;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_window_pos_y
{
	struct
	{
		uint32_t window_y_start  : 13;
		unsigned /*unused */     :  3;
		uint32_t window_y_end    : 13;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_circle_pos_x_center
{
	struct
	{
		uint32_t x_center        : 14;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_circle_pos_x_squared
{
	struct
	{
		uint32_t x_squared       : 26;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_circle_pos_y_center
{
	struct
	{
		uint32_t y_center        : 14;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_circle_pos_y_squared
{
	struct
	{
		uint32_t y_squared       : 26;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_circle_radius_squared
{
	struct
	{
		uint32_t radius_squared  : 29;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_increments_log2
{
	struct
	{
		uint32_t x_log2_inc      :  4;
		unsigned /*unused */     : 13;
		uint32_t y_log2_inc      :  4;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_ae_nb_valid_y
{
	struct
	{
		uint32_t nb_valid_y      : 22;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_awb_threshold
{
	struct
	{
		uint32_t awb_threshold   :  8;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_awb_sum_y
{
	struct
	{
		uint32_t awb_sum_y       : 30;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_awb_sum_u
{
	struct
	{
		uint32_t awb_sum_u       : 30;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_awb_sum_v
{
	struct
	{
		uint32_t awb_sum_v       : 30;
	};
	uint32_t _register;
};

union avi_isp_statistics_yuv_awb_nb_grey_pixels
{
	struct
	{
		uint32_t nb_grey_pixels  : 22;
	};
	uint32_t _register;
};

struct avi_isp_statistics_yuv_regs
{
	union avi_isp_statistics_yuv_measure_req     measure_req;                /* 0x000 */
	union avi_isp_statistics_yuv_measure_status  measure_status;             /* 0x004 */
	union avi_isp_statistics_yuv_window_pos_x    window_pos_x;               /* 0x008 */
	union avi_isp_statistics_yuv_window_pos_y    window_pos_y;               /* 0x00c */
	union avi_isp_statistics_yuv_circle_pos_x_center circle_pos_x_center;        /* 0x010 */
	union avi_isp_statistics_yuv_circle_pos_x_squared circle_pos_x_squared;       /* 0x014 */
	union avi_isp_statistics_yuv_circle_pos_y_center circle_pos_y_center;        /* 0x018 */
	union avi_isp_statistics_yuv_circle_pos_y_squared circle_pos_y_squared;       /* 0x01c */
	union avi_isp_statistics_yuv_circle_radius_squared circle_radius_squared;      /* 0x020 */
	union avi_isp_statistics_yuv_increments_log2 increments_log2;            /* 0x024 */
	unsigned                                 /*unused*/ : 32;            /* 0x028 */
	unsigned                                 /*unused*/ : 32;            /* 0x02c */
	union avi_isp_statistics_yuv_ae_nb_valid_y   ae_nb_valid_y;              /* 0x030 */
	unsigned                                 /*unused*/ : 32;            /* 0x034 */
	unsigned                                 /*unused*/ : 32;            /* 0x038 */
	unsigned                                 /*unused*/ : 32;            /* 0x03c */
	union avi_isp_statistics_yuv_awb_threshold   awb_threshold;              /* 0x040 */
	union avi_isp_statistics_yuv_awb_sum_y       awb_sum_y;                  /* 0x044 */
	union avi_isp_statistics_yuv_awb_sum_u       awb_sum_u;                  /* 0x048 */
	union avi_isp_statistics_yuv_awb_sum_v       awb_sum_v;                  /* 0x04c */
	union avi_isp_statistics_yuv_awb_nb_grey_pixels awb_nb_grey_pixels;         /* 0x050 */
};

#define AVI_ISP_STATISTICS_YUV_AE_HISTOGRAM_Y         0x400

union avi_isp_statistics_yuv_ae_histogram_y
{
	struct
	{
		uint32_t histogram_y     : 22;
	};
	uint32_t _register;
};

struct avi_isp_statistics_yuv_ae_histogram_y_regs
{
	union avi_isp_statistics_yuv_ae_histogram_y ae_histogram_y[256];
};

#endif /* _AVI_ISP_STATISTICS_YUV_H_ */
