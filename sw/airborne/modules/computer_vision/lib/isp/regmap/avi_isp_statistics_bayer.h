/*********************************************************************
 * avi_isp_statistics_bayer register map
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

#ifndef _AVI_ISP_STATISTICS_BAYER_H_
#define _AVI_ISP_STATISTICS_BAYER_H_

#define AVI_ISP_STATISTICS_BAYER_MEASURE_REQ          0x00
#define AVI_ISP_STATISTICS_BAYER_WINDOW_X             0x04
#define AVI_ISP_STATISTICS_BAYER_WINDOW_Y             0x08
#define AVI_ISP_STATISTICS_BAYER_CIRCLE_POS_X_CENTER  0x0c
#define AVI_ISP_STATISTICS_BAYER_CIRCLE_POS_X_SQUARED  0x10
#define AVI_ISP_STATISTICS_BAYER_CIRCLE_POS_Y_CENTER  0x14
#define AVI_ISP_STATISTICS_BAYER_CIRCLE_POS_Y_SQUARED  0x18
#define AVI_ISP_STATISTICS_BAYER_CIRCLE_RADIUS_SQUARED  0x1c
#define AVI_ISP_STATISTICS_BAYER_INCREMENTS_LOG2      0x20
#define AVI_ISP_STATISTICS_BAYER_SAT_THRESHOLD        0x24
#define AVI_ISP_STATISTICS_BAYER_CFA                  0x28
#define AVI_ISP_STATISTICS_BAYER_MAX_NB_WINDOWS       0x2c

union avi_isp_statistics_bayer_measure_req
{
	struct
	{
		uint32_t clear           :  1;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_window_x
{
	struct
	{
		uint32_t x_offset        : 13;
		unsigned /*unused */     :  3;
		uint32_t x_width         : 11;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_window_y
{
	struct
	{
		uint32_t y_offset        : 13;
		unsigned /*unused */     :  3;
		uint32_t y_width         : 11;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_circle_pos_x_center
{
	struct
	{
		uint32_t x_center        : 14;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_circle_pos_x_squared
{
	struct
	{
		uint32_t x_squared       : 26;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_circle_pos_y_center
{
	struct
	{
		uint32_t y_center        : 14;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_circle_pos_y_squared
{
	struct
	{
		uint32_t y_squared       : 26;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_circle_radius_squared
{
	struct
	{
		uint32_t radius_squared  : 29;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_increments_log2
{
	struct
	{
		uint32_t x_log2_inc      :  3;
		unsigned /*unused */     : 13;
		uint32_t y_log2_inc      :  3;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_sat_threshold
{
	struct
	{
		uint32_t threshold       : 10;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_cfa
{
	struct
	{
		uint32_t cfa             :  2;
	};
	uint32_t _register;
};

union avi_isp_statistics_bayer_max_nb_windows
{
	struct
	{
		uint32_t x_window_count  :  7;
		unsigned /*unused */     :  9;
		uint32_t y_window_count  :  7;
	};
	uint32_t _register;
};

struct avi_isp_statistics_bayer_regs
{
	union avi_isp_statistics_bayer_measure_req   measure_req;                /* 0x000 */
	union avi_isp_statistics_bayer_window_x      window_x;                   /* 0x004 */
	union avi_isp_statistics_bayer_window_y      window_y;                   /* 0x008 */
	union avi_isp_statistics_bayer_circle_pos_x_center circle_pos_x_center;        /* 0x00c */
	union avi_isp_statistics_bayer_circle_pos_x_squared circle_pos_x_squared;       /* 0x010 */
	union avi_isp_statistics_bayer_circle_pos_y_center circle_pos_y_center;        /* 0x014 */
	union avi_isp_statistics_bayer_circle_pos_y_squared circle_pos_y_squared;       /* 0x018 */
	union avi_isp_statistics_bayer_circle_radius_squared circle_radius_squared;      /* 0x01c */
	union avi_isp_statistics_bayer_increments_log2 increments_log2;            /* 0x020 */
	union avi_isp_statistics_bayer_sat_threshold sat_threshold;              /* 0x024 */
	union avi_isp_statistics_bayer_cfa           cfa;                        /* 0x028 */
	union avi_isp_statistics_bayer_max_nb_windows max_nb_windows;             /* 0x02c */
};

#endif /* _AVI_ISP_STATISTICS_BAYER_H_ */
