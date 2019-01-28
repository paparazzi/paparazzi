/*********************************************************************
 * avi_isp_green_imbalance register map
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

#ifndef _AVI_ISP_GREEN_IMBALANCE_H_
#define _AVI_ISP_GREEN_IMBALANCE_H_

#define AVI_ISP_GREEN_IMBALANCE_BAYER_CFA     0x00
#define AVI_ISP_GREEN_IMBALANCE_OFFSET_X_Y    0x04
#define AVI_ISP_GREEN_IMBALANCE_CELL_ID_X_Y   0x08
#define AVI_ISP_GREEN_IMBALANCE_CELL_W        0x0c
#define AVI_ISP_GREEN_IMBALANCE_CELL_H        0x10
#define AVI_ISP_GREEN_IMBALANCE_CELL_W_INV    0x14
#define AVI_ISP_GREEN_IMBALANCE_CELL_H_INV    0x18
#define AVI_ISP_GREEN_IMBALANCE_ALPHA         0x1c
#define AVI_ISP_GREEN_IMBALANCE_BETA          0x20

union avi_isp_green_imbalance_bayer_cfa
{
	struct
	{
		uint32_t cfa             :  2;
	};
	uint32_t _register;
};

union avi_isp_green_imbalance_offset_x_y
{
	struct
	{
		uint32_t offset_x        :  9;
		unsigned /*unused */     :  7;
		uint32_t offset_y        : 10;
	};
	uint32_t _register;
};

union avi_isp_green_imbalance_cell_id_x_y
{
	struct
	{
		uint32_t cell_id_x       :  4;
		unsigned /*unused */     : 12;
		uint32_t cell_id_y       :  4;
	};
	uint32_t _register;
};

union avi_isp_green_imbalance_cell_w
{
	struct
	{
		uint32_t cell_w          :  9;
	};
	uint32_t _register;
};

union avi_isp_green_imbalance_cell_h
{
	struct
	{
		uint32_t cell_h          : 10;
	};
	uint32_t _register;
};

union avi_isp_green_imbalance_cell_w_inv
{
	struct
	{
		uint32_t w_inv           : 17;
	};
	uint32_t _register;
};

union avi_isp_green_imbalance_cell_h_inv
{
	struct
	{
		uint32_t h_inv           : 17;
	};
	uint32_t _register;
};

union avi_isp_green_imbalance_alpha
{
	struct
	{
		uint32_t alpha           : 17;
	};
	uint32_t _register;
};

union avi_isp_green_imbalance_beta
{
	struct
	{
		uint32_t beta            : 17;
	};
	uint32_t _register;
};

struct avi_isp_green_imbalance_regs
{
	union avi_isp_green_imbalance_bayer_cfa bayer_cfa;                  /* 0x000 */
	union avi_isp_green_imbalance_offset_x_y offset_x_y;                 /* 0x004 */
	union avi_isp_green_imbalance_cell_id_x_y cell_id_x_y;                /* 0x008 */
	union avi_isp_green_imbalance_cell_w cell_w;                     /* 0x00c */
	union avi_isp_green_imbalance_cell_h cell_h;                     /* 0x010 */
	union avi_isp_green_imbalance_cell_w_inv cell_w_inv;                 /* 0x014 */
	union avi_isp_green_imbalance_cell_h_inv cell_h_inv;                 /* 0x018 */
	union avi_isp_green_imbalance_alpha  alpha;                      /* 0x01c */
	union avi_isp_green_imbalance_beta   beta;                       /* 0x020 */
};

#define AVI_ISP_GREEN_IMBALANCE_GREEN_RED_COEFF_MEM  0x1000

union avi_isp_green_imbalance_green_red_coeff_mem
{
	struct
	{
		uint32_t gr_coeff_value   :  8;
	};
	uint32_t _register;
};

struct avi_isp_green_imbalance_green_red_coeff_mem_regs
{
	union avi_isp_green_imbalance_green_red_coeff_mem red_coeff_mem[221];
};

#define AVI_ISP_GREEN_IMBALANCE_GREEN_BLUE_COEFF_MEM  0x1400

union avi_isp_green_imbalance_green_blue_coeff_mem
{
	struct
	{
		uint32_t gb_coeff_value   :  8;
	};
	uint32_t _register;
};

struct avi_isp_green_imbalance_green_blue_coeff_mem_regs
{
	union avi_isp_green_imbalance_green_blue_coeff_mem green_coeff_mem[221];
};

#endif /* _AVI_ISP_GREEN_IMBALANCE_H_ */
