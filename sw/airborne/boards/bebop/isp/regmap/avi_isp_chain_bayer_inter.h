/*********************************************************************
 * avi_isp_chain_bayer_inter register map
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

#ifndef _AVI_ISP_CHAIN_BAYER_INTER_H_
#define _AVI_ISP_CHAIN_BAYER_INTER_H_

#define AVI_ISP_CHAIN_BAYER_INTER_MODULE_BYPASS 0x00

union avi_isp_chain_bayer_inter_module_bypass
{
	struct
	{
		uint32_t pedestal_bypass     :  1;
		uint32_t grim_bypass         :  1;
		uint32_t rip_bypass          :  1;
		uint32_t denoise_bypass      :  1;
		uint32_t lsc_bypass          :  1;
		uint32_t chroma_aber_bypass  :  1;
		uint32_t bayer_bypass        :  1;
		uint32_t color_matrix_bypass :  1;
	};
	uint32_t _register;
};

struct avi_isp_chain_bayer_inter_regs
{
	union avi_isp_chain_bayer_inter_module_bypass module_bypass; /* 0x000 */
};

#endif /* _AVI_ISP_CHAIN_BAYER_INTER_H_ */
