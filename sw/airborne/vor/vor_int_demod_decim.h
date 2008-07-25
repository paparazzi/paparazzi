#ifndef VOR_INT_DEMOD_DECIM_H
#define VOR_INT_DEMOD_DECIM_H

#include <inttypes.h>
#include "std.h"


extern void vor_int_demod_init( void);
extern void vor_int_demod_run( int16_t sample);

extern       int32_t vid_ref_sig;

extern       int32_t vid_loREF;
extern	     int32_t vid_yREF;
extern	     int32_t vid_ref_err;
extern       int32_t vid_ref_err_decim1;
extern       int32_t vid_ref_err_decim2;

extern       int32_t vid_var_sig;
extern       int32_t vid_var_err_decim1;
extern       int32_t vid_var_err_decim2;
 
extern       int32_t vid_loVAR;
extern	     int32_t vid_yVAR;
extern	     int32_t vid_var_err;
extern       int32_t vid_cumVAR;

extern       int32_t vid_loFM;
extern	     int32_t vid_yFM;
extern	     int32_t vid_fm_err;
extern       int32_t vid_cumFM;

extern	     int32_t vid_qdr;
extern       uint8_t vid_qdr_available;


extern       const int32_t N_VAR_FM;

#endif /* VOR_INT_DEMOD_DECIM_H */
