#include "vor_int_demod_decim.h"

#include  "vor_int_filters_decim.h"

#include  "lo.h"

// Variables pour la démodulation REF
int32_t vid_ref_sig;

int32_t vid_loREF;
int32_t vid_phaseREF;
int32_t vid_ref_err;
int32_t vid_yREF;

#undef VIF_RES
#define VIF_RES   10
#define VID_REF (1<<10)
const int32_t vid_alphaREF = VIF_PCOEF(0.003);
const int32_t vid_DELTA_9960 = 500*1;

// Variable pour la décimation REF
int32_t vid_ref_err_decim1;
int32_t vid_ref_err_decim2;

// Variable pour la démodulation du VAR
int32_t vid_var_sig;

int32_t vid_var_err_decim1;
int32_t vid_var_err_decim2;

int32_t vid_loVAR;
int32_t vid_phaseVAR;
int32_t vid_var_err;
int32_t vid_yVAR;
int32_t vid_cumVAR;

#define VID_VAR (1<<5)
const int32_t vid_DELTA_30 = 27*300;

// Variable pour la démodulation FM
int32_t vid_loFM;
int32_t vid_phaseFM;
int32_t vid_fm_err;
int32_t vid_yFM;
int32_t vid_cumFM;

#define VID_FM (1<<5)

// Le QDR
int32_t vid_qdr;
uint8_t vid_qdr_available;
const int32_t correcQDR = 454; // 5

uint32_t decim1 = 0;
#define vid_DECIM1 2
uint32_t decim2 = 0;
#define vid_DECIM2 9
uint32_t decim3 = 0;
#define vid_DECIM3 9

// Variables liées au séquencement des switchs
int32_t fix_var_sig;
int32_t fix_var_err_decim1;
int32_t fix_var_err_decim2;

#define pllREF() {						    \
    int32_t vid_phiREF = -vid_alphaREF*vid_ref_err;		    \
    vid_phiREF /= VID_REF;					    \
    vid_phaseREF +=  vid_DELTA_9960 + vid_phiREF;		    \
    if (vid_phaseREF >= N_REF) {				    \
      vid_phaseREF -= N_REF;					    \
    }								    \
    vid_loREF = loREF[vid_phaseREF];				    \
    vid_yREF = vid_ref_sig*vid_loREF;				    \
    vid_yREF /= VID_LO;						    \
    vid_ref_err = vor_int_filter_lp_ref(vid_yREF);		    \
  }

#define pllVAR() {						    \
    vid_var_err /= VID_VAR;					    \
    vid_cumVAR += vid_var_err;					    \
    vid_phaseVAR += vid_DELTA_30 + vid_var_err;			    \
    if (vid_phaseVAR >= N_VAR_FM) {				    \
      vid_phaseVAR -= N_VAR_FM;					    \
    }								    \
    vid_loVAR = lo[vid_phaseVAR];				    \
    vid_yVAR = vid_var_err_decim2*vid_loVAR;			    \
    vid_yVAR /= VID_LO;						    \
    vid_var_err = vor_int_filter_lp_var4(vid_yVAR);		    \
  }

#define pllFM() {						    \
    vid_fm_err /= VID_FM;					    \
    vid_cumFM += vid_fm_err;					    \
    vid_phaseFM +=  vid_DELTA_30 + vid_fm_err;			    \
    if (vid_phaseFM >= N_VAR_FM) {				    \
      vid_phaseFM -= N_VAR_FM;					    \
    }								    \
    vid_loFM = lo[vid_phaseFM];					    \
    vid_yFM = vid_ref_err_decim2*vid_loFM;			    \
    vid_yFM /= VID_LO;						    \
    vid_fm_err = vor_int_filter_lp_fm4(vid_yFM);		    \
  }

#define calculQDR() {						    \
    vid_qdr = vid_cumVAR - vid_cumFM;				    \
    								    \
    if (vid_qdr >= N_VAR_FM) {					    \
      vid_qdr -= N_VAR_FM;					    \
      vid_cumVAR = vid_qdr;					    \
      vid_cumFM = 0;						    \
    }								    \
    else {							    \
      if (vid_qdr <= -N_VAR_FM) {				    \
	vid_qdr += N_VAR_FM;					    \
	vid_cumVAR = 0;						    \
	vid_cumFM = vid_qdr;					    \
      }								    \
      else {							    \
	vid_cumVAR = vid_qdr;					    \
	vid_cumFM = 0;						    \
      }								    \
    }								    \
    								    \
    vid_qdr -= correcQDR;					    \
  }

void vor_int_demod_init( void) {
  vid_qdr_available = FALSE;
}

void vor_int_demod_run ( int16_t sample) {

  decim1++;

  // Le signal arrive sur 10 bits, on le met sur 16 bits
  sample = sample*(1<<6);

  // get VAR signal by bandpassing input signal 
  vid_var_sig = vor_int_filter_bp_var(sample);

  // get REF signal by bandpassing input signal 
  vid_ref_sig = vor_int_filter_bp_ref(sample);

  //=================================================================
  switch (decim1) {

    //---------------------------------------------------------------
    case 1 : {

      decim2++;
    
      pllREF()

      // filter 30 REF before decimation 2
      vid_ref_err_decim1 = vor_int_filter_lp_decim1r(vid_ref_err);
      
      fix_var_sig = vid_var_sig;

      break;
    }
    //---------------------------------------------------------------

    //---------------------------------------------------------------
    case vid_DECIM1 : {

      decim1 = 0; 

      // filter 30 VAR before decimation 2
      vid_var_err_decim1 = vor_int_filter_lp_decim1v(fix_var_sig);

      //=============================================================
      switch (decim2) {

        //-----------------------------------------------------------
        case (vid_DECIM2-2) : {

          decim3++;

          // filter 30 REF before decimation 3
          vid_ref_err_decim2 = 
          vor_int_filter_lp_decim2r(vid_ref_err_decim1);

          fix_var_err_decim1 = vid_var_err_decim1;

          break;
        }
        //-----------------------------------------------------------

        //-----------------------------------------------------------
        case (vid_DECIM2-1) : {

          // filter 30 VAR before decimation 3
          vid_var_err_decim2 = 
          vor_int_filter_lp_decim2v(fix_var_err_decim1);

          break;
        }
        //-----------------------------------------------------------

        //-----------------------------------------------------------
        case (vid_DECIM2) : {

          decim2 = 0;

          //=========================================================
          switch (decim3) {

            //-------------------------------------------------------
            case (vid_DECIM3-2) : {

              pllFM();

              fix_var_err_decim2 = vid_var_err_decim2;

              break;
            }
            //-------------------------------------------------------

            //-------------------------------------------------------
            case (vid_DECIM3-1) : {

              vid_var_err_decim2 = fix_var_err_decim2;

              pllVAR();

              break;
            }
            //-------------------------------------------------------

            //-------------------------------------------------------
            case vid_DECIM3 : {

              decim3 = 0;

              calculQDR();
	      vid_qdr_available = TRUE;
              break;
            }
            //-------------------------------------------------------
          }  
          //=========================================================

          break;
        }
        //-----------------------------------------------------------
      }
      //=============================================================

    break;
    }
    //---------------------------------------------------------------
  }
  //=================================================================

}

