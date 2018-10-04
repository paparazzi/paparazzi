
#include "ransac.h"
#include "filter.h"
#include "flightplan.h"
#include "math/RANSAC.h"
#include "stdio.h"

// RANSAC Measurement buffer size
#define  RANSAC_BUF_SIZE   30


// NOTE: the following defines determine how often there will be a RANSAC fit and a correction.

// number of measurements necessary for a fit:
#define RANSAC_MIN_SAMPLES_FOR_FIT 5
// used to be 1.0 for max and 0.5 for no vision
// max delta time (DT) for a measurement to be considered in the RANSAC buffer
#define RANSAC_DT_MAX 3.0
// delta time (DT) after which RANSAC decides that there are no vision measurements any more
// the correction will then be applied and the buffer emptied
#define RANSAC_DT_NO_VISION 0.5

// WARNING: UPDATE std.h in windows if buffer size is increased

// Buffer Data Type
struct dronerace_ransac_buf_struct
{
    // Settings
    float time;

    // Predicted States
    float x;
    float y;

    // Measured States
    float mx;
    float my;
};

// Variables
struct dronerace_ransac_buf_struct ransac_buf[RANSAC_BUF_SIZE];
struct dronerace_ransac_struct dr_ransac;


/////////////////////////////////////////////////////////////
// Helper functions

// The RANSAC buffer is a rolling buffer. The index wraps arround zero.
// @param[in]: From newest (0) to oldest (RANSAC_BUF_SIZE)
// @param[out]: the index of the element
inline int get_index(int element)
{
    int ind = dr_ransac.buf_index_of_last - element;
    if (ind < 0) { ind += RANSAC_BUF_SIZE; }
    return ind;
}

// The buffer size is computed from the time of the samples.
inline void ransac_update_buffer_size(void)
{
    int i;

    // Update buffer size
    dr_ransac.buf_size = 0;
    for (i=0; i<RANSAC_BUF_SIZE; i++ )
    {
        float mt = ransac_buf[get_index(i)].time;
        if ((mt == 0) || ((dr_state.time - mt) > dr_ransac.dt_max))
        {
            break;
        }
        dr_ransac.buf_size++;
    }
}


/////////////////////////////////////////////////////////////
// Helper functions

// Reset
void ransac_reset(void)
{
    int i;

    // Set the settings
    dr_ransac.dt_max = RANSAC_DT_MAX;
    dr_ransac.dt_novision = RANSAC_DT_NO_VISION;

    // Reset the buffer
    dr_ransac.buf_index_of_last = RANSAC_BUF_SIZE-1;
    dr_ransac.buf_size = 0;
    for (i=0; i<RANSAC_BUF_SIZE; i++) {
        ransac_buf[i].time = 0;
        ransac_buf[i].x = 0;
        ransac_buf[i].y = 0;
        ransac_buf[i].mx = 0;
        ransac_buf[i].my = 0;
    }

    // Reset the outputs
    dr_ransac.corr_x = dr_ransac.corr_y = 0;
    dr_ransac.corr_vx = dr_ransac.corr_vy = 0;
}


// @param[ax,ay]: acceleration prediction in EARTH axis
// @param[dt]: Time step in seconds

void ransac_propagate( void )
{
    // Propagate Internal Model
    // TODO: ?

    // Remove Old Vision updates from the buffer (if any)
    ransac_update_buffer_size();

    // If no vision for long time, reset nav:
    if (dr_ransac.buf_size > 0)
    {
        // IF the oldest vision sample is from X seconds ago, then reset
        float age = (dr_state.time - ransac_buf[dr_ransac.buf_index_of_last].time);
        if (age > dr_ransac.dt_novision)
        {
          //printf("\n\n*** RESET DUE TO NO VISION ***\n\n");
          //correct_state();
        }
    }
}

void correct_state() {
  // Apply correction to the Kalman Filter
  dr_state.x += dr_ransac.corr_x;
  dr_state.y += dr_ransac.corr_y;
  dr_state.vx += dr_ransac.corr_vx;
  dr_state.vy += dr_ransac.corr_vy;

  // Reset the RANSAC process
  ransac_reset();
}

// Add new measurement and DO RANSAC
// @param[time]: Filter time
// @param[x,y]: Filter predictions
// @param[mx,my]: Vision measurements
void ransac_push(float time, float _x, float _y, float _mx, float _my)
{
    int i = 0;

    // Insert the new sample in the buffer
    dr_ransac.buf_index_of_last++;
    if (dr_ransac.buf_index_of_last >= RANSAC_BUF_SIZE)
    {
        dr_ransac.buf_index_of_last = 0;
    }
    ransac_buf[dr_ransac.buf_index_of_last].time = time;
    ransac_buf[dr_ransac.buf_index_of_last].x = _x;
    ransac_buf[dr_ransac.buf_index_of_last].y = _y;
    ransac_buf[dr_ransac.buf_index_of_last].mx = _mx;
    ransac_buf[dr_ransac.buf_index_of_last].my = _my;

    // If sufficient items in buffer
    if (dr_ransac.buf_size > RANSAC_MIN_SAMPLES_FOR_FIT)
    {
        // Variables
        int n_samples = ((int)(dr_ransac.buf_size * 0.4));
        int n_iterations = 200;
        float error_threshold = 1.0;
        // int Dimension = 1;
        int count = dr_ransac.buf_size;
        float targets_x[RANSAC_BUF_SIZE];
        float targets_y[RANSAC_BUF_SIZE];
        float samples[RANSAC_BUF_SIZE][1];
        float params_x[2];
        float params_y[2];
        float fit_error;

        // Fill LINEAR MODEL to fit
        for (i=0;i<count;i++)
        {
            // Between the prediction and the vision, there is:
            // - a position offset = PARAM[0] * 1;
            // - a scale error = bias in speed (t - t0) = PARAM[1] * dt;
            // DX = PARAM * [dt  1];

            struct dronerace_ransac_buf_struct* r = &ransac_buf[get_index(i)];
            targets_x[i] = r->x - r->mx;
            targets_y[i] = r->y - r->my;
            samples[i][0] = r->time - dr_state.time;
            //printf("Fit %f to %f\n", targets_x[i], samples[i][0]);
        }

        //printf("Running RANSAC with %d points and %d samples, err_max %f\n",count, n_samples, error_threshold);
        RANSAC_linear_model(n_samples, n_iterations, error_threshold, targets_x, 1,
                                 samples, count, params_x, &fit_error);

        RANSAC_linear_model(n_samples, n_iterations, error_threshold, targets_y, 1,
                                         samples, count, params_y, &fit_error);

        // Export the RANSAC corrections
        dr_ransac.corr_x = -params_x[1];
        dr_ransac.corr_y = -params_y[1];
        dr_ransac.corr_vx = -params_x[0];
        dr_ransac.corr_vy = -params_y[0];

        dr_ransac.ransac_cnt ++;

        // Put EVERY FIT in a file to verify
#define DEBUG_RANSAC
#ifdef DEBUG_RANSAC

        if(dr_ransac.ransac_cnt % 3 == 0)
        {
            char filename[128];
            FILE* fp;
            sprintf(filename,"ransac%06d.txt",dr_ransac.ransac_cnt);
            fp = fopen(filename,"w");
            // fprintf(fp,"nr,time,x,y,mx,my,fitx,fity\n");
            // printf("t=%f %f\n\n",dr_state.time,dr_ransac.dt_max);
            for (i=0;i<dr_ransac.buf_size;i++)
            {
                float t_fit = (ransac_buf[get_index(i)].time - dr_state.time);
                fprintf(fp,"%d,%f,%f,%f,%f,%f,%f,%f\n",i,ransac_buf[get_index(i)].time,
                        ransac_buf[get_index(i)].x,
                        ransac_buf[get_index(i)].y,
                        ransac_buf[get_index(i)].mx,
                        ransac_buf[get_index(i)].my,
                        ransac_buf[get_index(i)].x + dr_ransac.corr_x + t_fit * dr_ransac.corr_vx,
                        ransac_buf[get_index(i)].y + dr_ransac.corr_y + t_fit * dr_ransac.corr_vy
                );
            }
			{
			  float gate = (float) dr_fp.gate_nr;
			  float assigned = (float) dr_state.assigned_gate_index;
	          fprintf(fp,"%d,%f,%f,%f,%f,%f,%f,%f\n", -1 , gate, assigned,  params_x[0], params_x[1], params_y[0], params_y[1] ,0.0f);
			}

            // fprintf(fp,"\n\n X = %f %f Y = %f  %f \n",params_x[0], params_x[1], params_y[0], params_y[1] );
            fclose(fp);
        }
#endif

    }
}


