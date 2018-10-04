#include "fifo.h"

// VISION LATENCY COMPENSATION

struct dronerace_vision_struct
{
  float x[VISION_LATENCY_TIME_STEPS];
  float y[VISION_LATENCY_TIME_STEPS];
  float z[VISION_LATENCY_TIME_STEPS];
  int index;
};

// Variable
struct dronerace_vision_struct dr_past_state;

// Reset
void fifo_reset(void)
{
  int i;
  for (i=0; i<VISION_LATENCY_TIME_STEPS;i++) {
    dr_past_state.x[i] = 0;
    dr_past_state.y[i] = 0;
    dr_past_state.z[i] = 0;
  }
  dr_past_state.index = 0;
}

// Add New
void fifo_push(float x, float y, float z __attribute__((unused)))
{
  dr_past_state.index++;
  if (dr_past_state.index >= VISION_LATENCY_TIME_STEPS) {
    dr_past_state.index = 0;
  }
  dr_past_state.x[dr_past_state.index] = x;
  dr_past_state.y[dr_past_state.index] = y;

}

// Retrieve Oldest
void fifo_pop(float *x, float *y, float *z)
{
  int index = dr_past_state.index + 1;
  if (index >= VISION_LATENCY_TIME_STEPS) {
    index = 0;
  }
  *x = dr_past_state.x[index];
  *y = dr_past_state.y[index];
  *z = dr_past_state.z[index];
}
