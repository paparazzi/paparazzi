
#define VISION_LATENCY_TIME_STEPS 2 ///< Time steps: note: must be at least 1


extern void fifo_reset(void);
extern void fifo_push(float x, float y, float z);
extern void fifo_pop(float *x, float *y, float *z);
