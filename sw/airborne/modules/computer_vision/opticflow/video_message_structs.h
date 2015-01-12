#ifndef VMSTRTS_H
#define VMSTRTS_H

/***
      an exact copy of this file to exist in ppz
      this files keeps the structs that are serialized and streamed over tcp/ip through localhost
 */

#define N_BINS  10

struct gst2ppz_message_struct {
  unsigned int ID;                // Keep different modules for using each others data
  unsigned int counter;           // counter to keep track of data
  unsigned int obstacle_bins[N_BINS];     // optical flow output, shift in x direction
  unsigned int uncertainty_bins[N_BINS];    //optical flow output, shift in y direction
};
extern struct gst2ppz_message_struct gst2ppz;

struct ppz2gst_message_struct {
  unsigned int ID;                // Keep different modules for using each others data
  unsigned int counter;           //counter to keep track of data
  int pitch;
  int roll;
  int alt;
  int adjust_factor;              // 0-10 :adjust brightness
};
extern struct ppz2gst_message_struct ppz2gst;

#endif  /*  VMSTRTS_H  */

