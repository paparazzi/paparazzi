

#ifndef VIDEO_DEVICE_H
#define VIDEO_DEVICE_H
#include <stdbool.h>
#include <inttypes.h>
#define VIDEO_FILTER_DEBAYER 0x01

typedef struct image_t *(*cvFunction)(struct image_t *img);

struct video_listener {
  struct video_listener *next;
  cvFunction func;
};


// Main video_thread structure
struct video_thread_t {
  volatile bool is_running;   ///< When the device is running
  struct v4l2_device *dev;        ///< The V4L2 device that is used for the video stream
  uint8_t fps;                    ///< The amount of frames per second

  volatile bool take_shot;      ///< Wether to take an image
  uint16_t shot_number;           ///< The last shot number
};

/** V4L2 device settings */
struct video_config_t {
  int w;              ///< Width
  int h;              ///< Height
  char *dev_name;     ///< path to device
  char *subdev_name;  ///< path to sub device
  uint32_t format;    ///< Video format
  uint8_t buf_cnt;    ///< Amount of V4L2 video device buffers
  uint8_t filters;    ///< filters to use (bitfield with VIDEO_FILTER_x)
  struct video_thread_t thread; ///< Information about the thread this camera is running on
  struct video_listener *pointer_to_first_listener; ///< The first listener in the linked list for this video device
};
#endif
