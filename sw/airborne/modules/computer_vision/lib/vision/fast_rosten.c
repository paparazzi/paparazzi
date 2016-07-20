/*
Copyright (c) 2006, 2008 Edward Rosten
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:


  *Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

  *Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  *Neither the name of the University of Cambridge nor the names of
   its contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include "fast_rosten.h"

static void fast_make_offsets(int32_t *pixel, uint16_t row_stride, uint8_t pixel_size);

/**
 * Do a FAST9 corner detection. The array *ret_corners can be reallocated in this function every time
 * it becomes too full, *ret_corners_length is updated appropriately.
 * @param[in] *img The image to do the corner detection on
 * @param[in] threshold The threshold which we use for FAST9
 * @param[in] min_dist The minimum distance in pixels between detections
 * @param[in] x_padding The padding in the x direction to not scan for corners
 * @param[in] y_padding The padding in the y direction to not scan for corners
 * @param[in] *num_corners reference to the amount of corners found, set by this function
 * @param[in] *ret_corners_length the length of the array *ret_corners.
 * @param[in] *ret_corners array which contains the corners that were detected.
*/
void fast9_detect(struct image_t *img, uint8_t threshold, uint16_t min_dist, uint16_t x_padding, uint16_t y_padding, uint16_t *num_corners, uint16_t *ret_corners_length,struct point_t *ret_corners) {
  uint32_t corner_cnt = 0;

  int pixel[16];
  int16_t i;
  uint16_t x, y, x_min, x_max, y_min;
  uint8_t need_skip;
  // Set the pixel size
  uint8_t pixel_size = 1;
  if (img->type == IMAGE_YUV422) {
    pixel_size = 2;
  }

  // Calculate the pixel offsets
  fast_make_offsets(pixel, img->w, pixel_size);

  // Go trough all the pixels (minus the borders)
  for (y = 3 + y_padding; y < img->h - 3 - y_padding; y++) {

    if (min_dist > 0) y_min = y - min_dist;

    for (x = 3 + x_padding; x < img->w - 3 - x_padding; x++) {
      // First check if we aren't in range vertical (TODO: fix less intensive way)
      if (min_dist > 0) {

        need_skip = 0;

        x_min = x - min_dist;
        x_max = x + min_dist;


        // Go through the previous corners until y goes out of range
        i = corner_cnt-1;
        while( i >= 0) {

          // corners are stored with increasing y,
          // so if we go from the last to the first, then their y-coordinate will go out of range
          if(ret_corners[i].y < y_min)
            break;

          if (x_min < ret_corners[i].x && ret_corners[i].x < x_max) {
            need_skip = 1;
            break;
          }

          i--;
        }

        // Skip the box if we found a pixel nearby
        if (need_skip) {
          x += min_dist;
          continue;
        }
      }

      // Calculate the threshold values
      const uint8_t *p = ((uint8_t *)img->buf) + y * img->w * pixel_size + x * pixel_size + pixel_size / 2;
      int16_t cb = *p + threshold;
      int16_t c_b = *p - threshold;

      // Do the checks if it is a corner
      if (p[pixel[0]] > cb)
        if (p[pixel[1]] > cb)
          if (p[pixel[2]] > cb)
            if (p[pixel[3]] > cb)
              if (p[pixel[4]] > cb)
                if (p[pixel[5]] > cb)
                  if (p[pixel[6]] > cb)
                    if (p[pixel[7]] > cb)
                      if (p[pixel[8]] > cb)
                        {}
                      else if (p[pixel[15]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else if (p[pixel[7]] < c_b)
                      if (p[pixel[14]] > cb)
                        if (p[pixel[15]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else if (p[pixel[14]] < c_b)
                        if (p[pixel[8]] < c_b)
                          if (p[pixel[9]] < c_b)
                            if (p[pixel[10]] < c_b)
                              if (p[pixel[11]] < c_b)
                                if (p[pixel[12]] < c_b)
                                  if (p[pixel[13]] < c_b)
                                    if (p[pixel[15]] < c_b)
                                      {}
                                    else {
                                      continue;
                                    }
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[14]] > cb)
                      if (p[pixel[15]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[6]] < c_b)
                    if (p[pixel[15]] > cb)
                      if (p[pixel[13]] > cb)
                        if (p[pixel[14]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else if (p[pixel[13]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              if (p[pixel[10]] < c_b)
                                if (p[pixel[11]] < c_b)
                                  if (p[pixel[12]] < c_b)
                                    if (p[pixel[14]] < c_b)
                                      {}
                                    else {
                                      continue;
                                    }
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[7]] < c_b)
                      if (p[pixel[8]] < c_b)
                        if (p[pixel[9]] < c_b)
                          if (p[pixel[10]] < c_b)
                            if (p[pixel[11]] < c_b)
                              if (p[pixel[12]] < c_b)
                                if (p[pixel[13]] < c_b)
                                  if (p[pixel[14]] < c_b)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[13]] > cb)
                    if (p[pixel[14]] > cb)
                      if (p[pixel[15]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[13]] < c_b)
                    if (p[pixel[7]] < c_b)
                      if (p[pixel[8]] < c_b)
                        if (p[pixel[9]] < c_b)
                          if (p[pixel[10]] < c_b)
                            if (p[pixel[11]] < c_b)
                              if (p[pixel[12]] < c_b)
                                if (p[pixel[14]] < c_b)
                                  if (p[pixel[15]] < c_b)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[5]] < c_b)
                  if (p[pixel[14]] > cb)
                    if (p[pixel[12]] > cb)
                      if (p[pixel[13]] > cb)
                        if (p[pixel[15]] > cb)
                          {}
                        else if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              if (p[pixel[9]] > cb)
                                if (p[pixel[10]] > cb)
                                  if (p[pixel[11]] > cb)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[12]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              if (p[pixel[10]] < c_b)
                                if (p[pixel[11]] < c_b)
                                  if (p[pixel[13]] < c_b)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[14]] < c_b)
                    if (p[pixel[7]] < c_b)
                      if (p[pixel[8]] < c_b)
                        if (p[pixel[9]] < c_b)
                          if (p[pixel[10]] < c_b)
                            if (p[pixel[11]] < c_b)
                              if (p[pixel[12]] < c_b)
                                if (p[pixel[13]] < c_b)
                                  if (p[pixel[6]] < c_b)
                                    {}
                                  else if (p[pixel[15]] < c_b)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[6]] < c_b)
                    if (p[pixel[7]] < c_b)
                      if (p[pixel[8]] < c_b)
                        if (p[pixel[9]] < c_b)
                          if (p[pixel[10]] < c_b)
                            if (p[pixel[11]] < c_b)
                              if (p[pixel[12]] < c_b)
                                if (p[pixel[13]] < c_b)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[12]] > cb)
                  if (p[pixel[13]] > cb)
                    if (p[pixel[14]] > cb)
                      if (p[pixel[15]] > cb)
                        {}
                      else if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              if (p[pixel[10]] > cb)
                                if (p[pixel[11]] > cb)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[12]] < c_b)
                  if (p[pixel[7]] < c_b)
                    if (p[pixel[8]] < c_b)
                      if (p[pixel[9]] < c_b)
                        if (p[pixel[10]] < c_b)
                          if (p[pixel[11]] < c_b)
                            if (p[pixel[13]] < c_b)
                              if (p[pixel[14]] < c_b)
                                if (p[pixel[6]] < c_b)
                                  {}
                                else if (p[pixel[15]] < c_b)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[4]] < c_b)
                if (p[pixel[13]] > cb)
                  if (p[pixel[11]] > cb)
                    if (p[pixel[12]] > cb)
                      if (p[pixel[14]] > cb)
                        if (p[pixel[15]] > cb)
                          {}
                        else if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              if (p[pixel[9]] > cb)
                                if (p[pixel[10]] > cb)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              if (p[pixel[9]] > cb)
                                if (p[pixel[10]] > cb)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[11]] < c_b)
                    if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              if (p[pixel[10]] < c_b)
                                if (p[pixel[12]] < c_b)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[13]] < c_b)
                  if (p[pixel[7]] < c_b)
                    if (p[pixel[8]] < c_b)
                      if (p[pixel[9]] < c_b)
                        if (p[pixel[10]] < c_b)
                          if (p[pixel[11]] < c_b)
                            if (p[pixel[12]] < c_b)
                              if (p[pixel[6]] < c_b)
                                if (p[pixel[5]] < c_b)
                                  {}
                                else if (p[pixel[14]] < c_b)
                                  {}
                                else {
                                  continue;
                                }
                              else if (p[pixel[14]] < c_b)
                                if (p[pixel[15]] < c_b)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[5]] < c_b)
                  if (p[pixel[6]] < c_b)
                    if (p[pixel[7]] < c_b)
                      if (p[pixel[8]] < c_b)
                        if (p[pixel[9]] < c_b)
                          if (p[pixel[10]] < c_b)
                            if (p[pixel[11]] < c_b)
                              if (p[pixel[12]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[11]] > cb)
                if (p[pixel[12]] > cb)
                  if (p[pixel[13]] > cb)
                    if (p[pixel[14]] > cb)
                      if (p[pixel[15]] > cb)
                        {}
                      else if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              if (p[pixel[10]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              if (p[pixel[10]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[11]] < c_b)
                if (p[pixel[7]] < c_b)
                  if (p[pixel[8]] < c_b)
                    if (p[pixel[9]] < c_b)
                      if (p[pixel[10]] < c_b)
                        if (p[pixel[12]] < c_b)
                          if (p[pixel[13]] < c_b)
                            if (p[pixel[6]] < c_b)
                              if (p[pixel[5]] < c_b)
                                {}
                              else if (p[pixel[14]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else if (p[pixel[14]] < c_b)
                              if (p[pixel[15]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[3]] < c_b)
              if (p[pixel[10]] > cb)
                if (p[pixel[11]] > cb)
                  if (p[pixel[12]] > cb)
                    if (p[pixel[13]] > cb)
                      if (p[pixel[14]] > cb)
                        if (p[pixel[15]] > cb)
                          {}
                        else if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              if (p[pixel[9]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              if (p[pixel[9]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[4]] > cb)
                      if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              if (p[pixel[9]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[10]] < c_b)
                if (p[pixel[7]] < c_b)
                  if (p[pixel[8]] < c_b)
                    if (p[pixel[9]] < c_b)
                      if (p[pixel[11]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[5]] < c_b)
                            if (p[pixel[4]] < c_b)
                              {}
                            else if (p[pixel[12]] < c_b)
                              if (p[pixel[13]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else if (p[pixel[12]] < c_b)
                            if (p[pixel[13]] < c_b)
                              if (p[pixel[14]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else if (p[pixel[12]] < c_b)
                          if (p[pixel[13]] < c_b)
                            if (p[pixel[14]] < c_b)
                              if (p[pixel[15]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[10]] > cb)
              if (p[pixel[11]] > cb)
                if (p[pixel[12]] > cb)
                  if (p[pixel[13]] > cb)
                    if (p[pixel[14]] > cb)
                      if (p[pixel[15]] > cb)
                        {}
                      else if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[4]] > cb)
                    if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[10]] < c_b)
              if (p[pixel[7]] < c_b)
                if (p[pixel[8]] < c_b)
                  if (p[pixel[9]] < c_b)
                    if (p[pixel[11]] < c_b)
                      if (p[pixel[12]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[5]] < c_b)
                            if (p[pixel[4]] < c_b)
                              {}
                            else if (p[pixel[13]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else if (p[pixel[13]] < c_b)
                            if (p[pixel[14]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else if (p[pixel[13]] < c_b)
                          if (p[pixel[14]] < c_b)
                            if (p[pixel[15]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else if (p[pixel[2]] < c_b)
            if (p[pixel[9]] > cb)
              if (p[pixel[10]] > cb)
                if (p[pixel[11]] > cb)
                  if (p[pixel[12]] > cb)
                    if (p[pixel[13]] > cb)
                      if (p[pixel[14]] > cb)
                        if (p[pixel[15]] > cb)
                          {}
                        else if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[4]] > cb)
                      if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[3]] > cb)
                    if (p[pixel[4]] > cb)
                      if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            if (p[pixel[8]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[9]] < c_b)
              if (p[pixel[7]] < c_b)
                if (p[pixel[8]] < c_b)
                  if (p[pixel[10]] < c_b)
                    if (p[pixel[6]] < c_b)
                      if (p[pixel[5]] < c_b)
                        if (p[pixel[4]] < c_b)
                          if (p[pixel[3]] < c_b)
                            {}
                          else if (p[pixel[11]] < c_b)
                            if (p[pixel[12]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else if (p[pixel[11]] < c_b)
                          if (p[pixel[12]] < c_b)
                            if (p[pixel[13]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[11]] < c_b)
                        if (p[pixel[12]] < c_b)
                          if (p[pixel[13]] < c_b)
                            if (p[pixel[14]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[11]] < c_b)
                      if (p[pixel[12]] < c_b)
                        if (p[pixel[13]] < c_b)
                          if (p[pixel[14]] < c_b)
                            if (p[pixel[15]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else if (p[pixel[9]] > cb)
            if (p[pixel[10]] > cb)
              if (p[pixel[11]] > cb)
                if (p[pixel[12]] > cb)
                  if (p[pixel[13]] > cb)
                    if (p[pixel[14]] > cb)
                      if (p[pixel[15]] > cb)
                        {}
                      else if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[4]] > cb)
                    if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[3]] > cb)
                  if (p[pixel[4]] > cb)
                    if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else if (p[pixel[9]] < c_b)
            if (p[pixel[7]] < c_b)
              if (p[pixel[8]] < c_b)
                if (p[pixel[10]] < c_b)
                  if (p[pixel[11]] < c_b)
                    if (p[pixel[6]] < c_b)
                      if (p[pixel[5]] < c_b)
                        if (p[pixel[4]] < c_b)
                          if (p[pixel[3]] < c_b)
                            {}
                          else if (p[pixel[12]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else if (p[pixel[12]] < c_b)
                          if (p[pixel[13]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[12]] < c_b)
                        if (p[pixel[13]] < c_b)
                          if (p[pixel[14]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[12]] < c_b)
                      if (p[pixel[13]] < c_b)
                        if (p[pixel[14]] < c_b)
                          if (p[pixel[15]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else if (p[pixel[1]] < c_b)
          if (p[pixel[8]] > cb)
            if (p[pixel[9]] > cb)
              if (p[pixel[10]] > cb)
                if (p[pixel[11]] > cb)
                  if (p[pixel[12]] > cb)
                    if (p[pixel[13]] > cb)
                      if (p[pixel[14]] > cb)
                        if (p[pixel[15]] > cb)
                          {}
                        else if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[4]] > cb)
                      if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[3]] > cb)
                    if (p[pixel[4]] > cb)
                      if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[2]] > cb)
                  if (p[pixel[3]] > cb)
                    if (p[pixel[4]] > cb)
                      if (p[pixel[5]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[7]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else if (p[pixel[8]] < c_b)
            if (p[pixel[7]] < c_b)
              if (p[pixel[9]] < c_b)
                if (p[pixel[6]] < c_b)
                  if (p[pixel[5]] < c_b)
                    if (p[pixel[4]] < c_b)
                      if (p[pixel[3]] < c_b)
                        if (p[pixel[2]] < c_b)
                          {}
                        else if (p[pixel[10]] < c_b)
                          if (p[pixel[11]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[10]] < c_b)
                        if (p[pixel[11]] < c_b)
                          if (p[pixel[12]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[10]] < c_b)
                      if (p[pixel[11]] < c_b)
                        if (p[pixel[12]] < c_b)
                          if (p[pixel[13]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[10]] < c_b)
                    if (p[pixel[11]] < c_b)
                      if (p[pixel[12]] < c_b)
                        if (p[pixel[13]] < c_b)
                          if (p[pixel[14]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[10]] < c_b)
                  if (p[pixel[11]] < c_b)
                    if (p[pixel[12]] < c_b)
                      if (p[pixel[13]] < c_b)
                        if (p[pixel[14]] < c_b)
                          if (p[pixel[15]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else if (p[pixel[8]] > cb)
          if (p[pixel[9]] > cb)
            if (p[pixel[10]] > cb)
              if (p[pixel[11]] > cb)
                if (p[pixel[12]] > cb)
                  if (p[pixel[13]] > cb)
                    if (p[pixel[14]] > cb)
                      if (p[pixel[15]] > cb)
                        {}
                      else if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[4]] > cb)
                    if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[3]] > cb)
                  if (p[pixel[4]] > cb)
                    if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[2]] > cb)
                if (p[pixel[3]] > cb)
                  if (p[pixel[4]] > cb)
                    if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else if (p[pixel[8]] < c_b)
          if (p[pixel[7]] < c_b)
            if (p[pixel[9]] < c_b)
              if (p[pixel[10]] < c_b)
                if (p[pixel[6]] < c_b)
                  if (p[pixel[5]] < c_b)
                    if (p[pixel[4]] < c_b)
                      if (p[pixel[3]] < c_b)
                        if (p[pixel[2]] < c_b)
                          {}
                        else if (p[pixel[11]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else if (p[pixel[11]] < c_b)
                        if (p[pixel[12]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[11]] < c_b)
                      if (p[pixel[12]] < c_b)
                        if (p[pixel[13]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[11]] < c_b)
                    if (p[pixel[12]] < c_b)
                      if (p[pixel[13]] < c_b)
                        if (p[pixel[14]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[11]] < c_b)
                  if (p[pixel[12]] < c_b)
                    if (p[pixel[13]] < c_b)
                      if (p[pixel[14]] < c_b)
                        if (p[pixel[15]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else {
          continue;
        }
      else if (p[pixel[0]] < c_b)
        if (p[pixel[1]] > cb)
          if (p[pixel[8]] > cb)
            if (p[pixel[7]] > cb)
              if (p[pixel[9]] > cb)
                if (p[pixel[6]] > cb)
                  if (p[pixel[5]] > cb)
                    if (p[pixel[4]] > cb)
                      if (p[pixel[3]] > cb)
                        if (p[pixel[2]] > cb)
                          {}
                        else if (p[pixel[10]] > cb)
                          if (p[pixel[11]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[10]] > cb)
                        if (p[pixel[11]] > cb)
                          if (p[pixel[12]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[10]] > cb)
                      if (p[pixel[11]] > cb)
                        if (p[pixel[12]] > cb)
                          if (p[pixel[13]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[10]] > cb)
                    if (p[pixel[11]] > cb)
                      if (p[pixel[12]] > cb)
                        if (p[pixel[13]] > cb)
                          if (p[pixel[14]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[10]] > cb)
                  if (p[pixel[11]] > cb)
                    if (p[pixel[12]] > cb)
                      if (p[pixel[13]] > cb)
                        if (p[pixel[14]] > cb)
                          if (p[pixel[15]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else if (p[pixel[8]] < c_b)
            if (p[pixel[9]] < c_b)
              if (p[pixel[10]] < c_b)
                if (p[pixel[11]] < c_b)
                  if (p[pixel[12]] < c_b)
                    if (p[pixel[13]] < c_b)
                      if (p[pixel[14]] < c_b)
                        if (p[pixel[15]] < c_b)
                          {}
                        else if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[4]] < c_b)
                      if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[3]] < c_b)
                    if (p[pixel[4]] < c_b)
                      if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[2]] < c_b)
                  if (p[pixel[3]] < c_b)
                    if (p[pixel[4]] < c_b)
                      if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else if (p[pixel[1]] < c_b)
          if (p[pixel[2]] > cb)
            if (p[pixel[9]] > cb)
              if (p[pixel[7]] > cb)
                if (p[pixel[8]] > cb)
                  if (p[pixel[10]] > cb)
                    if (p[pixel[6]] > cb)
                      if (p[pixel[5]] > cb)
                        if (p[pixel[4]] > cb)
                          if (p[pixel[3]] > cb)
                            {}
                          else if (p[pixel[11]] > cb)
                            if (p[pixel[12]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else if (p[pixel[11]] > cb)
                          if (p[pixel[12]] > cb)
                            if (p[pixel[13]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[11]] > cb)
                        if (p[pixel[12]] > cb)
                          if (p[pixel[13]] > cb)
                            if (p[pixel[14]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[11]] > cb)
                      if (p[pixel[12]] > cb)
                        if (p[pixel[13]] > cb)
                          if (p[pixel[14]] > cb)
                            if (p[pixel[15]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[9]] < c_b)
              if (p[pixel[10]] < c_b)
                if (p[pixel[11]] < c_b)
                  if (p[pixel[12]] < c_b)
                    if (p[pixel[13]] < c_b)
                      if (p[pixel[14]] < c_b)
                        if (p[pixel[15]] < c_b)
                          {}
                        else if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[4]] < c_b)
                      if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[3]] < c_b)
                    if (p[pixel[4]] < c_b)
                      if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else if (p[pixel[2]] < c_b)
            if (p[pixel[3]] > cb)
              if (p[pixel[10]] > cb)
                if (p[pixel[7]] > cb)
                  if (p[pixel[8]] > cb)
                    if (p[pixel[9]] > cb)
                      if (p[pixel[11]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[5]] > cb)
                            if (p[pixel[4]] > cb)
                              {}
                            else if (p[pixel[12]] > cb)
                              if (p[pixel[13]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else if (p[pixel[12]] > cb)
                            if (p[pixel[13]] > cb)
                              if (p[pixel[14]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else if (p[pixel[12]] > cb)
                          if (p[pixel[13]] > cb)
                            if (p[pixel[14]] > cb)
                              if (p[pixel[15]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[10]] < c_b)
                if (p[pixel[11]] < c_b)
                  if (p[pixel[12]] < c_b)
                    if (p[pixel[13]] < c_b)
                      if (p[pixel[14]] < c_b)
                        if (p[pixel[15]] < c_b)
                          {}
                        else if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              if (p[pixel[9]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              if (p[pixel[9]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[4]] < c_b)
                      if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              if (p[pixel[9]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[3]] < c_b)
              if (p[pixel[4]] > cb)
                if (p[pixel[13]] > cb)
                  if (p[pixel[7]] > cb)
                    if (p[pixel[8]] > cb)
                      if (p[pixel[9]] > cb)
                        if (p[pixel[10]] > cb)
                          if (p[pixel[11]] > cb)
                            if (p[pixel[12]] > cb)
                              if (p[pixel[6]] > cb)
                                if (p[pixel[5]] > cb)
                                  {}
                                else if (p[pixel[14]] > cb)
                                  {}
                                else {
                                  continue;
                                }
                              else if (p[pixel[14]] > cb)
                                if (p[pixel[15]] > cb)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[13]] < c_b)
                  if (p[pixel[11]] > cb)
                    if (p[pixel[5]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              if (p[pixel[10]] > cb)
                                if (p[pixel[12]] > cb)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[11]] < c_b)
                    if (p[pixel[12]] < c_b)
                      if (p[pixel[14]] < c_b)
                        if (p[pixel[15]] < c_b)
                          {}
                        else if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              if (p[pixel[9]] < c_b)
                                if (p[pixel[10]] < c_b)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[5]] < c_b)
                        if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              if (p[pixel[9]] < c_b)
                                if (p[pixel[10]] < c_b)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[5]] > cb)
                  if (p[pixel[6]] > cb)
                    if (p[pixel[7]] > cb)
                      if (p[pixel[8]] > cb)
                        if (p[pixel[9]] > cb)
                          if (p[pixel[10]] > cb)
                            if (p[pixel[11]] > cb)
                              if (p[pixel[12]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[4]] < c_b)
                if (p[pixel[5]] > cb)
                  if (p[pixel[14]] > cb)
                    if (p[pixel[7]] > cb)
                      if (p[pixel[8]] > cb)
                        if (p[pixel[9]] > cb)
                          if (p[pixel[10]] > cb)
                            if (p[pixel[11]] > cb)
                              if (p[pixel[12]] > cb)
                                if (p[pixel[13]] > cb)
                                  if (p[pixel[6]] > cb)
                                    {}
                                  else if (p[pixel[15]] > cb)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[14]] < c_b)
                    if (p[pixel[12]] > cb)
                      if (p[pixel[6]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              if (p[pixel[10]] > cb)
                                if (p[pixel[11]] > cb)
                                  if (p[pixel[13]] > cb)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[12]] < c_b)
                      if (p[pixel[13]] < c_b)
                        if (p[pixel[15]] < c_b)
                          {}
                        else if (p[pixel[6]] < c_b)
                          if (p[pixel[7]] < c_b)
                            if (p[pixel[8]] < c_b)
                              if (p[pixel[9]] < c_b)
                                if (p[pixel[10]] < c_b)
                                  if (p[pixel[11]] < c_b)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[6]] > cb)
                    if (p[pixel[7]] > cb)
                      if (p[pixel[8]] > cb)
                        if (p[pixel[9]] > cb)
                          if (p[pixel[10]] > cb)
                            if (p[pixel[11]] > cb)
                              if (p[pixel[12]] > cb)
                                if (p[pixel[13]] > cb)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[5]] < c_b)
                  if (p[pixel[6]] > cb)
                    if (p[pixel[15]] < c_b)
                      if (p[pixel[13]] > cb)
                        if (p[pixel[7]] > cb)
                          if (p[pixel[8]] > cb)
                            if (p[pixel[9]] > cb)
                              if (p[pixel[10]] > cb)
                                if (p[pixel[11]] > cb)
                                  if (p[pixel[12]] > cb)
                                    if (p[pixel[14]] > cb)
                                      {}
                                    else {
                                      continue;
                                    }
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[13]] < c_b)
                        if (p[pixel[14]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[7]] > cb)
                      if (p[pixel[8]] > cb)
                        if (p[pixel[9]] > cb)
                          if (p[pixel[10]] > cb)
                            if (p[pixel[11]] > cb)
                              if (p[pixel[12]] > cb)
                                if (p[pixel[13]] > cb)
                                  if (p[pixel[14]] > cb)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[6]] < c_b)
                    if (p[pixel[7]] > cb)
                      if (p[pixel[14]] > cb)
                        if (p[pixel[8]] > cb)
                          if (p[pixel[9]] > cb)
                            if (p[pixel[10]] > cb)
                              if (p[pixel[11]] > cb)
                                if (p[pixel[12]] > cb)
                                  if (p[pixel[13]] > cb)
                                    if (p[pixel[15]] > cb)
                                      {}
                                    else {
                                      continue;
                                    }
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[14]] < c_b)
                        if (p[pixel[15]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[7]] < c_b)
                      if (p[pixel[8]] < c_b)
                        {}
                      else if (p[pixel[15]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else if (p[pixel[14]] < c_b)
                      if (p[pixel[15]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[13]] > cb)
                    if (p[pixel[7]] > cb)
                      if (p[pixel[8]] > cb)
                        if (p[pixel[9]] > cb)
                          if (p[pixel[10]] > cb)
                            if (p[pixel[11]] > cb)
                              if (p[pixel[12]] > cb)
                                if (p[pixel[14]] > cb)
                                  if (p[pixel[15]] > cb)
                                    {}
                                  else {
                                    continue;
                                  }
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[13]] < c_b)
                    if (p[pixel[14]] < c_b)
                      if (p[pixel[15]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[12]] > cb)
                  if (p[pixel[7]] > cb)
                    if (p[pixel[8]] > cb)
                      if (p[pixel[9]] > cb)
                        if (p[pixel[10]] > cb)
                          if (p[pixel[11]] > cb)
                            if (p[pixel[13]] > cb)
                              if (p[pixel[14]] > cb)
                                if (p[pixel[6]] > cb)
                                  {}
                                else if (p[pixel[15]] > cb)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[12]] < c_b)
                  if (p[pixel[13]] < c_b)
                    if (p[pixel[14]] < c_b)
                      if (p[pixel[15]] < c_b)
                        {}
                      else if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              if (p[pixel[10]] < c_b)
                                if (p[pixel[11]] < c_b)
                                  {}
                                else {
                                  continue;
                                }
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[11]] > cb)
                if (p[pixel[7]] > cb)
                  if (p[pixel[8]] > cb)
                    if (p[pixel[9]] > cb)
                      if (p[pixel[10]] > cb)
                        if (p[pixel[12]] > cb)
                          if (p[pixel[13]] > cb)
                            if (p[pixel[6]] > cb)
                              if (p[pixel[5]] > cb)
                                {}
                              else if (p[pixel[14]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else if (p[pixel[14]] > cb)
                              if (p[pixel[15]] > cb)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[11]] < c_b)
                if (p[pixel[12]] < c_b)
                  if (p[pixel[13]] < c_b)
                    if (p[pixel[14]] < c_b)
                      if (p[pixel[15]] < c_b)
                        {}
                      else if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              if (p[pixel[10]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              if (p[pixel[10]] < c_b)
                                {}
                              else {
                                continue;
                              }
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[10]] > cb)
              if (p[pixel[7]] > cb)
                if (p[pixel[8]] > cb)
                  if (p[pixel[9]] > cb)
                    if (p[pixel[11]] > cb)
                      if (p[pixel[12]] > cb)
                        if (p[pixel[6]] > cb)
                          if (p[pixel[5]] > cb)
                            if (p[pixel[4]] > cb)
                              {}
                            else if (p[pixel[13]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else if (p[pixel[13]] > cb)
                            if (p[pixel[14]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else if (p[pixel[13]] > cb)
                          if (p[pixel[14]] > cb)
                            if (p[pixel[15]] > cb)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[10]] < c_b)
              if (p[pixel[11]] < c_b)
                if (p[pixel[12]] < c_b)
                  if (p[pixel[13]] < c_b)
                    if (p[pixel[14]] < c_b)
                      if (p[pixel[15]] < c_b)
                        {}
                      else if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[4]] < c_b)
                    if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            if (p[pixel[9]] < c_b)
                              {}
                            else {
                              continue;
                            }
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else if (p[pixel[9]] > cb)
            if (p[pixel[7]] > cb)
              if (p[pixel[8]] > cb)
                if (p[pixel[10]] > cb)
                  if (p[pixel[11]] > cb)
                    if (p[pixel[6]] > cb)
                      if (p[pixel[5]] > cb)
                        if (p[pixel[4]] > cb)
                          if (p[pixel[3]] > cb)
                            {}
                          else if (p[pixel[12]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else if (p[pixel[12]] > cb)
                          if (p[pixel[13]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else if (p[pixel[12]] > cb)
                        if (p[pixel[13]] > cb)
                          if (p[pixel[14]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[12]] > cb)
                      if (p[pixel[13]] > cb)
                        if (p[pixel[14]] > cb)
                          if (p[pixel[15]] > cb)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else if (p[pixel[9]] < c_b)
            if (p[pixel[10]] < c_b)
              if (p[pixel[11]] < c_b)
                if (p[pixel[12]] < c_b)
                  if (p[pixel[13]] < c_b)
                    if (p[pixel[14]] < c_b)
                      if (p[pixel[15]] < c_b)
                        {}
                      else if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[4]] < c_b)
                    if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[3]] < c_b)
                  if (p[pixel[4]] < c_b)
                    if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          if (p[pixel[8]] < c_b)
                            {}
                          else {
                            continue;
                          }
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else if (p[pixel[8]] > cb)
          if (p[pixel[7]] > cb)
            if (p[pixel[9]] > cb)
              if (p[pixel[10]] > cb)
                if (p[pixel[6]] > cb)
                  if (p[pixel[5]] > cb)
                    if (p[pixel[4]] > cb)
                      if (p[pixel[3]] > cb)
                        if (p[pixel[2]] > cb)
                          {}
                        else if (p[pixel[11]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else if (p[pixel[11]] > cb)
                        if (p[pixel[12]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[11]] > cb)
                      if (p[pixel[12]] > cb)
                        if (p[pixel[13]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[11]] > cb)
                    if (p[pixel[12]] > cb)
                      if (p[pixel[13]] > cb)
                        if (p[pixel[14]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[11]] > cb)
                  if (p[pixel[12]] > cb)
                    if (p[pixel[13]] > cb)
                      if (p[pixel[14]] > cb)
                        if (p[pixel[15]] > cb)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else if (p[pixel[8]] < c_b)
          if (p[pixel[9]] < c_b)
            if (p[pixel[10]] < c_b)
              if (p[pixel[11]] < c_b)
                if (p[pixel[12]] < c_b)
                  if (p[pixel[13]] < c_b)
                    if (p[pixel[14]] < c_b)
                      if (p[pixel[15]] < c_b)
                        {}
                      else if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[4]] < c_b)
                    if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[3]] < c_b)
                  if (p[pixel[4]] < c_b)
                    if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[2]] < c_b)
                if (p[pixel[3]] < c_b)
                  if (p[pixel[4]] < c_b)
                    if (p[pixel[5]] < c_b)
                      if (p[pixel[6]] < c_b)
                        if (p[pixel[7]] < c_b)
                          {}
                        else {
                          continue;
                        }
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else {
          continue;
        }
      else if (p[pixel[7]] > cb)
        if (p[pixel[8]] > cb)
          if (p[pixel[9]] > cb)
            if (p[pixel[6]] > cb)
              if (p[pixel[5]] > cb)
                if (p[pixel[4]] > cb)
                  if (p[pixel[3]] > cb)
                    if (p[pixel[2]] > cb)
                      if (p[pixel[1]] > cb)
                        {}
                      else if (p[pixel[10]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else if (p[pixel[10]] > cb)
                      if (p[pixel[11]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[10]] > cb)
                    if (p[pixel[11]] > cb)
                      if (p[pixel[12]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[10]] > cb)
                  if (p[pixel[11]] > cb)
                    if (p[pixel[12]] > cb)
                      if (p[pixel[13]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[10]] > cb)
                if (p[pixel[11]] > cb)
                  if (p[pixel[12]] > cb)
                    if (p[pixel[13]] > cb)
                      if (p[pixel[14]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[10]] > cb)
              if (p[pixel[11]] > cb)
                if (p[pixel[12]] > cb)
                  if (p[pixel[13]] > cb)
                    if (p[pixel[14]] > cb)
                      if (p[pixel[15]] > cb)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else {
          continue;
        }
      else if (p[pixel[7]] < c_b)
        if (p[pixel[8]] < c_b)
          if (p[pixel[9]] < c_b)
            if (p[pixel[6]] < c_b)
              if (p[pixel[5]] < c_b)
                if (p[pixel[4]] < c_b)
                  if (p[pixel[3]] < c_b)
                    if (p[pixel[2]] < c_b)
                      if (p[pixel[1]] < c_b)
                        {}
                      else if (p[pixel[10]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else if (p[pixel[10]] < c_b)
                      if (p[pixel[11]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else if (p[pixel[10]] < c_b)
                    if (p[pixel[11]] < c_b)
                      if (p[pixel[12]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else if (p[pixel[10]] < c_b)
                  if (p[pixel[11]] < c_b)
                    if (p[pixel[12]] < c_b)
                      if (p[pixel[13]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else if (p[pixel[10]] < c_b)
                if (p[pixel[11]] < c_b)
                  if (p[pixel[12]] < c_b)
                    if (p[pixel[13]] < c_b)
                      if (p[pixel[14]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else if (p[pixel[10]] < c_b)
              if (p[pixel[11]] < c_b)
                if (p[pixel[12]] < c_b)
                  if (p[pixel[13]] < c_b)
                    if (p[pixel[14]] < c_b)
                      if (p[pixel[15]] < c_b)
                        {}
                      else {
                        continue;
                      }
                    else {
                      continue;
                    }
                  else {
                    continue;
                  }
                else {
                  continue;
                }
              else {
                continue;
              }
            else {
              continue;
            }
          else {
            continue;
          }
        else {
          continue;
        }
      else {
        continue;
      }

      // When we have more corner than allocted space reallocate
      if (corner_cnt >= *ret_corners_length) {
        *ret_corners_length *= 2;
        ret_corners = realloc(ret_corners, sizeof(struct point_t) * (*ret_corners_length));
      }

      ret_corners[corner_cnt].x = x;
      ret_corners[corner_cnt].y = y;
      corner_cnt++;

      // Skip some in the width direction
      x += min_dist;
    }
  }
  *num_corners = corner_cnt;
}

/**
 * Make offsets for FAST9 calculation
 * @param[out] *pixel The offset array of the different pixels
 * @param[in] row_stride The row stride in the image
 */
static void fast_make_offsets(int32_t *pixel, uint16_t row_stride, uint8_t pixel_size)
{
  pixel[0]  = 0 * pixel_size  + row_stride * 3 * pixel_size;
  pixel[1]  = 1 * pixel_size  + row_stride * 3 * pixel_size;
  pixel[2]  = 2 * pixel_size  + row_stride * 2 * pixel_size;
  pixel[3]  = 3 * pixel_size  + row_stride * 1 * pixel_size;
  pixel[4]  = 3 * pixel_size  + row_stride * 0 * pixel_size;
  pixel[5]  = 3 * pixel_size  + row_stride * -1 * pixel_size;
  pixel[6]  = 2 * pixel_size  + row_stride * -2 * pixel_size;
  pixel[7]  = 1 * pixel_size  + row_stride * -3 * pixel_size;
  pixel[8]  = 0 * pixel_size  + row_stride * -3 * pixel_size;
  pixel[9]  = -1 * pixel_size + row_stride * -3 * pixel_size;
  pixel[10] = -2 * pixel_size + row_stride * -2 * pixel_size;
  pixel[11] = -3 * pixel_size + row_stride * -1 * pixel_size;
  pixel[12] = -3 * pixel_size + row_stride * 0 * pixel_size;
  pixel[13] = -3 * pixel_size + row_stride * 1 * pixel_size;
  pixel[14] = -2 * pixel_size + row_stride * 2 * pixel_size;
  pixel[15] = -1 * pixel_size + row_stride * 3 * pixel_size;
}
