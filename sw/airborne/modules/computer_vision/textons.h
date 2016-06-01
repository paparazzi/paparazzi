/*
 * Copyright (C) 2016 Hann Woei Ho, Guido de Croon
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/textons.h
 *
 * Takes an image and represents the texture and colors in the image with a texton histogram.
 * A texton is a cluster centroid in a space populated by image patches. First, this code
 * learns or loads a texton dictionary. Then, for each incoming image, patches are sampled from
 * the image, compared to textons in the dictionary, and the closest texton is identified,
 * augmenting the corresponding bin in the texton histogram.
 *
 */

#ifndef TEXTONS_H
#define TEXTONS_H

#include <stdint.h>

// outputs
extern float *texton_distribution; // main outcome of the image processing: the distribution of textons in the image

// settings
extern uint8_t load_dictionary;
extern uint8_t alpha_uint;
extern uint8_t n_textons;
extern uint8_t patch_size; // Should be even
extern uint32_t n_learning_samples;
extern uint32_t n_samples_image;
extern uint8_t FULL_SAMPLING;
extern uint32_t border_width;
extern uint32_t border_height;
extern uint8_t dictionary_number;

// status variables
extern uint8_t dictionary_ready;
extern float alpha;
extern float ** **dictionary;
extern uint32_t learned_samples;
extern uint8_t dictionary_initialized;

// functions:
void DictionaryTrainingYUV(uint8_t *frame, uint16_t width, uint16_t height);
void DistributionExtraction(uint8_t *frame, uint16_t width, uint16_t height);
void save_texton_dictionary(void);
void load_texton_dictionary(void);

// Module functions
extern void textons_init(void);
extern void textons_stop(void);

#endif /* TEXTONS_H */
