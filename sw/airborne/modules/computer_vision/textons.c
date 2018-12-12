/*
 * Copyright (C) 2016, Hann Woei Ho, Guido de Croon
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
 * @file modules/computer_vision/textons.c
 *
 * Takes an image and represents the texture and colors in the image with a texton histogram.
 * A texton is a cluster centroid in a space populated by image patches. First, this code
 * learns or loads a texton dictionary. Then, for each incoming image, patches are sampled from
 * the image, compared to textons in the dictionary, and the closest texton is identified,
 * augmenting the corresponding bin in the texton histogram.
 */

#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/textons.h"

static pthread_mutex_t textons_mutex; ///< Mutex lock for thread safety
float *TD_0; // temporary array meant for internal use
float *TD_1; // temporary array meant for internal use
uint8_t TD_ID; // indicates which array to take

float ** **dictionary;
uint32_t learned_samples = 0;
uint8_t dictionary_initialized = 0;
float *texton_distribution;

#define MAX_N_TEXTONS 255

// initial settings:
#ifndef TEXTONS_FPS
#define TEXTONS_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(TEXTONS_FPS)

#ifndef TEXTONS_RUN
#define TEXTONS_RUN 1
#endif
PRINT_CONFIG_VAR(TEXTONS_RUN)

#ifndef TEXTONS_LOAD_DICTIONARY
#define TEXTONS_LOAD_DICTIONARY 1
#endif
PRINT_CONFIG_VAR(TEXTONS_LOAD_DICTIONARY)

#ifndef TEXTONS_REINITIALIZE_DICTIONARY
#define TEXTONS_REINITIALIZE_DICTIONARY 0
#endif
PRINT_CONFIG_VAR(TEXTONS_REINITIALIZE_DICTIONARY)

#ifndef TEXTONS_ALPHA
#define TEXTONS_ALPHA 0
#endif
PRINT_CONFIG_VAR(TEXTONS_ALPHA)

#ifndef TEXTONS_N_TEXTONS
#define TEXTONS_N_TEXTONS 20
#endif
PRINT_CONFIG_VAR(TEXTONS_N_TEXTONS)

#ifndef TEXTONS_N_SAMPLES
#define TEXTONS_N_SAMPLES 250
#endif
PRINT_CONFIG_VAR(TEXTONS_N_SAMPLES)

#ifndef TEXTONS_PATCH_SIZE
#define TEXTONS_PATCH_SIZE 6
#endif
PRINT_CONFIG_VAR(TEXTONS_PATCH_SIZE)

#ifndef TEXTONS_N_LEARNING_SAMPLES
#define TEXTONS_N_LEARNING_SAMPLES 5000
#endif
PRINT_CONFIG_VAR(TEXTONS_N_LEARNING_SAMPLES)

#ifndef TEXTONS_FULL_SAMPLING
#define TEXTONS_FULL_SAMPLING 0
#endif
PRINT_CONFIG_VAR(TEXTONS_FULL_SAMPLING)

#ifndef TEXTONS_BORDER_WIDTH
#define TEXTONS_BORDER_WIDTH 0
#endif
PRINT_CONFIG_VAR(TEXTONS_BORDER_WIDTH)

#ifndef TEXTONS_BORDER_HEIGHT
#define TEXTONS_BORDER_HEIGHT 0
#endif
PRINT_CONFIG_VAR(TEXTONS_BORDER_HEIGHT)

#ifndef TEXTONS_DICTIONARY_NUMBER
#define TEXTONS_DICTIONARY_NUMBER 0
#endif
PRINT_CONFIG_VAR(TEXTONS_DICTIONARY_NUMBER)

#ifndef TEXTONS_EXECUTION_PERIOD
#define TEXTONS_EXECUTION_PERIOD 1
#endif
PRINT_CONFIG_VAR(TEXTONS_EXECUTION_PERIOD)

uint8_t running = TEXTONS_RUN;
uint8_t load_dictionary = TEXTONS_LOAD_DICTIONARY;
uint8_t reinitialize_dictionary = TEXTONS_REINITIALIZE_DICTIONARY;
uint8_t alpha_uint = TEXTONS_ALPHA;
uint8_t n_textons = TEXTONS_N_TEXTONS;
uint8_t patch_size = TEXTONS_PATCH_SIZE;
uint32_t n_learning_samples = TEXTONS_N_LEARNING_SAMPLES;
uint32_t n_samples_image = TEXTONS_N_SAMPLES;
uint8_t FULL_SAMPLING = TEXTONS_FULL_SAMPLING;
uint32_t border_width = TEXTONS_BORDER_WIDTH;
uint32_t border_height = TEXTONS_BORDER_HEIGHT;
uint8_t dictionary_number = TEXTONS_DICTIONARY_NUMBER;
uint8_t execution_period = TEXTONS_EXECUTION_PERIOD;

// status variables
uint8_t dictionary_ready = 0;
float alpha = 0.0;
uint8_t cycle = 0;

// File pointer for saving the dictionary
static FILE *dictionary_logger = NULL;

#ifndef DICTIONARY_PATH
#define DICTIONARY_PATH /data/ftp/internal_000
#endif

/**
 * Main texton processing function that first either loads or learns a dictionary and then extracts the texton histogram.
 * @param[out] *img The output image
 * @param[in] *img The input image (YUV422)
 */

struct image_t *texton_func(struct image_t *img);
struct image_t *texton_func(struct image_t *img)
{

  // whether to execute the function:
  if (!running) { return img; }

  // only execute the texton function once every execution_period times:
  cycle = (cycle + 1) % execution_period;
  if (cycle > 0) { return img; }

  if (img->buf_size == 0) { return img; }

  // extract frame from img struct:
  uint8_t *frame = (uint8_t *)img->buf;

  // if patch size odd, correct:
  if (patch_size % 2 == 1) { patch_size++; }

  // check whether we have to reinitialize the dictionary:
  if (reinitialize_dictionary) {
    // set all vars to trigger a reinitialization and learning phase of the dictionary:
    dictionary_ready = 0;
    dictionary_initialized = 0;
    load_dictionary = 0;
    learned_samples = 0;
    alpha_uint = 10;
    // reset reinitialize_dictionary
    reinitialize_dictionary = 0;
  }

  // if dictionary not initialized:
  if (dictionary_ready == 0) {
    if (load_dictionary == 0) {

      printf("Learned samples: %d / %d\n", learned_samples, n_learning_samples);

      // Train the dictionary:
      DictionaryTrainingYUV(frame, img->w, img->h);

      // After a number of samples, stop learning:
      if (learned_samples >= n_learning_samples) {
        // Save the dictionary:
        save_texton_dictionary();
        // stop learning:
        dictionary_ready = 1;
        // lower learning rate
        alpha = 0.0;
        printf("Enough learning!\n");
        alpha_uint = 0;
        // set learned samples back to 0
        learned_samples = 0;
      }
    } else {
      // Load the dictionary:
      load_texton_dictionary();
    }
  } else {

    if (alpha_uint > 0) {

      //printf("Learning, frame time = %d\n", img->ts.tv_sec * 1000 + img->ts.tv_usec / 1000);
      printf("Learning, nr of samples: %d\n", n_learning_samples);
      DictionaryTrainingYUV(frame, img->w, img->h);

      if (learned_samples >= n_learning_samples) {
        // Save the dictionary:
        save_texton_dictionary();
        // reset learned_samples:
        learned_samples = 0;
      }
    } else {
      // Extract distributions
      DistributionExtraction(frame, img->w, img->h);
    }

    // printf("N textons = %d\n", n_samples_image);
    // printf("Entropy texton distribution = %f\n", get_entropy(texton_distribution, n_textons));

    /*printf("Distribution = [");
    for (i = 0; i < n_textons-1; i++) {
      printf("%f, ", texton_distribution[i]);
    }
    printf("%f]\n", texton_distribution[n_textons-1]);
    */
    /*
    float sum = 0;
    for (i = 0; i < n_textons; i++) {
      sum += texton_distribution[i];
    }
    printf("sum = %f\n", sum);
    */

  }

  return img; // Colorfilter did not make a new image
}

/**
 * Function that performs one pass for dictionary training. It extracts samples from an image, finds the closest texton
 * and moves it towards the sample.
 * @param[in] frame* The YUV image data
 * @param[in] width The width of the image
 * @param[in] height The height of the image
 */
void DictionaryTrainingYUV(uint8_t *frame, uint16_t width, uint16_t height)
{
  unsigned int i, j, w, s, texton, c; // iterators
  int x, y; // image coordinates
  float error_texton; // distance between an image patch and a texton
  float *TD;
  uint8_t *buf;

  // ***********************
  //   DICTIONARY LEARNING
  // ***********************

  if (!dictionary_initialized) {
    // **************
    // INITIALISATION
    // **************

    printf("Intializing dictionary!\n");

    // in the first image, we initialize the textons to random patches in the image
    for (w = 0; w < n_textons; w++) {
      // select a coordinate
      x = rand() % (width - patch_size);
      y = rand() % (height - patch_size);

      //printf("(x,y) = (%d,%d), (w,h) = (%d,%d), ps = %d\n", x, y, width, height, patch_size);
      // take the sample
      for (i = 0; i < patch_size; i++) {
        buf = frame + (width * 2 * (i + y)) + 2 * x;
        for (j = 0; j < patch_size; j++) {
          // put it in a texton
          printf("Setting dictionary:\n");
          // U/V component

          dictionary[w][i][j][0] = (float) * buf;
          buf += 1;
          // Y1/Y2 component
          dictionary[w][i][j][1] = (float) * buf;
          buf += 1;
          printf("Done!\n");
        }
      }
    }
    dictionary_initialized = 1;
  } else {
    // ********
    // LEARNING
    // ********
    printf("Learning!\n");
    alpha = ((float) alpha_uint) / 255.0;

    float *texton_distances, * **patch;
    texton_distances = (float *)calloc(n_textons, sizeof(float));
    patch = (float ** *)calloc(patch_size, sizeof(float **));

    for (i = 0; i < patch_size; i++) {
      patch[i] = (float **)calloc(patch_size, sizeof(float *));
      for (j = 0; j < patch_size; j++) {
        patch[i][j] = (float *)calloc(2, sizeof(float));
      }
    }

    // make sure that the other threads have access to the normalized distribution
    // and not the one we are going to fill / change.
    pthread_mutex_lock(&textons_mutex);
    if (TD_ID == 0) {
      // we are going to fill TD_0:
      TD = TD_0;
      texton_distribution = TD_1;
    } else {
      // we are going to fill TD_1:
      TD = TD_1;
      texton_distribution = TD_0;
    }
    TD_ID = (TD_ID + 1) % 2;
    pthread_mutex_unlock(&textons_mutex);

    for (i = 0; i < n_textons; i++) {
      TD[i] = 0.0f;
    }

    // Extract and learn from n_samples_image per image
    for (s = 0; s < n_samples_image; s++) {
      // select a random sample from the image
      x = rand() % (width - patch_size);
      y = rand() % (height - patch_size);

      // reset texton_distances
      for (texton = 0; texton < n_textons; texton++) {
        texton_distances[texton] = 0;
      }

      // extract sample
      for (i = 0; i < patch_size; i++) {
        buf = frame + (width * 2 * (i + y)) + 2 * x;
        for (j = 0; j < patch_size; j++) {
          // U/V component
          patch[i][j][0] = (float) * buf;
          buf += 1;
          // Y1/Y2 component
          patch[i][j][1] = (float) * buf;
          buf += 1;
        }
      }

      // determine distances to the textons:
      for (i = 0; i < patch_size; i++) {
        for (j = 0; j < patch_size; j++) {
          for (c = 0; c < 2; c++) {
            // determine the distance to textons
            for (texton = 0; texton < n_textons; texton++) {
              texton_distances[texton] += (patch[i][j][c] - dictionary[texton][i][j][c])
                                          * (patch[i][j][c] - dictionary[texton][i][j][c]);
            }
          }
        }
      }

      // search the closest texton
      int assignment = 0;
      float min_dist = texton_distances[0];
      for (texton = 1; texton < n_textons; texton++) {
        if (texton_distances[texton] < min_dist) {
          min_dist = texton_distances[texton];
          assignment = texton;
        }
      }

      // move the neighbour closer to the input
      for (i = 0; i < patch_size; i++) {
        for (j = 0; j < patch_size; j++) {
          for (c = 0; c < 2; c++) {
            error_texton = patch[i][j][c] - dictionary[assignment][i][j][c];
            dictionary[assignment][i][j][c] += (alpha * error_texton);
          }
        }
      }

      // put the assignment in the histogram
      TD[assignment]++;

      // Augment the number of learned samples:
      learned_samples++;
    }

    // Normalize distribution:
    for (i = 0; i < n_textons; i++) {
      TD[i] = TD[i] / (float) n_samples_image;
    }

    // Free the allocated memory:
    for (i = 0; i < patch_size; i++) {
      for (j = 0; j < patch_size; j++) {
        free(patch[i][j]);
      }
      free(patch[i]);
    }
    free(patch);
    free(texton_distances);
  }

  // Free the buffer
  buf = NULL;
  free(buf);
}

float* get_texton_distribution() {
	float* texton_distr = (float *)malloc(n_textons * sizeof(float));
	memcpy(texton_distr, texton_distribution, n_textons * sizeof(float));
	return texton_distr;
}

/**
 * Function that extracts a texton histogram from an image.
 * @param[in] frame* The YUV image data
 * @param[in] width The width of the image
 * @param[in] height The height of the image
 */
void DistributionExtraction(uint8_t *frame, uint16_t width, uint16_t height)
{
  int i, j, texton, c; // iterators
  int x, y, max_addition_x, max_addition_y; //, stride; // coordinates
  unsigned int n_extracted_textons = 0;
  float *TD;
  uint8_t *buf;
  float pixel_diff;

  max_addition_x = width - patch_size - 2 * border_width;
  max_addition_y = height - patch_size - 2 * border_height;
  // stride = 2 * width;

  // ************************
  //       EXECUTION
  // ************************

  // Allocate memory for texton distances and image patch:
  // TODO: do this globally once?
  float *texton_distances, * **patch;
  texton_distances = (float *)calloc(n_textons, sizeof(float));
  patch = (float ** *)calloc(patch_size, sizeof(float **));
  for (i = 0; i < patch_size; i++) {
    patch[i] = (float **)calloc(patch_size, sizeof(float *));
    for (j = 0; j < patch_size; j++) {
      patch[i][j] = (float *)calloc(2, sizeof(float));
    }
  }

  pthread_mutex_lock(&textons_mutex);
  if (TD_ID == 0) {
    // we are going to fill TD_0:
    TD = TD_0;
    texton_distribution = TD_1;
  } else {
    // we are going to fill TD_1:
    TD = TD_1;
    texton_distribution = TD_0;
  }
  TD_ID = (TD_ID + 1) % 2;
  pthread_mutex_unlock(&textons_mutex);

  for (i = 0; i < n_textons; i++) {
    TD[i] = 0.0f;
  }

  int finished = 0;
  x = 0;
  y = 0;
  while (!finished) {

    // if sub-sampling, take a random coordinate:
    // TODO: also allow for a grid:
    if (!FULL_SAMPLING) {
      x = border_width + rand() % max_addition_x;
      y = border_height + rand() % max_addition_y;
    }

    // reset texton_distances
    for (texton = 0; texton < n_textons; texton++) {
      texton_distances[texton] = 0;
    }

    // extract sample
    for (i = 0; i < patch_size; i++) {
      buf = frame + (width * 2 * (i + y)) + 2 * x;
      // old version:
      // buf = frame + (stride * (i + y)) + 2 * x;
      for (j = 0; j < patch_size; j++) {
        // U/V component
        patch[i][j][0] = (float) * buf;
        buf += 1;
        // Y1/Y2 component
        patch[i][j][1] = (float) * buf;
        buf += 1;
      }
    }

    // determine distances:
    for (i = 0; i < patch_size; i++) {
      for (j = 0; j < patch_size; j++) {
        for (c = 0; c < 2; c++) {
          // determine the distance to words
          for (texton = 0; texton < n_textons; texton++) {
            pixel_diff = patch[i][j][c] - dictionary[texton][i][j][c];
            texton_distances[texton] += pixel_diff * pixel_diff;
          }
        }
      }
    }

    // determine the nearest neighbour
    // search the closest centroid
    int assignment = 0;
    float min_dist = texton_distances[0];
    for (texton = 1; texton < n_textons; texton++) {
      if (texton_distances[texton] < min_dist) {
        min_dist = texton_distances[texton];
        assignment = texton;
      }
    }

    // put the assignment in the histogram
    TD[assignment]++;
    n_extracted_textons++;

    if (!FULL_SAMPLING && n_extracted_textons == n_samples_image) {
      finished = 1;
    } else {
      // FULL_SAMPLING is actually a sampling that covers the image:
      y += patch_size;
      // True full sampling would require:
      // y++;

      if (y > height - patch_size) {
        if (!FULL_SAMPLING) {
          x += patch_size;
        } else {
          x++;
        }
        y = 0;
      }
      if (x > width - patch_size) {
        finished = 1;
      }
    }
  }

  // Normalize distribution:
  for (i = 0; i < n_textons; i++) {
    TD[i] = TD[i] / (float) n_extracted_textons;
  }

  // free memory:
  for (i = 0; i < patch_size; i++) {
    for (j = 0; j < patch_size; j++) {
      free(patch[i][j]);
    }
    free(patch[i]);
  }
  free(patch);
  free(texton_distances);

  buf = NULL;
  free(buf);

} // EXECUTION



/**
 * Save the texton dictionary.
 */
void save_texton_dictionary(void)
{

  printf("Saving dictionary!\n");
  //save a dictionary
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/Dictionary_%05d.dat", STRINGIFY(DICTIONARY_PATH), dictionary_number);

  dictionary_logger = fopen(filename, "w");

  if (dictionary_logger == NULL) {
    printf("Filename: %s\n", filename);
    perror("Error while opening the file.\n");
  } else {
    // (over-)write dictionary
    for (uint8_t i = 0; i < n_textons; i++) {
      for (uint8_t j = 0; j < patch_size; j++) {
        for (uint8_t k = 0; k < patch_size; k++) {
          fprintf(dictionary_logger, "%f\n", dictionary[i][j][k][0]);
          fprintf(dictionary_logger, "%f\n", dictionary[i][j][k][1]);
        }
      }
    }
    fclose(dictionary_logger);
  }
}

/**
 * Load a texton dictionary.
 */
void load_texton_dictionary(void)
{
  char filename[512];
  sprintf(filename, "%s/Dictionary_%05d.dat", STRINGIFY(DICTIONARY_PATH), dictionary_number);

  if ((dictionary_logger = fopen(filename, "r"))) {
    // Load the dictionary:
    for (int i = 0; i < n_textons; i++) {
      for (int j = 0; j < patch_size; j++) {
        for (int k = 0; k < patch_size; k++) {
          if (fscanf(dictionary_logger, "%f\n", &dictionary[i][j][k][0]) == EOF) { break; }
          if (fscanf(dictionary_logger, "%f\n", &dictionary[i][j][k][1]) == EOF) { break; }
        }
      }
    }

    fclose(dictionary_logger);
    dictionary_ready = 1;
  } else {
    // If the given dictionary does not exist, we start learning one:
    printf("Texton dictionary %d does not exist, we start learning one.\n", dictionary_number);
    load_dictionary = 0;
    learned_samples = 0;
    dictionary_initialized = 0;
  }
}

/**
 * Initialize
 */
void textons_init(void)
{
  printf("Textons init\n");
  TD_ID = 0;
  TD_0 = (float *)calloc(MAX_N_TEXTONS, sizeof(float));
  TD_1 = (float *)calloc(MAX_N_TEXTONS, sizeof(float));

  dictionary_initialized = 0;
  learned_samples = 0;
  dictionary_ready = 0;

  dictionary = (float ** **)calloc(MAX_N_TEXTONS, sizeof(float ** *));
  for (int w = 0; w < MAX_N_TEXTONS; w++) {
    dictionary[w] = (float ** *) calloc(patch_size, sizeof(float **));
    for (int i = 0; i < patch_size; i++) {
      dictionary[w][i] = (float **) calloc(patch_size, sizeof(float *));
      for (int j = 0; j < patch_size; j++) {
        dictionary[w][i][j] = (float *) calloc(2, sizeof(float));
      }
    }
  }

  cv_add_to_device(&TEXTONS_CAMERA, texton_func, TEXTONS_FPS);
}

void textons_stop(void)
{
  free(TD_0);
  free(TD_1);
  free(dictionary);
}

/**
 * Function that calculates a base-2 Shannon entropy for a probability distribution.
 * @param[in] p_dist The probability distribution array
 * @param[in] D Size of the array
 */
float get_entropy(float *p_dist, int D)
{
  float entropy = 0.0f;
  int i;
  for (i = 0; i < D; i++) {
    if (p_dist[i] > 0) {
      entropy -= p_dist[i] * log2(p_dist[i]);
    }
  }

  return entropy;
}
