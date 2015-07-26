#include "size_divergence.h"
#include <stdlib.h>

#define NO_DIV 0.0

/**
 * Get divergence from optical flow vectors based on line sizes between corners
 * @param[out] divergence
 * @param[in] flow_t* vectors The optical flow vectors
 * @param[in] count The number of optical flow vectors
 * @param[in] n_samples The number of line segments that will be taken into account. 0 means all line segments will be considered.
 */
float get_size_divergence(struct flow_t *vectors, int count, int n_samples)
{
  float distance_1;
  float distance_2;
  float *divs;
  unsigned int sample;
  float dx;
  float dy;
  float mean_divergence;
  int n_elements;
  unsigned int i, j;

  if (count < 2) {
    return NO_DIV;
  }

  if (n_samples == 0) {
    // divs will contain the individual divergence estimates:
    n_elements = (count * count - count) / 2;
    divs = (float *) malloc(sizeof(float) * n_elements);

    // go through all possible lines:
    sample = 0;
    for (i = 0; i < count; i++) {
      for (j = i + 1; j < count; j++) {
        // distance in previous image:
        dx = vectors[i].pos.x - vectors[j].pos.x;
        dy = vectors[i].pos.y - vectors[j].pos.y;
        distance_1 = sqrt(dx * dx + dy * dy);

        // distance in current image:
        dx = vectors[i].pos.x + vectors[i].flow_x - vectors[j].pos.x - vectors[j].flow_x;
        dy = vectors[i].pos.y + vectors[i].flow_y - vectors[j].pos.y - vectors[j].flow_y;
        distance_2 = sqrt(dx * dx + dy * dy);

        // calculate divergence for this sample:
        divs[sample] = (distance_2 - distance_1) / distance_1;
        sample++;
      }
    }

    // calculate the mean divergence:
    mean_divergence = get_mean(divs, n_elements);

    // free the memory of divs:
    free(divs);
  } else {
    // vector that will contain individual divergence estimates:
    divs = (float *) malloc(sizeof(float) * n_samples);

    // take random samples:
    for (sample = 0; sample < n_samples; sample++) {
      // take two random indices:
      i = rand() % count;
      j = rand() % count;
      // ensure it is not the same index:
      while (i == j) {
        j = rand() % count;
      }

      // distance in previous image:
      dx = vectors[i].pos.x - vectors[j].pos.x;
      dy = vectors[i].pos.y - vectors[j].pos.y;
      distance_1 = sqrt(dx * dx + dy * dy);

      // distance in current image:
      dx = vectors[i].pos.x + vectors[i].flow_x - vectors[j].pos.x - vectors[j].flow_x;
      dy = vectors[i].pos.y + vectors[i].flow_y - vectors[j].pos.y - vectors[j].flow_y;
      distance_2 = sqrt(dx * dx + dy * dy);

      // calculate divergence for this sample:
      divs[sample] = (distance_2 - distance_1) / distance_1;
    }

    // calculate the mean divergence:
    mean_divergence = get_mean(divs, n_samples);

    // free the memory of divs:
    free(divs);
  }

  // return the calculated divergence:
  return mean_divergence;
}

/**
 * Get the sample mean of a vector of floats
 * @param[out] mean
 * @param[in] float* numbers Vector of numbers
 * @param[in] n_elements Number of elements
 */
float get_mean(float *numbers, int n_elements)
{
  int i = 0;
  float mean = 0;
  for (i = 0; i < n_elements; i++) {
    mean += numbers[i];
  }
  mean /= n_elements;
  return mean;
}