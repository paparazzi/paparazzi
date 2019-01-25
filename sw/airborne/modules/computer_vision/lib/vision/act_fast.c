/*
Copyright (c) 2017, Guido de Croon, TU Delft
All rights reserved.
*/


/**
 * @file modules/computer_vision/lib/vision/act_fast.c
 * @brief Finds corners in an image by actively scanning the image. This method is inspired by the work in:
 * de Croon, G.C.H.E., and Nolfi, S. (2013, May). Act-corner: Active corner finding for optic flow determination. In Robotics and Automation (ICRA), 2013 IEEE International Conference on (pp. 4679-4684). IEEE.
 *
 * The main idea of this particular implementation, called ACT-FAST, is that actively scanning the image allows to:
 * 1. Skip uniform areas in the image. If these areas are contiguous, many non-corners will never be evaluated.
 * 2. Follow edges. Typically, following an edge will bring an agent to a corner.
 * These two simple rules lead to a significant lower number of corner evaluations, while still detecting a reasonable number of corners.
 * Moreover, since the agents scanning the image start on a grid, corners will be quite well-distributed over the image.
 * Each step of the agent starts by classifying the agent location as a corner or not with FAST.
 *
 * For bigger images (e.g., 640 x 640), the computational advantage of ACT-FAST over the normal, exhaustive application of FAST becomes significant (in the order of a factor > 10).
 *
 * The code here is part of the following publication:
 * de Croon, G.C.H.E. "ACT-FAST: efficiently finding corners by actively exploring images.", in submission.
 *
 */

#include "fast_rosten.h"
#include "act_fast.h"
#include "math.h"
#include "image.h"


#define MAX_AGENTS 1000
struct agent_t agents[MAX_AGENTS];

/**
 * Do an ACT-FAST corner detection.
 * @param[in] *img The image to do the corner detection on
 * @param[in] threshold The threshold which we use for FAST9
 * @param[in] *num_corners reference to the amount of corners found, set by this function
 * @param[in] **ret_corners pointer to the array which contains the corners that were detected.
 * @param[in] n_agents The number of agents that will scan the image for corners
 * @param[in] n_time_steps The maximum number of time steps allowed for scanning
 * @param[in] long_step When there is not enough texture, the agent will take a long step to a next point of this length in pixels
 * @param[in] short_step When there is texture, the agent will follow the edge with this short step in pixels
 * @param[in] min_gradient The minimum gradient, in order to determine when to take a long or short step
 * @param[in] gradient_method: 0 = simple {-1, 0, 1}, 1 = Sobel {-1,0,1,-2,0,2,-1,0,1}
*/
void act_fast(struct image_t *img, uint8_t fast_threshold, uint16_t *num_corners, struct point_t **ret_corners,
              uint16_t n_agents, uint16_t n_time_steps, float long_step, float short_step, int min_gradient, int gradient_method)
{

  /*
   * Procedure:
   * 1) initialize agent positions
   * 2) loop over the agents, moving and checking for corners
   */

  // ensure that n_agents is never bigger than MAX_AGENTS
  n_agents = (n_agents < MAX_AGENTS) ? n_agents : MAX_AGENTS;

  int border = 4;

  // ***********************************
  // 1) initialize the agents' positions
  // ***********************************

  // grid sampling with a border:
  int init_border = 10;
  float GRID_ROWS = (int) ceil(sqrtf((float) n_agents));
  float step_size_x = (img->w - 2 * init_border) / (GRID_ROWS - 1);
  float step_size_y = (img->h - 2 * init_border) / (GRID_ROWS - 1);

  int a = 0;
  float px, py, pnorm;
  for (int c = 0; c < GRID_ROWS; c++) {
    for (int r = 0; r < GRID_ROWS; r++) {
      // px, py represent the preferred direction of the agent when there is no texture
      // here we initialize it differently for each agent:
      // TODO: don't we have a randf function in Paparazzi?
      px = ((float)(rand() % 10000) + 1) / 10000.0f;
      py = ((float)(rand() % 10000) + 1) / 10000.0f;
      pnorm = sqrtf(px * px + py * py);
      struct agent_t ag = { (border + c * step_size_x), (border + r * step_size_y), 1, px / pnorm, py / pnorm};
      agents[a] = ag;
      a++;
      if (a == n_agents) { break; }
    }

    // only initialize a maximum of n_agents agents.
    if (a == n_agents) { break; }
  }

  /* ********************************************************
   * 2) loop over the agents, moving and checking for corners
   * ********************************************************/

  // gradient
  int dx, dy;

  // loop over all time steps:
  for (int t = 0; t < n_time_steps; t++) {
    // loop over the agents
    for (a = 0; a < n_agents; a++) {
      // only do something if the agent is active:
      if (agents[a].active) {
        // check if this position is a corner:
        uint16_t x = (uint16_t) agents[a].x;
        uint16_t y = (uint16_t) agents[a].y;
        if (fast9_detect_pixel(img, fast_threshold, x, y)) {
          // we arrived at a corner, yeah!!!
          agents[a].active = 0;
          break;
        } else {
          // make a step:
          struct point_t loc = { .x = agents[a].x, .y = agents[a].y};
          image_gradient_pixel(img, &loc, gradient_method, &dx, &dy);
          int gradient = (abs(dx) + abs(dy)) / 2;
          if (abs(gradient) >= min_gradient) {
            // determine the angle and make a step in that direction:
            float norm_factor = sqrtf((float)(dx * dx + dy * dy));
            agents[a].x += (dy / norm_factor) * short_step;
            agents[a].y += (dx / norm_factor) * short_step;
          } else {
            // make a step in the preferred direction:
            agents[a].x += agents[a].preferred_dir_x * long_step;
            agents[a].y += agents[a].preferred_dir_y * long_step;
          }
        }

        // let the agent move over the image in a toroid world:
        if (agents[a].x > img->w - border) {
          agents[a].x = border;
        } else if (agents[a].x < border) {
          agents[a].x = img->w - border;
        }
        if (agents[a].y > img->h - border) {
          agents[a].y = border;
        } else if (agents[a].y < border) {
          agents[a].y = img->h - border;
        }
      }
    }
  }

  // Transform agents to corners:
  (*num_corners) = 0;
  for (a = 0; a < n_agents; a++) {

    // for active agents do a last check on the new position:
    if (agents[a].active) {
      // check if the last step brought the agent to a corner:
      uint16_t x = (uint16_t) agents[a].x;
      uint16_t y = (uint16_t) agents[a].y;
      if (fast9_detect_pixel(img, fast_threshold, x, y)) {
        // we arrived at a corner, yeah!!!
        agents[a].active = 0;
      }
    }

    // if inactive, the agent is a corner:
    if (!agents[a].active) {
      (*ret_corners)[(*num_corners)].x = (uint32_t) agents[a].x;
      (*ret_corners)[(*num_corners)].y = (uint32_t) agents[a].y;
      (*num_corners)++;
    }
  }
}

