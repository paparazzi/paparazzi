#ifndef POLY_ADV_H
#define POLY_ADV_H

#include "std.h"

typedef struct {float x; float y;} point2d;

typedef enum {ERR, ENTRY, SEG, TURN1, RET, TURN2} survey_stage;

extern bool_t init_poly_survey_adv(uint8_t first_wp, uint8_t size, float angle, float sweep_width, float shot_dist, float min_rad, float altitude);
extern bool_t poly_survey_adv(void);

#endif
