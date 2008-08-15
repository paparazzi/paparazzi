/** Automatic survey of a rectangle (defined by two points) (south-north or west-east sweep) */

#ifndef NAV_SURVEY_RECTANGLE_H
#define NAV_SURVEY_RECTANGLE_H

#include "nav.h"

typedef enum {NS, WE} survey_orientation_t;

extern void nav_survey_rectangle_init(uint8_t wp1, uint8_t wp2, float grid, survey_orientation_t so);
extern void nav_survey_rectangle(uint8_t wp1, uint8_t wp2);

#define NavSurveyRectangleInit(_wp1, _wp2, _grid, _orientation) nav_survey_rectangle_init(_wp1, _wp2, _grid, _orientation)
#define NavSurveyRectangle(_wp1, _wp2) nav_survey_rectangle(_wp1, _wp2)


#endif
