#ifndef OSAMNav_H
#define OSAMNav_H

#include "std.h"
#include "nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "flight_plan.h"

struct Point2D {float x; float y;};
struct Line {float m;float b;float x;};

extern bool_t FlowerNav(void);
extern bool_t InitializeFlower(uint8_t CenterWP, uint8_t EdgeWP);

extern bool_t InitializeBungeeTakeoff(uint8_t BungeeWP);
extern bool_t BungeeTakeoff(void);

#define PolygonSize 10
#define MaxFloat   1000000000
#define MinFloat   -1000000000

extern bool_t InitializePolygonSurvey(uint8_t FirstWP, uint8_t Size, float Sweep, float Orientation);
extern bool_t PolygonSurvey(void);
extern uint16_t PolySurveySweepNum;
extern uint16_t PolySurveySweepBackNum;

void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY);
void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY);

void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2);
float EvaluateLineForY(float x, struct Line L);
float EvaluateLineForX(float y, struct Line L);
float DistanceEquation(struct Point2D p1,struct Point2D p2);

#endif
