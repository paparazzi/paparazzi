/************** Spiral Navigation **********************************************/

/** creating a  helix:
	start radius to end radius, increasing after reaching alphamax
	Alphamax is calculated from given segments
	IMPORTANT: numer of segments has to be larger than 2!
*/

#include "subsystems/navigation/spiral.h"

#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"

#ifdef DIGITAL_CAM
#include "modules/digital_cam/dc.h"
#endif

enum SpiralStatus { Outside, StartCircle, Circle, IncSpiral };
static enum SpiralStatus CSpiralStatus;
// static float SpiralTheta;
// static float Fly2X;
// static float Fly2Y;

static float FlyFromX;
static float FlyFromY;
static float TransCurrentX;
static float TransCurrentY;
static float TransCurrentZ;
static float EdgeCurrentX;
static float EdgeCurrentY;
static float LastCircleX;
static float LastCircleY;
static float DistanceFromCenter;
static float Spiralradius;
static uint8_t Center;
static uint8_t Edge;
static float SRad;
static float IRad;
static float Alphalimit;
static float Segmente;
static float ZPoint;
static float nav_radius_min;

#ifndef MIN_CIRCLE_RADIUS
#define MIN_CIRCLE_RADIUS 120
#endif


bool_t InitializeSpiral(uint8_t CenterWP, uint8_t EdgeWP, float StartRad, float IncRad, float Segments, float ZKoord)
{
  Center = CenterWP; 	// center of the helix
  Edge = EdgeWP; 		// edge point on the maximaum radius
  SRad = StartRad;	// start radius of the helix
  Segmente = Segments;
  ZPoint = ZKoord;
  nav_radius_min = MIN_CIRCLE_RADIUS;
  if (SRad < nav_radius_min) SRad = nav_radius_min;
  IRad = IncRad;		// multiplier for increasing the spiral

  EdgeCurrentX = WaypointX(Edge) - WaypointX(Center);
  EdgeCurrentY = WaypointY(Edge) - WaypointY(Center);

  Spiralradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);

  TransCurrentX = estimator_x - WaypointX(Center);
  TransCurrentY = estimator_y - WaypointY(Center);
  TransCurrentZ = estimator_z - ZPoint;
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

  // 	SpiralTheta = atan2(TransCurrentY,TransCurrentX);
  // 	Fly2X = Spiralradius*cos(SpiralTheta+M_PI)+WaypointX(Center);
  // 	Fly2Y = Spiralradius*sin(SpiralTheta+M_PI)+WaypointY(Center);

  // Alphalimit denotes angle, where the radius will be increased
  Alphalimit = 2*M_PI / Segments;
  //current position
  FlyFromX = estimator_x;
  FlyFromY = estimator_y;

  if(DistanceFromCenter > Spiralradius)
	CSpiralStatus = Outside;
  return FALSE;
}

bool_t SpiralNav(void)
{
  TransCurrentX = estimator_x - WaypointX(Center);
  TransCurrentY = estimator_y - WaypointY(Center);
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

  bool_t InCircle = TRUE;
  float DistanceStartEstim;
  float CircleAlpha;

  if(DistanceFromCenter > Spiralradius)
	InCircle = FALSE;

  switch(CSpiralStatus)
	{
	case Outside:
	  //flys until center of the helix is reached an start helix
	  nav_route_xy(FlyFromX,FlyFromY,WaypointX(Center), WaypointY(Center));
	  // center reached?
	  if (nav_approaching_xy(WaypointX(Center), WaypointY(Center), FlyFromX, FlyFromY, 0)) {
		// nadir image
#ifdef DIGITAL_CAM
		dc_send_command(DC_SHOOT);
#endif
		CSpiralStatus = StartCircle;
	  }
	  break;
	case StartCircle:
	  // Starts helix
	  // storage of current coordinates
	  // calculation needed, State switch to Circle
	  nav_circle_XY(WaypointX(Center), WaypointY(Center), SRad);
	  if(DistanceFromCenter >= SRad){
		LastCircleX = estimator_x;
		LastCircleY = estimator_y;
		CSpiralStatus = Circle;
		// Start helix
#ifdef DIGITAL_CAM
		dc_Circle(360/Segmente);
#endif
	  }
	  break;
	case Circle: {
	  nav_circle_XY(WaypointX(Center), WaypointY(Center), SRad);
	  // Trigonometrische Berechnung des bereits geflogenen Winkels alpha
	  // equation:
	  // alpha = 2 * asin ( |Starting position angular - current positon| / (2* SRad)
	  // if alphamax already reached, increase radius.

	  //DistanceStartEstim = |Starting position angular - current positon|
	  DistanceStartEstim = sqrt (((LastCircleX-estimator_x)*(LastCircleX-estimator_x))
									   + ((LastCircleY-estimator_y)*(LastCircleY-estimator_y)));
	  CircleAlpha = (2.0 * asin (DistanceStartEstim / (2 * SRad)));
	  if (CircleAlpha >= Alphalimit) {
		LastCircleX = estimator_x;
		LastCircleY = estimator_y;
		CSpiralStatus = IncSpiral;
	  }
	  break;
    }
	case IncSpiral:
	  // increasing circle radius as long as it is smaller than max helix radius
	  if(SRad + IRad < Spiralradius)
		{
		  SRad = SRad + IRad;
#ifdef DIGITAL_CAM
		  if (dc_cam_tracing) {
			// calculating Cam angle for camera alignment
			TransCurrentZ = estimator_z - ZPoint;
			dc_cam_angle = atan(SRad/TransCurrentZ) * 180  / M_PI;
          }
#endif
		}
	  else {
		SRad = Spiralradius;
#ifdef DIGITAL_CAM
		// Stopps DC
		dc_stop();
#endif
	  }
	  CSpiralStatus = Circle;
	  break;

	}
  return TRUE;
}
