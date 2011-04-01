/************** Spiral Navigation **********************************************/

/** creating a  helix:
	start radius to end radius, increasing after reaching alphamax
	Alphamax is calculated from given segments
	IMPORTANT: numer of segments has to be larger than 2!
*/

#include "spiral.h"

#include "subsystems/nav.h"
#include "estimator.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
//#include "modules/digital_cam/dc.h"

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
static float CamAngle;
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

  EdgeCurrentX = waypoints[Edge].x - waypoints[Center].x;
  EdgeCurrentY = waypoints[Edge].y - waypoints[Center].y;

  Spiralradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);

  TransCurrentX = estimator_x - waypoints[Center].x;
  TransCurrentY = estimator_y - waypoints[Center].y;
  TransCurrentZ = estimator_z - ZPoint;
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

  // 	SpiralTheta = atan2(TransCurrentY,TransCurrentX);
  // 	Fly2X = Spiralradius*cos(SpiralTheta+3.14)+waypoints[Center].x;
  // 	Fly2Y = Spiralradius*sin(SpiralTheta+3.14)+waypoints[Center].y;

  // Alphalimit denotes angle, where the radius will be increased
  Alphalimit = 2*3.14 / Segments;
  //current position
  FlyFromX = estimator_x;
  FlyFromY = estimator_y;

  if(DistanceFromCenter > Spiralradius)
	CSpiralStatus = Outside;
  return FALSE;
}

bool_t SpiralNav(void)
{
  TransCurrentX = estimator_x - waypoints[Center].x;
  TransCurrentY = estimator_y - waypoints[Center].y;
  DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

  bool_t InCircle = TRUE;

  if(DistanceFromCenter > Spiralradius)
	InCircle = FALSE;

  switch(CSpiralStatus)
	{
	case Outside:
	  //flys until center of the helix is reached an start helix
	  nav_route_xy(FlyFromX,FlyFromY,waypoints[Center].x, waypoints[Center].y);
	  // center reached?
	  if (nav_approaching_xy(waypoints[Center].x, waypoints[Center].y, FlyFromX, FlyFromY, 0)) {
		// nadir image
		//dc_shutter();
		CSpiralStatus = StartCircle;
	  }
	  break;
	case StartCircle:
	  // Starts helix
	  // storage of current coordinates
	  // calculation needed, State switch to Circle
	  nav_circle_XY(waypoints[Center].x, waypoints[Center].y, SRad);
	  if(DistanceFromCenter >= SRad){
		LastCircleX = estimator_x;
		LastCircleY = estimator_y;
		CSpiralStatus = Circle;
		// Start helix
		//dc_Circle(360/Segmente);
	  }
	  break;
	case Circle: {
	  nav_circle_XY(waypoints[Center].x, waypoints[Center].y, SRad);
	  // Trigonometrische Berechnung des bereits geflogenen Winkels alpha
	  // equation:
	  // alpha = 2 * asin ( |Starting position angular - current positon| / (2* SRad)
	  // if alphamax already reached, increase radius.

	  //DistanceStartEstim = |Starting position angular - current positon|
	  float DistanceStartEstim = sqrt (((LastCircleX-estimator_x)*(LastCircleX-estimator_x))
									   + ((LastCircleY-estimator_y)*(LastCircleY-estimator_y)));
	  float CircleAlpha = (2.0 * asin (DistanceStartEstim / (2 * SRad)));
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
		  /*if (dc_cam_tracing) {
			// calculating Camwinkel for camera alignment
			TransCurrentZ = estimator_z - ZPoint;
			CamAngle = atan(SRad/TransCurrentZ) * 180  / 3.14;
			//dc_cam_angle = CamAngle;
			}*/
		}
	  else {
		SRad = Spiralradius;
		// Stopps DC
		//dc_stop();
	  }
	  CSpiralStatus = Circle;
	  break;

	}
  return TRUE;
}
