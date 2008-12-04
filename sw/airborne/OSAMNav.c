#include "OSAMNav.h"

/************** Flower Navigation **********************************************/

/** Makes a flower pattern. 
	CenterWP is the center of the flower. The Navigation Height is taken from this waypoint.
	EdgeWP defines the radius of the flower (distance from CenterWP to EdgeWP)	
*/

enum FlowerStatus { Outside, Line, Circle };
static enum FlowerStatus CFlowerStatus;
static float CircleX;
static float CircleY;
static float Fly2X;
static float Fly2Y;
static float FlyFromX;
static float FlyFromY;
static float TransCurrentX;
static float TransCurrentY;
static float EdgeCurrentX;
static float EdgeCurrentY;
static float DistanceFromCenter;
static float Theta;
static float radius;
static uint8_t Center;
static uint8_t Edge;

bool_t InitializeFlower(uint8_t CenterWP, uint8_t EdgeWP)
{
	Center = CenterWP;
	Edge = EdgeWP;

	EdgeCurrentX = waypoints[Edge].x - waypoints[Center].x;
	EdgeCurrentY = waypoints[Edge].y - waypoints[Center].y;
	
	radius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);

	TransCurrentX = estimator_x - waypoints[Center].x;
	TransCurrentY = estimator_y - waypoints[Center].y;
	DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

	Theta = atan2(TransCurrentY,TransCurrentX);
	Fly2X = radius*cos(Theta+3.14)+waypoints[Center].x;
	Fly2Y = radius*sin(Theta+3.14)+waypoints[Center].y;
	FlyFromX = estimator_x;
	FlyFromY = estimator_y;

	if(DistanceFromCenter > radius)
		CFlowerStatus = Outside;
	else
		CFlowerStatus = Line;

	CircleX = 0;
	CircleY = 0;
	return FALSE;
}

bool_t FlowerNav(void)
{
	TransCurrentX = estimator_x - waypoints[Center].x;
	TransCurrentY = estimator_y - waypoints[Center].y;
	DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);
	
	bool_t InCircle = TRUE;
	float CircleTheta;

	if(DistanceFromCenter > radius)
		InCircle = FALSE;

	NavVerticalAutoThrottleMode(0); /* No pitch */
  	NavVerticalAltitudeMode(waypoints[Center].a, 0.);

	switch(CFlowerStatus)
	{
	case Outside:		
		nav_route_xy(FlyFromX,FlyFromY,Fly2X,Fly2Y);
		if(InCircle)
		{
			CFlowerStatus = Line;
			Theta = atan2(TransCurrentY,TransCurrentX);
			Fly2X = radius*cos(Theta+3.14)+waypoints[Center].x;
			Fly2Y = radius*sin(Theta+3.14)+waypoints[Center].y;
			FlyFromX = estimator_x;
			FlyFromY = estimator_y;
			nav_init_stage();
		}
		break;
	case Line:
		nav_route_xy(FlyFromX,FlyFromY,Fly2X,Fly2Y);
		if(!InCircle)
		{
			CFlowerStatus = Circle;
			CircleTheta = nav_radius/radius;
			CircleX = radius*cos(Theta+3.14-CircleTheta)+waypoints[Center].x;
			CircleY = radius*sin(Theta+3.14-CircleTheta)+waypoints[Center].y;
			nav_init_stage();
		}
		break;
	case Circle:
		nav_circle_XY(CircleX, CircleY, nav_radius);
		if(InCircle)
		{
			EdgeCurrentX = waypoints[Edge].x - waypoints[Center].x;
			EdgeCurrentY = waypoints[Edge].y - waypoints[Center].y;	
			radius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);
			if(DistanceFromCenter > radius)
				CFlowerStatus = Outside;
			else
				CFlowerStatus = Line;
			Theta = atan2(TransCurrentY,TransCurrentX);
			Fly2X = radius*cos(Theta+3.14)+waypoints[Center].x;
			Fly2Y = radius*sin(Theta+3.14)+waypoints[Center].y;
			FlyFromX = estimator_x;
			FlyFromY = estimator_y;
			nav_init_stage();
		}
		break;

	}
	return TRUE;
} 

/************** OSAM Takeoff **********************************************/

/** Takeoff functions for bungee takeoff. 	
Run initialize function when the plane is on the bungee, the bungee is fully extended and you are ready to 
launch the plane. After initialized, the plane will follow a line drawn by the position of the plane on initialization and the 
position of the bungee (given in the arguments). Once the plane crosses the throttle line, which is perpendicular to the line the plane is following, 
and intersects the position of the bungee (plus or minus a fixed distance (TakeOff_Distance in airframe file) from the bungee just in case the bungee doesn't release directly above the bungee) the prop will come on. The plane will then continue to follow the line until it has reached a specific
height (defined in as Takeoff_Height in airframe file) above the bungee waypoint and speed (defined as Takeoff_Speed in the airframe file).

<section name="Takeoff" prefix="Takeoff_">
  <define name="Height" value="30" unit="m"/>
  <define name="Speed" value="15" unit="m/s"/>
  <define name="Distance" value="10" unit="m"/>
</section>
 */

enum TakeoffStatus { Launch, Throttle, Finished };
static enum TakeoffStatus CTakeoffStatus;
float throttlePx;
float throttlePy;
float initialx;
float initialy;
float ThrottleSlope;
bool_t AboveLine;
float BungeeAlt;

bool_t InitializeBungeeTakeoff(uint8_t BungeeWP)
{
	float ThrottleB;

	initialx = estimator_x;
	initialy = estimator_y;

	//Takeoff_Distance can only be positive
	float TDistance = abs(Takeoff_Distance);

	//Translate initial position so that the position of the bungee is (0,0)
	float Currentx = initialx-(waypoints[BungeeWP].x);
	float Currenty = initialy-(waypoints[BungeeWP].y);

	//Record bungee alt (which should be the ground alt at that point)
	BungeeAlt = waypoints[BungeeWP].a;

	//Find Launch line slope and Throttle line slope
	float MLaunch = Currenty/Currentx;
	
	//Find Throttle Point (the point where the throttle line and launch line intersect)
	if(Currentx < 0)
		throttlePx = TDistance/sqrt(MLaunch*MLaunch+1);
	else
		throttlePx = -(TDistance/sqrt(MLaunch*MLaunch+1));

	if(Currenty < 0)
		throttlePy = sqrt((TDistance*TDistance)-(throttlePx*throttlePx));
	else
		throttlePy = -sqrt((TDistance*TDistance)-(throttlePx*throttlePx));

	//Find ThrottleLine
	ThrottleSlope = -MLaunch;
	ThrottleB = (throttlePy - (ThrottleSlope*throttlePx));

	//Determine whether the UAV is below or above the throttle line
	if(Currenty > ((ThrottleSlope*Currentx)+ThrottleB))
		AboveLine = TRUE;
	else
		AboveLine = FALSE;	

	//Enable Launch Status and turn kill throttle on
	CTakeoffStatus = Launch;
	kill_throttle = 1;

	//Translate the throttle point back
	throttlePx = throttlePx+(waypoints[BungeeWP].x);
	throttlePy = throttlePy+(waypoints[BungeeWP].y);

	return FALSE;
}

bool_t BungeeTakeoff(void)
{
	//Translate current position so Throttle point is (0,0)
	float Currentx = estimator_x-throttlePx;
	float Currenty = estimator_y-throttlePy;
	bool_t CurrentAboveLine;
	
	NavVerticalAutoThrottleMode(0);
  	NavVerticalAltitudeMode(BungeeAlt+Takeoff_Height, 0.);

	switch(CTakeoffStatus)
	{
	case Launch:
		//Follow Launch Line
		nav_route_xy(initialx,initialy,throttlePx,throttlePy);

		//Find out if the UAV is currently above the line
		if(Currenty > (ThrottleSlope*Currentx))
			CurrentAboveLine = TRUE;
		else
			CurrentAboveLine = FALSE;

		//Find out if UAV has crossed the line
		if(AboveLine != CurrentAboveLine)
		{
			CTakeoffStatus = Throttle;
			kill_throttle = 0;
		}
		break;
	case Throttle:
		//Follow Launch Line
		nav_route_xy(initialx,initialy,throttlePx,throttlePy);
		
		if((estimator_z > BungeeAlt+Takeoff_Height) && (estimator_hspeed_mod > Takeoff_Speed))
		{
			CTakeoffStatus = Finished;
			return FALSE;
		}			
		else
		{
			return TRUE;
		}
		break;
	default:
		break;
	}
	return TRUE;
}
