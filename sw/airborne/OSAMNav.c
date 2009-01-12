#include "OSAMNav.h"

/************** Flower Navigation **********************************************/

/** Makes a flower pattern. 
	CenterWP is the center of the flower. The Navigation Height is taken from this waypoint.
	EdgeWP defines the radius of the flower (distance from CenterWP to EdgeWP)	
*/

enum FlowerStatus { Outside, FlowerLine, Circle };
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
static float FlowerTheta;
static float Flowerradius;
static uint8_t Center;
static uint8_t Edge;

bool_t InitializeFlower(uint8_t CenterWP, uint8_t EdgeWP)
{
	Center = CenterWP;
	Edge = EdgeWP;

	EdgeCurrentX = waypoints[Edge].x - waypoints[Center].x;
	EdgeCurrentY = waypoints[Edge].y - waypoints[Center].y;
	
	Flowerradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);

	TransCurrentX = estimator_x - waypoints[Center].x;
	TransCurrentY = estimator_y - waypoints[Center].y;
	DistanceFromCenter = sqrt(TransCurrentX*TransCurrentX+TransCurrentY*TransCurrentY);

	FlowerTheta = atan2(TransCurrentY,TransCurrentX);
	Fly2X = Flowerradius*cos(FlowerTheta+3.14)+waypoints[Center].x;
	Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+waypoints[Center].y;
	FlyFromX = estimator_x;
	FlyFromY = estimator_y;

	if(DistanceFromCenter > Flowerradius)
		CFlowerStatus = Outside;
	else
		CFlowerStatus = FlowerLine;

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

	if(DistanceFromCenter > Flowerradius)
		InCircle = FALSE;

	NavVerticalAutoThrottleMode(0); /* No pitch */
  	NavVerticalAltitudeMode(waypoints[Center].a, 0.);

	switch(CFlowerStatus)
	{
	case Outside:		
		nav_route_xy(FlyFromX,FlyFromY,Fly2X,Fly2Y);
		if(InCircle)
		{
			CFlowerStatus = FlowerLine;
			FlowerTheta = atan2(TransCurrentY,TransCurrentX);
			Fly2X = Flowerradius*cos(FlowerTheta+3.14)+waypoints[Center].x;
			Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+waypoints[Center].y;
			FlyFromX = estimator_x;
			FlyFromY = estimator_y;
			nav_init_stage();
		}
		break;
	case FlowerLine:
		nav_route_xy(FlyFromX,FlyFromY,Fly2X,Fly2Y);
		if(!InCircle)
		{
			CFlowerStatus = Circle;
			CircleTheta = nav_radius/Flowerradius;
			CircleX = Flowerradius*cos(FlowerTheta+3.14-CircleTheta)+waypoints[Center].x;
			CircleY = Flowerradius*sin(FlowerTheta+3.14-CircleTheta)+waypoints[Center].y;
			nav_init_stage();
		}
		break;
	case Circle:
		nav_circle_XY(CircleX, CircleY, nav_radius);
		if(InCircle)
		{
			EdgeCurrentX = waypoints[Edge].x - waypoints[Center].x;
			EdgeCurrentY = waypoints[Edge].y - waypoints[Center].y;	
			Flowerradius = sqrt(EdgeCurrentX*EdgeCurrentX+EdgeCurrentY*EdgeCurrentY);
			if(DistanceFromCenter > Flowerradius)
				CFlowerStatus = Outside;
			else
				CFlowerStatus = FlowerLine;
			FlowerTheta = atan2(TransCurrentY,TransCurrentX);
			Fly2X = Flowerradius*cos(FlowerTheta+3.14)+waypoints[Center].x;
			Fly2Y = Flowerradius*sin(FlowerTheta+3.14)+waypoints[Center].y;
			FlyFromX = estimator_x;
			FlyFromY = estimator_y;
			nav_init_stage();
		}
		break;

	}
	return TRUE;
} 

/************** Bungee Takeoff **********************************************/

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
static float throttlePx;
static float throttlePy;
static float initialx;
static float initialy;
static float ThrottleSlope;
static bool_t AboveLine;
static float BungeeAlt;

bool_t InitializeBungeeTakeoff(uint8_t BungeeWP)
{
	float ThrottleB;

	initialx = estimator_x;
	initialy = estimator_y;

	//Takeoff_Distance can only be positive
	float TDistance = fabs(Takeoff_Distance);

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
		kill_throttle = 1;

		//Find out if the UAV is currently above the line
		if(Currenty > (ThrottleSlope*Currentx))
			CurrentAboveLine = TRUE;
		else
			CurrentAboveLine = FALSE;

		//Find out if UAV has crossed the line
		if(AboveLine != CurrentAboveLine && estimator_hspeed_mod > 5)
		{
			CTakeoffStatus = Throttle;
			kill_throttle = 0;
		}
		break;
	case Throttle:
		//Follow Launch Line
		nav_route_xy(initialx,initialy,throttlePx,throttlePy);
		kill_throttle = 0;
		
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

/************** Polygon Survey **********************************************/

/** This routine will cover the enitre area of any Polygon defined in the flightplan which is a convex polygon.
 */

enum SurveyStatus { Init, Entry, Sweep, SweepCircle };
static enum SurveyStatus CSurveyStatus;
static struct Point2D PolygonCenter;
static struct Line Edges[PolygonSize];
static float EdgeMaxY[PolygonSize];
static float EdgeMinY[PolygonSize];
static float SurveyTheta;
static float dSweep;
static float SurveyRadius;
static struct Point2D SurveyToWP;
static struct Point2D SurveyFromWP;
static struct Point2D SurveyCircle;
static uint8_t SurveyEntryWP;
static uint8_t SurveySize;
static float SurveyCircleQdr;
static float MinY;
static float MaxY;
uint16_t PolySurveySweepNum;
uint16_t PolySurveySweepBackNum;

bool_t InitializePolygonSurvey(uint8_t EntryWP, uint8_t Size, float sw, float Orientation)
{
	PolygonCenter.x = 0;
	PolygonCenter.y = 0;
	int i = 0;
	float ys = 0;
	float Xpmin = MaxFloat;
	float Xnmin = -MinFloat;
	static struct Point2D EntryPoint;
	float LeftYInt;
	float RightYInt;
	float temp;
	float XIntercept1 = 0;
	float XIntercept2 = 0;

	SurveyTheta = RadOfDeg(Orientation);	
	PolySurveySweepNum = 0;
	PolySurveySweepBackNum = 0;

	SurveyEntryWP = EntryWP;
	SurveySize = Size;

	struct Point2D Corners[PolygonSize];

	CSurveyStatus = Init;

	//Don't initialize if Polygon is too big or if the orientation is not between 0 and 90
	if(Size <= PolygonSize && Orientation >= 0 && Orientation <= 90)
	{
		//Find Polygon Center
		for(i = EntryWP; i < Size+EntryWP; i++)
		{
			PolygonCenter.x = PolygonCenter.x + waypoints[i].x;
			PolygonCenter.y = PolygonCenter.y + waypoints[i].y;
		}

		PolygonCenter.x = PolygonCenter.x/Size;
		PolygonCenter.y = PolygonCenter.y/Size;

		//Initialize Corners
		for(i = 0; i < Size; i++)
		{
			Corners[i].x = waypoints[i+EntryWP].x;
			Corners[i].y = waypoints[i+EntryWP].y;
		}

		//Rotate and Translate Corners
		for(i = 0; i < Size; i++)
			TranslateAndRotateFromWorld(&Corners[i], SurveyTheta, PolygonCenter.x, PolygonCenter.y);

		//Rotate and Translate Entry Point
		EntryPoint.x = Corners[0].x;
		EntryPoint.y = Corners[0].y;

		//Find min and max y
		MinY = Corners[0].y;
		MaxY = Corners[0].y;
		for(i = 1; i < Size; i++)
		{
			if(Corners[i].y > MaxY)
				MaxY = Corners[i].y;

			if(Corners[i].y < MinY)
				MinY = Corners[i].y;
		}

		//Find polygon edges
		for(i = 0; i < Size; i++)
		{
			if(i == 0)
				if(Corners[Size-1].x == Corners[i].x) //Don't divide by zero!
					Edges[i].m = MaxFloat;
				else
					Edges[i].m = ((Corners[Size-1].y-Corners[i].y)/(Corners[Size-1].x-Corners[i].x));
			else
				if(Corners[i].x == Corners[i-1].x)
					Edges[i].m = MaxFloat;
				else
					Edges[i].m = ((Corners[i].y-Corners[i-1].y)/(Corners[i].x-Corners[i-1].x));

			//Edges[i].m = MaxFloat;
			Edges[i].b = (Corners[i].y - (Corners[i].x*Edges[i].m));
		}

		//Find Min and Max y for each line
		FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[0], Edges[1]);
		FindInterceptOfTwoLines(&temp, &RightYInt, Edges[0], Edges[Size-1]);

		if(LeftYInt > RightYInt)
		{
			EdgeMaxY[0] = LeftYInt;
			EdgeMinY[0] = RightYInt;
		}
		else
		{
			EdgeMaxY[0] = RightYInt;
			EdgeMinY[0] = LeftYInt;
		}

		for(i = 1; i < Size-1; i++)
		{
			FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[i], Edges[i+1]);
			FindInterceptOfTwoLines(&temp, &RightYInt, Edges[i], Edges[i-1]);

			if(LeftYInt > RightYInt)
			{
				EdgeMaxY[i] = LeftYInt;
				EdgeMinY[i] = RightYInt;
			}
			else
			{
				EdgeMaxY[i] = RightYInt;
				EdgeMinY[i] = LeftYInt;
			}
		}

		FindInterceptOfTwoLines(&temp, &LeftYInt, Edges[Size-1], Edges[0]);
		FindInterceptOfTwoLines(&temp, &RightYInt, Edges[Size-1], Edges[Size-2]);

		if(LeftYInt > RightYInt)
		{
			EdgeMaxY[Size-1] = LeftYInt;
			EdgeMinY[Size-1] = RightYInt;
		}
		else
		{
			EdgeMaxY[Size-1] = RightYInt;
			EdgeMinY[Size-1] = LeftYInt;
		}

		//Find amount to increment by every sweep
		if(EntryPoint.y >= 0)
			dSweep = -sw;
		else
			dSweep = sw;

		//CircleQdr tells the plane when to exit the circle
		if(dSweep >= 0)
			SurveyCircleQdr = -DegOfRad(SurveyTheta);
		else
			SurveyCircleQdr = 180-DegOfRad(SurveyTheta);

		//If the Sweep distance makes a circle smaller than the nav radius, the sweep distance will be multiplied by two
		// and the plane will cover the sweeps inbetween on the way back
		//if(Sweep/2 < nav_radius)
		//	dSweep = dSweep*2;
		//**Didin't like it

		//Find out which way to circle
		if(EntryPoint.x >= 0)
			SurveyRadius = -dSweep/2;
		else
			SurveyRadius = dSweep/2;

		//Find y value of the first sweep
		ys = EntryPoint.y+(dSweep/2);

		//Find the edges which intercet the sweep line first
		for(i = 0; i < SurveySize; i++)
		{
			if(EdgeMinY[i] <= ys && EdgeMaxY[i] > ys)
			{
				XIntercept2 = XIntercept1;
				XIntercept1 = EvaluateLineForX(ys, Edges[i]);
			}
		}

		//Find out which intercept is smaller than the other
		if(XIntercept1 > XIntercept2)
		{
			Xnmin = XIntercept2;
			Xpmin = XIntercept1;
		}
		else
		{
			Xnmin = XIntercept1;
			Xpmin = XIntercept2;
		}
	
		//Find point to come from and point to go to
		if(fabs(EntryPoint.x - XIntercept2) <= fabs(EntryPoint.x - XIntercept1))
		{
			SurveyToWP.x = XIntercept1;
			SurveyToWP.y = ys;

			SurveyFromWP.x = XIntercept2;
			SurveyFromWP.y = ys;
		}
		else
		{
			SurveyToWP.x = XIntercept2;
			SurveyToWP.y = ys;

			SurveyFromWP.x = XIntercept1;
			SurveyFromWP.y = ys;
		}

		//Find the entry circle
		SurveyCircle.x = SurveyFromWP.x;
		SurveyCircle.y = EntryPoint.y;

		//Go into entry circle state
		CSurveyStatus = Entry;	
	}

	return FALSE;
}

bool_t PolygonSurvey(void)
{
	struct Point2D C;
	struct Point2D ToP;
	struct Point2D FromP;
	float ys;
	static struct Point2D LastPoint;
	int i;
	float Xpmin = MaxFloat;
	float Xnmin = -MinFloat;
	bool_t SweepingBack = FALSE;
	float XIntercept1 = 0;
	float XIntercept2 = 0;
	
	NavVerticalAutoThrottleMode(0); /* No pitch */
  	NavVerticalAltitudeMode(waypoints[SurveyEntryWP].a, 0.);

	switch(CSurveyStatus)
	{
	case Entry:
		//Rotate and translate circle point into real world
		C.x = SurveyCircle.x;
		C.y = SurveyCircle.y;
		RotateAndTranslateToWorld(&C, SurveyTheta, PolygonCenter.x, PolygonCenter.y);

		//follow the circle		
		nav_circle_XY(C.x, C.y, SurveyRadius);

		if(NavQdrCloseTo(SurveyCircleQdr) && NavCircleCount() > 0 && estimator_z > waypoints[SurveyEntryWP].a-10)
		{
			CSurveyStatus = Sweep;
			nav_init_stage();
		}
		break;
	case Sweep:
		//Rotate and Translate Line points into real world
		ToP.x = SurveyToWP.x;
		ToP.y = SurveyToWP.y;
		FromP.x = SurveyFromWP.x;
		FromP.y = SurveyFromWP.y;
		
		RotateAndTranslateToWorld(&ToP, SurveyTheta, PolygonCenter.x, PolygonCenter.y);
		RotateAndTranslateToWorld(&FromP, SurveyTheta, PolygonCenter.x, PolygonCenter.y);

		//follow the line
		nav_route_xy(FromP.x,FromP.y,ToP.x,ToP.y);
		if(nav_approaching_xy(ToP.x,ToP.y,FromP.x,FromP.y, 0))
		{
			LastPoint.x = SurveyToWP.x;
			LastPoint.y = SurveyToWP.y;

			if(LastPoint.y+dSweep >= MaxY || LastPoint.y+dSweep <= MinY) //Your out of the Polygon so Sweep Back
			{
				dSweep = -dSweep;
				ys = LastPoint.y+(dSweep/2);

				if(dSweep >= 0)
					SurveyCircleQdr = -DegOfRad(SurveyTheta);
				else
					SurveyCircleQdr = 180-DegOfRad(SurveyTheta);
				SweepingBack = TRUE;
				PolySurveySweepBackNum++;
			}
			else
			{
				SurveyRadius = -SurveyRadius;
			
				//Find y value of the first sweep
				ys = LastPoint.y+dSweep;
			}

			//Find the edges which intercet the sweep line first
			for(i = 0; i < SurveySize; i++)
			{
				if(EdgeMinY[i] < ys && EdgeMaxY[i] >= ys)
				{
					XIntercept2 = XIntercept1;
					XIntercept1 = EvaluateLineForX(ys, Edges[i]);
				}
			}

			//Find out which intercept is small er than the other
			if(XIntercept1 > XIntercept2)
			{
				Xnmin = XIntercept2;
				Xpmin = XIntercept1;
			}
			else
			{
				Xnmin = XIntercept1;
				Xpmin = XIntercept2;
			}

			//Find point to come from and point to go to
			if(fabs(LastPoint.x - XIntercept2) <= fabs(LastPoint.x - XIntercept1))
			{
				SurveyToWP.x = XIntercept1;
				SurveyToWP.y = ys;

				SurveyFromWP.x = XIntercept2;
				SurveyFromWP.y = ys;
			}
			else
			{
				SurveyToWP.x = XIntercept2;
				SurveyToWP.y = ys;

				SurveyFromWP.x = XIntercept1;
				SurveyFromWP.y = ys;
			}

			if(fabs(LastPoint.x) > fabs(SurveyFromWP.x))
				SurveyCircle.x = LastPoint.x;
			else
				SurveyCircle.x = SurveyFromWP.x;

			
			if(!SweepingBack)
				SurveyCircle.y = LastPoint.y+(dSweep/2);
			else
				SurveyCircle.y = LastPoint.y;
			

			//Go into circle state
			CSurveyStatus = SweepCircle;	
			nav_init_stage();

			PolySurveySweepNum++;
		}

		break;
	case SweepCircle:
		//Rotate and translate circle point into real world
		C.x = SurveyCircle.x;
		C.y = SurveyCircle.y;
		RotateAndTranslateToWorld(&C, SurveyTheta, PolygonCenter.x, PolygonCenter.y);

		//follow the circle		
		nav_circle_XY(C.x, C.y, SurveyRadius);

		if(NavQdrCloseTo(SurveyCircleQdr) && NavCircleCount() > 0)
		{
			CSurveyStatus = Sweep;
			nav_init_stage();
		}
		break;
	case Init:
		return FALSE;
	default:
		return FALSE;
	}
	return TRUE;
}

/*
Translates point so (transX, transY) are (0,0) then rotates the point around z by Zrot
*/
void TranslateAndRotateFromWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
	float temp;

	p->x = p->x - transX;
	p->y = p->y - transY;

	temp = p->x;
	p->x = p->x*cos(Zrot)+p->y*sin(Zrot);
	p->y = -temp*sin(Zrot)+p->y*cos(Zrot);
}

/*
Rotates point round z by -Zrot then translates so (0,0) becomes (transX,transY)
*/
void RotateAndTranslateToWorld(struct Point2D *p, float Zrot, float transX, float transY)
{
	float temp = p->x;

	p->x = p->x*cos(Zrot)-p->y*sin(Zrot);
	p->y = temp*sin(Zrot)+p->y*cos(Zrot);

	p->x = p->x + transX;
	p->y = p->y + transY;
}

void FindInterceptOfTwoLines(float *x, float *y, struct Line L1, struct Line L2)
{
	*x = ((L2.b-L1.b)/(L1.m-L2.m));
	*y = L1.m*(*x)+L1.b;
}

float EvaluateLineForY(float x, struct Line L)
{
	return (L.m*x)+L.b;
}

float EvaluateLineForX(float y, struct Line L)
{
	return ((y-L.b)/L.m);
}

float DistanceEquation(struct Point2D p1,struct Point2D p2)
{
	return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}
