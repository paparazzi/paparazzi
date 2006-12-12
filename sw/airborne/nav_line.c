#include "nav_line.h"
#include "nav.h"

/************** Line Navigation **********************************************/



enum line_status { LR12, LQC21, LTC2, LQC22, LR21, LQC12, LTC1, LQC11 };

static enum line_status line_status;
bool_t nav_line_init( void ) {
  line_status = LR12;
  return FALSE;
}

bool_t nav_line(uint8_t l1, uint8_t l2, float radius) {
  float alt = waypoints[l1].a;
  waypoints[l2].a = alt;

  float l2_l1_x = waypoints[l1].x - waypoints[l2].x;
  float l2_l1_y = waypoints[l1].y - waypoints[l2].y;
  float d = sqrt(l2_l1_x*l2_l1_x+l2_l1_y*l2_l1_y);

  /* Unit vector from l1 to l2 */
  float u_x = l2_l1_x / d;
  float u_y = l2_l1_y / d;

  /* The half circle centers and the other leg */
  struct point l2_c1 = { waypoints[l1].x + radius * u_y,
			     waypoints[l1].y + radius * -u_x,
			     alt  };
  struct point l2_c2 = { waypoints[l1].x + 1.732*radius * u_x,
			  waypoints[l1].y + 1.732*radius * u_y,
			  alt  };
 struct point l2_c3 = { waypoints[l1].x + radius * -u_y,
			  waypoints[l1].y + radius * u_x,
			  alt  };
  
  struct point l1_c1 = { waypoints[l2].x + radius * -u_y,
			 waypoints[l2].y + radius * u_x,
			 alt  };
  struct point l1_c2 = { waypoints[l2].x +1.732*radius * -u_x,
			     waypoints[l2].y + 1.732*radius * -u_y,
			     alt  };
 struct point l1_c3 = { waypoints[l2].x + radius * u_y,
			  waypoints[l2].y + radius * -u_x,
			  alt  };
  float qdr_out_2_1 = M_PI/3. - atan2(u_y, u_x);
 
  float qdr_out_2_2 = -M_PI/3. - atan2(u_y, u_x);
 float qdr_out_2_3 = M_PI - atan2(u_y, u_x);
 
  switch (line_status) {

  /* LR12, LQC21, LTC2, LQC22, LR21 LQC12, LTC1, LQC11 */
  case LR12:
  nav_route_xy(waypoints[l2].x, waypoints[l2].y, waypoints[l1].x, waypoints[l1].y);
    if (NavApproaching(l1, CARROT)) { 
      line_status = LQC21;
      nav_init_stage();
    }
    break;
    case LQC21:
  nav_circle_XY(l2_c1.x, l2_c1.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_1)-10)) {
      line_status = LTC2;
      nav_init_stage();
    }
    break;
     case LTC2:
  nav_circle_XY(l2_c2.x, l2_c2.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_2)+10)) {
      line_status = LQC22;
      nav_init_stage();
    }
    break;
      case LQC22:
 nav_circle_XY(l2_c3.x, l2_c3.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_3)-10)) {
      line_status = LR21;
      nav_init_stage();
    }
    break;
    case LR21:
  nav_route_xy(waypoints[l1].x, waypoints[l1].y,waypoints[l2].x, waypoints[l2].y);
    if (NavApproaching(l2, CARROT)) { 
      line_status = LQC12;
      nav_init_stage();
    }
    break;
    case LQC12:
  nav_circle_XY(l1_c1.x, l1_c1.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_1 + M_PI)-10)) {
      line_status = LTC1;
      nav_init_stage();
    }
    break;
       case LTC1:
   nav_circle_XY(l1_c2.x, l1_c2.y, -radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_2 + M_PI)+10)) {
      line_status = LQC11;
      nav_init_stage();
    }
    break;
       case LQC11:
  nav_circle_XY(l1_c3.x, l1_c3.y, radius);
    if (NavQdrCloseTo(DegOfRad(qdr_out_2_3 + M_PI)-10)) {
      line_status = LR12;
      nav_init_stage();
    }
  }
  return TRUE;
}
