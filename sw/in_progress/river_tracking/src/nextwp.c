/*This module will do a bunch of fancy opencv stuff to figure out the
next waypoint.
 */

#include "waypoint.h"


Waypoint get_next_waypoint() {
    Waypoint next_wp;

    /* Insert all sorts of opencv magic here */
    
    next_wp.ac_id = 1;
    next_wp.wp = 3;
    next_wp.alt = 1420.;
    next_wp.lat = 41.820258;
    next_wp.lon = -111.98964;
    return next_wp;
}
