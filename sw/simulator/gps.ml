open Stdlib
open Latlong

type state = {
    wgs84 : Latlong.geographic;
    alt : float;
    time : float;
    climb : float;
    gspeed : float;
    course : float
  }


let earth_radius = 6378388.


let state = fun lat0 lon0 alt0 ->
  let last_x = ref 0. and last_y = ref 0. 
  and last_t = ref 0. and last_z = ref 0. in

  fun (x, y, z) t ->
   let dx = x -. !last_x
    and dy = y -. !last_y
    and dt = t -. !last_t in
    let gspeed = sqrt (dx*.dx +. dy*.dy) /. dt
    and course = norm_angle (pi/.2. -. atan2 dy dx)
    and climb = (z -. !last_z) /. dt in

    let lat = lat0 +. y /. earth_radius
    and long = lon0 +. x /.earth_radius /. cos lat0
    and alt = alt0 +. z in

    last_x := x;
    last_y := y;
    last_z := z;
    last_t := t;

    let course = if course < 0. then course +. 2. *. pi else course in (* ???? *)
    
    {
     wgs84 = { posn_lat=lat;posn_long=long };
     alt = alt0 +. z;
     time = t;
     climb = climb;
     gspeed = gspeed;
     course = course
   }
      
