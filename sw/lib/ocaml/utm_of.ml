open Latlong

let _ =
  let f = fun i -> (Deg>>Rad)(float_of_string Sys.argv.(i)) in
  let utm = utm_of WGS84 { posn_lat = f 1 ; posn_long = f 2 } in
  Printf.printf "%d %d\n" utm.utm_x utm.utm_y
