type us = int

let pi = 4. *. atan 1.
let rec norm_angle = fun x ->
  if x > pi then norm_angle (x-.2.*.pi)
  else if x < -.pi then norm_angle (x+.2.*.pi)
  else x
  
let deg = fun rad -> rad /. pi *. 180.

let rad_of_deg = fun x -> x /. 180. *. pi

let set_float = fun option var name ->
  (option, Arg.Set_float var, Printf.sprintf "%s (%f)" name !var)
let set_string = fun option var name ->
  (option, Arg.Set_string var, Printf.sprintf "%s (%s)" name !var)
