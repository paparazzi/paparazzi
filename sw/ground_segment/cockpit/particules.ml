open Printf
open Latlong

(* 255 -> red, 0 -> blue *)
let color_name = fun x ->
  let x = if x < 0 then 0 else if x > 255 then 255 else x in
  sprintf "#%02x%02x%02x" x 0 (255-x)


let fos = float_of_string
let ios = int_of_string

let particules = (Hashtbl.create 97: (int, GnomeCanvas.re_p GnoCanvas.item) Hashtbl.t)

let move_particule = fun (geomap:MapCanvas.widget) id geo value ->
  let fill_color = color_name value in
  if geomap#georef <> None then
    try
      let item = Hashtbl.find particules id in
      geomap#move_item item geo;
      item#set [`FILL_COLOR fill_color]
    with
        Not_found ->
          let group = geomap#background in
          let p = GnoCanvas.ellipse ~fill_color ~props:[`WIDTH_PIXELS 1; `OUTLINE_COLOR "black"] ~x1:(-3.) ~y1:(-3.) ~x2:3. ~y2:3. group in
    (* geomap#circle ~group ~fill_color:"red" geo 10. in *)
          p#raise_to_top ();
          Hashtbl.add particules id (p:>GnomeCanvas.re_p GnoCanvas.item)

let list_separator = Str.regexp ","



let listen = fun (geomap:MapCanvas.widget) ->
  let plumes_msg = fun _sender vs ->
    let split_val = fun tag -> Str.split list_separator (Pprz.string_assoc tag vs) in
    let ids = split_val "ids"
    and xs = split_val "lats"
    and ys = split_val "longs"
    and vs = split_val "values" in

    let rec loop = fun ids xs ys vs ->
      match ids, xs, ys, vs with
          [], [], [], [] -> ()
        | id::ids, x::xs, y::ys, v::vs ->
          let id = int_of_string id
          and wgs84 = {posn_lat=(Deg>>Rad)(fos x); posn_long=(Deg>>Rad)(fos y)} in
          move_particule geomap id wgs84 (ios v);
          loop ids xs ys vs
        | _ -> failwith "Particules.listen loop"
    in
    loop ids xs ys vs
  in

  Live.safe_bind "PLUMES" plumes_msg
