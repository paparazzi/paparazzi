(*
 * Flight plan preprocessing (from XML to C)
 *
 * Copyright (C) 2004-2008 ENAC, Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *)

open Printf
open Latlong

module G2D = Geometry_2d

(* From mapFP.ml to avoid complex linking *)
let georef_of_xml = fun xml ->
  let lat0 = Latlong.deg_of_string (ExtXml.attrib xml "lat0")
  and lon0 = Latlong.deg_of_string (ExtXml.attrib xml "lon0") in
  { posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 }

let sof = string_of_float
let soi = string_of_int

let index_of_blocks = ref []

let check_expressions = ref false

let cm = fun x -> 100. *. x

let parse = fun s ->
  let e = Fp_proc.parse_expression s in
  if !check_expressions then begin
    let unexpected = fun kind x ->
      fprintf stderr "Parsing error in '%s': unexpected %s: '%s' \n" s kind x;
      exit 1 in
    begin
      try
        Expr_syntax.check_expression e
      with
          Expr_syntax.Unknown_operator x -> unexpected "operator" x
        | Expr_syntax.Unknown_ident x -> unexpected "ident" x
        | Expr_syntax.Unknown_function x -> unexpected "function" x
    end
  end;
  Expr_syntax.sprint ~call_assoc:("IndexOfBlock", !index_of_blocks) e

let parsed_attrib = fun xml a ->
  parse (ExtXml.attrib xml a)

let gen_label =
  let x = ref 0 in
  fun p -> incr x; sprintf "%s_%d" p !x

(** Formatting with a margin *)
let margin = ref 0
let step = 2

let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun out f ->
  fprintf out "%s" (String.make !margin ' ');
  fprintf out f


let float_attrib = fun xml a ->
  try
    float_of_string (Xml.attrib xml a)
  with
      Failure _ ->
        failwith (sprintf "Float expected in attribute '%s' from %s" a (Xml.to_string_fmt xml))
let name_of = fun wp -> ExtXml.attrib wp "name"


type ref_type = UTM | LTP

type ref0 = {
    frame : ref_type;
    utm0 : utm;
    ecef0 : ecef;
    geo0 : geographic
}

let ground_alt = ref 0.
let security_height = ref 0.
let fp_wgs84 = ref { posn_lat = 0.; posn_long = 0.}

let check_altitude_srtm = fun a x wgs84 ->
  Srtm.add_path (Env.paparazzi_home ^ "/data/srtm");
  try
    let srtm_alt = float (Srtm.of_wgs84 wgs84) in
    if a < srtm_alt then begin (* Not fully correct, Flightplan "alt" is not alt as we know it *)
      fprintf stderr "MAJOR NOTICE: below SRTM ground altitude (%.0f<%.0f) in %s\n" a srtm_alt (Xml.to_string x)
    end
  with Srtm.Tile_not_found e ->
    fprintf stderr "No SRTM data found to check altitude.\n"

let check_altitude = fun a x ->
  if a < !ground_alt +. !security_height then begin
    fprintf stderr "NOTICE: low altitude (%.0f<%.0f+%.0f) in %s\n" a !ground_alt !security_height (Xml.to_string x)
  end


(** Computes "x" and "y" attributes if "lat" and "lon" are available *)
let localize_waypoint = fun rel_utm_of_wgs84 waypoint ->
  try
    let (x, y) =
      rel_utm_of_wgs84
        (Latlong.make_geo_deg
           (Latlong.deg_of_string (Xml.attrib waypoint "lat"))
           (Latlong.deg_of_string (Xml.attrib waypoint "lon"))) in
    let x = sprintf "%.2f" x and y = sprintf "%.2f" y in
    ExtXml.subst_attrib "y" y (ExtXml.subst_attrib "x" x waypoint)
  with
      Xml.No_attribute "lat" | Xml.No_attribute "lon" ->
        waypoint

let wgs84_of_x_y = fun ref0 x y alt ->
  match ref0.frame with
    | UTM ->
      Latlong.of_utm Latlong.WGS84 (Latlong.utm_add ref0.utm0 (x, y))
    | LTP ->
      let ned = make_ned [|y; x; -. (float_of_string alt -. !ground_alt)|] in
      let ecef = ecef_of_ned ref0.ecef0 ned in
      let (geo, _) = Latlong.geo_of_ecef Latlong.WGS84 ecef in
      geo

let print_waypoint_utm = fun out ref0 default_alt waypoint ->
  let (x, y) = (float_attrib waypoint "x", float_attrib waypoint "y")
  and alt = try sof (float_attrib waypoint "height" +. !ground_alt) with _ -> default_alt in
  let alt = try Xml.attrib waypoint "alt" with _ -> alt in
  check_altitude (float_of_string alt) waypoint;
  let (x, y) = match ref0.frame with
    | UTM -> (x, y)
    | LTP ->
      let wgs84 = wgs84_of_x_y ref0 x y alt in
      let utm = utm_of ~zone:ref0.utm0.utm_zone Latlong.WGS84 wgs84 in
      (utm.utm_x -. ref0.utm0.utm_x, utm.utm_y -. ref0.utm0.utm_y)
  in
  fprintf out " {%.1f, %.1f, %s},\\\n" x y alt

let print_waypoint_enu = fun out ref0 default_alt waypoint ->
  let (x, y) = (float_attrib waypoint "x", float_attrib waypoint "y")
  and alt = try sof (float_attrib waypoint "height" +. !ground_alt) with _ -> default_alt in
  let alt = try Xml.attrib waypoint "alt" with _ -> alt in
  let ned = match ref0.frame with
    | UTM ->
      let ecef = Latlong.ecef_of_geo Latlong.WGS84 (Latlong.of_utm Latlong.WGS84 (Latlong.utm_add ref0.utm0 (x, y))) (float_of_string alt) in
      Latlong.array_of_ned (Latlong.ned_of_ecef ref0.ecef0 ecef)
    | LTP ->
      [|y; x; -. (float_of_string alt -. !ground_alt)|]
  in
  fprintf out " {%.2f, %.2f, %.2f}, /* ENU in meters  */ \\\n" ned.(1) ned.(0) (-.ned.(2))

let convert_angle = fun rad -> Int64.of_float (1e7 *. (Rad>>Deg)rad)

let print_waypoint_lla = fun out ref0 default_alt waypoint ->
  let (x, y) = (float_attrib waypoint "x", float_attrib waypoint "y")
  and alt = try sof (float_attrib waypoint "height" +. !ground_alt) with _ -> default_alt in
  let alt = try Xml.attrib waypoint "alt" with _ -> alt in
  let wgs84 = wgs84_of_x_y ref0 x y alt in
  fprintf out " {.lat=%Ld, .lon=%Ld, .alt=%.0f}, /* 1e7deg, 1e7deg, mm (above NAV_MSL0, local msl=%.2fm) */ \\\n" (convert_angle wgs84.posn_lat) (convert_angle wgs84.posn_long) (1000. *. float_of_string alt) (Egm96.of_wgs84 wgs84)

let print_waypoint_lla_wgs84 = fun out ref0 default_alt waypoint ->
  let (x, y) = (float_attrib waypoint "x", float_attrib waypoint "y")
  and alt = try sof (float_attrib waypoint "height" +. !ground_alt) with _ -> default_alt in
  let alt = try Xml.attrib waypoint "alt" with _ -> alt in
  let wgs84 = wgs84_of_x_y ref0 x y alt in
  if Srtm.available wgs84 then
    check_altitude_srtm (float_of_string alt) waypoint wgs84;
  let alt = float_of_string alt +. Egm96.of_wgs84 wgs84 in
  fprintf out " {.lat=%Ld, .lon=%Ld, .alt=%.0f}, /* 1e7deg, 1e7deg, mm (above WGS84 ref ellipsoid) */ \\\n" (convert_angle wgs84.posn_lat) (convert_angle wgs84.posn_long) (1000. *. alt)

let print_waypoint_global = fun out waypoint ->
  try
    let (_, _) = (float_attrib waypoint "lat", float_attrib waypoint "lon") in
    fprintf out " TRUE, \\\n"
  with _ -> fprintf out " FALSE, \\\n"

let get_index_block = fun x ->
  try
    List.assoc x !index_of_blocks
  with
      Not_found -> failwith (sprintf "Unknown block: '%s'" x)

let print_exception = fun out x ->
  let c = parsed_attrib x "cond" in
  let i = get_index_block (ExtXml.attrib x "deroute") in
  begin
  try
    let f =  ExtXml.attrib x "exec" in
    lprintf out "if ((nav_block != %d) && %s) {%s; GotoBlock(%d); return; }\n" i c f i
  with
    ExtXml.Error _ -> (
     lprintf out "if ((nav_block != %d) && %s) { GotoBlock(%d); return; }\n" i c i
    )
  end


let element = fun a b c -> Xml.Element (a, b, c)
let goto l = element "goto" ["name",l] []
let exit_block = element "exit_block" [] []
let home_block = Xml.parse_string "<block name=\"HOME\"><home/></block>"

let stage = ref 0

let output_label out l = lprintf out "Label(%s)\n" l

let get_index_waypoint = fun x l ->
  try
    string_of_int (List.assoc x l)
  with
      Not_found -> failwith (sprintf "Unknown waypoint: %s" x)

let pprz_throttle = fun s ->
  begin
    try
      let g = float_of_string s in
      if g < 0. || g > 1. then
        failwith "throttle must be > 0 and < 1"
    with
        Failure _ -> () (* No possible check on expression *)
  end;
  sprintf "9600*(%s)" s


(********************* Vertical control ********************************************)
let output_vmode = fun out stage_xml wp last_wp ->
  let pitch = try Xml.attrib stage_xml "pitch" with _ -> "0.0" in
  let t = ExtXml.attrib_or_default stage_xml "nav_type" "Nav" in
  if String.lowercase_ascii (Xml.tag stage_xml) <> "manual"
  then begin
    if pitch = "auto"
    then begin
      lprintf out "%sVerticalAutoPitchMode(%s);\n" t (pprz_throttle (parsed_attrib stage_xml "throttle"))
    end else begin
      lprintf out "%sVerticalAutoThrottleMode(RadOfDeg(%s));\n" t (parse pitch);
    end
  end;

  let vmode = try ExtXml.attrib stage_xml "vmode" with _ -> "alt" in
  begin
    match vmode with
        "climb" ->
          lprintf out "%sVerticalClimbMode(%s);\n" t (parsed_attrib stage_xml "climb")
      | "alt" ->
        let alt =
          try
            let a = parsed_attrib stage_xml "alt" in
            begin
              try
                check_altitude (float_of_string a) stage_xml
              with
        (* Impossible to check the altitude on an expression: *)
                  Failure _ -> ()
            end;
            a
          with _ ->
            try
              let h = parsed_attrib stage_xml "height" in
              begin
                try
                  check_altitude ((float_of_string h) +. !ground_alt) stage_xml
                with
          (* Impossible to check the altitude on an expression: *)
                    Failure _ -> ()
              end;
              sprintf "Height(%s)" h
            with _ ->
              if wp = ""
              then failwith "alt or waypoint required in alt vmode"
              else sprintf "WaypointAlt(%s)" wp in
        lprintf out "%sVerticalAltitudeMode(%s, 0.);\n" t alt;
      | "xyz" -> () (** Handled in Goto3D() *)
      | "glide" ->
        lprintf out "%sGlide(%s, %s);\n" t last_wp wp
      | "throttle" ->
        if (pitch = "auto") then
          failwith "auto pich mode not compatible with vmode=throttle";
        lprintf out "%sVerticalThrottleMode(%s);\n" t (pprz_throttle (parsed_attrib stage_xml "throttle"))
      | x -> failwith (sprintf "Unknown vmode '%s'" x)
  end;
  vmode

(****************** Horizontal control *********************************************)
let output_hmode out x wp last_wp =
  let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
  let p = try ", " ^ (Xml.attrib x "nav_params") with _ -> "" in
  try
    let hmode = ExtXml.attrib x "hmode" in
    begin
      match hmode with
          "route" ->
            if last_wp = "last_wp" then
              fprintf stderr "NOTICE: Deprecated use of 'route' using last waypoint in %s\n"(Xml.to_string x);
            lprintf out "%sSegment(%s, %s%s);\n" t last_wp wp p
        | "direct" -> lprintf out "%sGotoWaypoint(%s%s);\n" t wp p
        | x -> failwith (sprintf "Unknown hmode '%s'" x)
    end;
    hmode
  with
      ExtXml.Error _ -> lprintf out "%sGotoWaypoint(%s%s);\n" t wp p; "direct" (* Default behaviour *)




let rec index_stage = fun x ->
  begin
    match Xml.tag x with
        "for" ->
          incr stage; (* Init of i *)
          incr stage;
          let n = !stage in
          let l = List.map index_stage (Xml.children x) in
          incr stage; (* To count the loop stage *)
          Xml.Element (Xml.tag x, Xml.attribs x@["no", soi n], l)
      | "while" ->
        incr stage;
        let n = !stage in
        let l = List.map index_stage (Xml.children x) in
        incr stage; (* To count the loop stage *)
        Xml.Element (Xml.tag x, Xml.attribs x@["no", soi n], l)
      | "return" | "goto"  | "deroute" | "exit_block" | "follow" | "call" | "call_once" | "home"
      | "heading" | "attitude" | "manual" | "go" | "stay" | "xyz" | "set" | "circle" ->
        incr stage;
        Xml.Element (Xml.tag x, Xml.attribs x@["no", soi !stage], Xml.children x)
      | "survey_rectangle" | "eight" | "oval"->
        incr stage; incr stage;
        Xml.Element (Xml.tag x, Xml.attribs x@["no", soi !stage], Xml.children x)
      | "exception" ->
        x
      | s -> failwith (sprintf "Unknown stage: %s\n" s)
  end


let inside_function = fun name -> "Inside" ^ String.capitalize_ascii name

(* pre call utility function *)
let fp_pre_call = fun out x ->
  try lprintf out "%s;\n" (ExtXml.attrib x "pre_call") with _ -> ()

(* post call utility function *)
let fp_post_call = fun out x ->
  try lprintf out "%s;\n" (ExtXml.attrib x "post_call") with _ -> ()


(* test until condition test if any, post_call before leaving *)
let stage_until = fun out x ->
  try
    let cond = parsed_attrib x "until" in
    lprintf out "if (%s) {\n" cond;
    right ();
    fp_post_call out x;
    lprintf out "NextStageAndBreak()\n";
    left ();
    lprintf out "}\n"
  with ExtXml.Error _ -> () (* fallback when "until" attribute doesn't exist *)

let rec print_stage = fun out index_of_waypoints x ->
  let stage out = incr stage; lprintf out "Stage(%d)\n" !stage; right () in
  begin
    match String.lowercase_ascii (Xml.tag x) with
      | "return" ->
        stage out;
        lprintf out "Return(%s);\n" (ExtXml.attrib_or_default x "reset_stage" "0");
        lprintf out "break;\n"
      | "goto" ->
        stage out;
        lprintf out "Goto(%s)\n" (name_of x)
      | "deroute" ->
        stage out;
        lprintf out "GotoBlock(%d);\n" (get_index_block (ExtXml.attrib x "block"));
        lprintf out "break;\n"
      | "exit_block" ->
        lprintf out "/* Falls through. */\n";
        lprintf out "default:\n";
        stage out;
        lprintf out "NextBlock();\n";
        lprintf out "break;\n"
      | "while" ->
        let w = gen_label "while" in
        let e = gen_label "endwhile" in
        output_label out w;
        stage out;
        let c = try parsed_attrib x "cond" with _ -> "TRUE" in
        lprintf out "if (! (%s)) Goto(%s) else NextStageAndBreak();\n" c e;
        List.iter (print_stage out index_of_waypoints) (Xml.children x);
        print_stage out index_of_waypoints (goto w);
        output_label out e
      | "for" ->
        let f = gen_label "for" in
        let e = gen_label "endfor" in
        let v = Expr_syntax.c_var_of_ident (ExtXml.attrib x "var")
        and from_ = parsed_attrib x "from"
        and to_expr = parsed_attrib x "to"  in
        let to_var = v ^ "_to" in
        lprintf out "static int8_t %s;\n" v;
        lprintf out "static int8_t %s;\n" to_var;

        (* init *)
        stage out;
        lprintf out "%s = %s - 1;\n" v from_;
        lprintf out "%s = %s;\n" to_var to_expr;
        lprintf out "INTENTIONAL_FALLTHRU\n";
        left ();

        output_label out f;
        stage out;
        lprintf out "if (++%s > %s) Goto(%s) else NextStageAndBreak();\n" v to_var e;
        List.iter (print_stage out index_of_waypoints) (Xml.children x);
        print_stage out index_of_waypoints (goto f);
        output_label out e
      | "heading" ->
        stage out;
        fp_pre_call out x;
        let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
        let p = try ", " ^ (Xml.attrib x "nav_params") with _ -> "" in
        lprintf out "%sHeading(RadOfDeg(%s)%s);\n" t (parsed_attrib x "course") p;
        ignore (output_vmode out x "" "");
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | "follow" ->
        stage out;
        fp_pre_call out x;
        let id = ExtXml.attrib x "ac_id"
        and d = ExtXml.attrib x "distance"
        and h = ExtXml.attrib x "height" in
        let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
        let p = try ", " ^ (Xml.attrib x "nav_params") with _ -> "" in
        lprintf out "%sFollow(%s, %s, %s%s);\n" t id d h p;
        fp_post_call out x;
        lprintf out "break;\n"
      | "attitude" ->
        stage out;
        fp_pre_call out x;
        let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
        let p = try ", " ^ (Xml.attrib x "nav_params") with _ -> "" in
        lprintf out "%sAttitude(RadOfDeg(%s)%s);\n" t (parsed_attrib x "roll") p;
        ignore (output_vmode out x "" "");
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | "manual" ->
        stage out;
        fp_pre_call out x;
        let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
        let p = try ", " ^ (Xml.attrib x "nav_params") with _ -> "" in
        lprintf out "%sSetManual(%s, %s, %s%s);\n" t (parsed_attrib x "roll") (parsed_attrib x "pitch") (parsed_attrib x "yaw") p;
        ignore (output_vmode out x "" "");
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | "go" ->
        stage out;
        fp_pre_call out x;
        let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
        let wp =
          try
            get_index_waypoint (ExtXml.attrib x "wp") index_of_waypoints
          with
              ExtXml.Error _ ->
                lprintf out "waypoints[0].x = %s;\n" (parsed_attrib x "x");
                lprintf out "waypoints[0].y = %s;\n" (parsed_attrib x "y");
                "0"
        in
        let at = try Some (ExtXml.attrib x "approaching_time") with _ -> None in
        let et = try Some (ExtXml.attrib x "exceeding_time") with _ -> None in
        let at = match at, et with
          | Some a, None -> a
          | None, Some e -> "-"^e
          | None, None -> "CARROT"
          | _, _ -> failwith "Error: 'approaching_time' and 'exceeding_time' attributes are not compatible"
        in
        let last_wp =
          try
            get_index_waypoint (ExtXml.attrib x "from") index_of_waypoints
          with ExtXml.Error _ -> "last_wp" in
        if last_wp = "last_wp" then
          lprintf out "if (%sApproaching(%s,%s)) {\n" t wp at
        else
          lprintf out "if (%sApproachingFrom(%s,%s,%s)) {\n" t wp last_wp at;
        right ();
        fp_post_call out x;
        lprintf out "NextStageAndBreakFrom(%s);\n" wp;
        left ();
        lprintf out "} else {\n";
        right ();
        let hmode = output_hmode out x wp last_wp in
        let vmode = output_vmode out x wp last_wp in
        if vmode = "glide" && hmode <> "route" then
          failwith "glide vmode requires route hmode";
        left (); lprintf out "}\n";
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | "stay" ->
        stage out;
        fp_pre_call out x;
        let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
        let p = try ", " ^ (Xml.attrib x "nav_params") with _ -> "" in
        begin
          try
            let wp = get_index_waypoint (ExtXml.attrib x "wp") index_of_waypoints in
            ignore (output_hmode out x wp "");
            ignore (output_vmode out x wp "");
          with
              Xml2h.Error _ ->
                lprintf out "%sGotoXY(last_x, last_y%s);\n" t p;
                ignore(output_vmode out x "" "")
        end;
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | "xyz" ->
        stage out;
        fp_pre_call out x;
        let r = try parsed_attrib x "radius" with _ -> "100" in
        lprintf out "Goto3D(%s)\n" r;
        let x = ExtXml.subst_attrib "vmode" "xyz" x in
        ignore (output_vmode out x "" ""); (** To handle "pitch" *)
        fp_post_call out x;
        lprintf out "break;\n"
      | "home" ->
        stage out;
        lprintf out "nav_home();\n";
        lprintf out "break;\n"
      | "circle" ->
        stage out;
        fp_pre_call out x;
        let wp = get_index_waypoint (ExtXml.attrib x "wp") index_of_waypoints in
        let r = parsed_attrib  x "radius" in
        let _vmode = output_vmode out x wp "" in
        let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
        let p = try ", " ^ (Xml.attrib x "nav_params") with _ -> "" in
        lprintf out "%sCircleWaypoint(%s, %s%s);\n" t wp r p;
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | "eight" ->
        stage out;
        lprintf out "nav_eight_init();\n";
        lprintf out "NextStageAndBreak();\n";
        left ();
        stage out;
        fp_pre_call out x;
        let center = get_index_waypoint (ExtXml.attrib x "center") index_of_waypoints
        and turn_about = get_index_waypoint (ExtXml.attrib x "turn_around") index_of_waypoints in
        let r = parsed_attrib x "radius" in
        let _vmode = output_vmode out x center "" in
        lprintf out "Eight(%s, %s, %s);\n" center turn_about r;
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | "oval" ->
        stage out;
        lprintf out "nav_oval_init();\n";
        lprintf out "NextStageAndBreak();\n";
        left ();
        stage out;
        fp_pre_call out x;
        let p1 = get_index_waypoint (ExtXml.attrib x "p1") index_of_waypoints
        and p2 = get_index_waypoint (ExtXml.attrib x "p2") index_of_waypoints in
        let r = parsed_attrib  x "radius" in
        let _vmode = output_vmode out x p1 "" in
        lprintf out "Oval(%s, %s, %s);\n" p1 p2 r;
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | "set" ->
        stage out;
        let var = ExtXml.attrib  x "var"
        and value = parsed_attrib  x "value" in
        lprintf out "%s = %s;\n" var value;
        lprintf out "NextStage();\n"
      | "call" ->
        stage out;
        let statement = ExtXml.attrib  x "fun" in
        (* by default, function is called while returning TRUE *)
        (* otherwise, function is called once and returned value is ignored *)
        let loop = String.uppercase_ascii (ExtXml.attrib_or_default x "loop" "TRUE") in
        (* be default, go to next stage immediately *)
        let break = String.uppercase_ascii (ExtXml.attrib_or_default x "break" "FALSE") in
        begin match loop with
        | "TRUE" ->
            lprintf out "if (! (%s)) {\n" statement;
            begin match break with
            | "TRUE" -> lprintf out "  NextStageAndBreak();\n";
            | "FALSE" -> lprintf out "  NextStage();\n";
            | _ -> failwith "FP: 'call' break attribute must be TRUE or FALSE";
            end;
            lprintf out "} else {\n";
            begin
              try
                let c = parsed_attrib x "until" in
                lprintf out "  if (%s) NextStageAndBreak();\n" c
              with
                  ExtXml.Error _ -> ()
            end;
            lprintf out "  break;\n";
            lprintf out "}\n"
        | "FALSE" ->
            lprintf out "%s;\n" statement;
            begin match break with
            | "TRUE" -> lprintf out "NextStageAndBreak();\n";
            | "FALSE" -> lprintf out "NextStage();\n";
            | _ -> failwith "FP: 'call' break attribute must be TRUE or FALSE";
            end;
        | _ -> failwith "FP: 'call' loop attribute must be TRUE or FALSE"
        end
      | "call_once" ->
        (* call_once is an alias for <call fun="x" loop="false"/> *)
        stage out;
        let statement = ExtXml.attrib  x "fun" in
        (* by default, go to next stage immediately *)
        let break = String.uppercase_ascii (ExtXml.attrib_or_default x "break" "FALSE") in
        lprintf out "%s;\n" statement;
        begin match break with
        | "TRUE" -> lprintf out "NextStageAndBreak();\n";
        | "FALSE" -> lprintf out "NextStage();\n";
        | _ -> failwith "FP: 'call_once' break attribute must be TRUE or FALSE";
        end;
      | "survey_rectangle" ->
        let grid = parsed_attrib x "grid"
        and wp1 = get_index_waypoint (ExtXml.attrib x "wp1") index_of_waypoints
        and wp2 = get_index_waypoint (ExtXml.attrib x "wp2") index_of_waypoints
        and orientation = ExtXml.attrib_or_default x "orientation" "NS" in
        let t = ExtXml.attrib_or_default x "nav_type" "Nav" in
        let p = try ", " ^ (Xml.attrib x "nav_params") with _ -> "" in
        stage out;
        if orientation <> "NS" && orientation <> "WE" then
          failwith (sprintf "Unknown survey orientation (NS or WE): %s" orientation);
        lprintf out "%sSurveyRectangleInit(%s, %s, %s, %s%s);\n" t wp1 wp2 grid orientation p;
        lprintf out "NextStageAndBreak();\n";
        left ();
        stage out;
        fp_pre_call out x;
        lprintf out "%sSurveyRectangle(%s, %s);\n" t wp1 wp2;
        stage_until out x;
        fp_post_call out x;
        lprintf out "break;\n"
      | _s -> failwith "Unreachable"
  end;
  left ()


let indexed_stages = fun blocks ->
  let lstages = ref [] in
  List.iter
    (fun b ->
      let block_name = name_of b
      and block_no = ExtXml.attrib b "no" in
      let rec f = fun stage ->
        try
          let stage_no = Xml.attrib stage "no" in
          lstages :=
            Xml.Element ("stage", [ "block", block_no;
                                    "block_name", block_name;
                                    "stage", stage_no], [stage]):: !lstages;
          if (ExtXml.tag_is stage "for" || ExtXml.tag_is stage "while") then
            List.iter f (Xml.children stage)
        with Xml.No_attribute "no" ->
          assert (ExtXml.tag_is stage "exception")
      in
      List.iter f (Xml.children b))
    blocks;
  !lstages


let index_blocks = fun xml ->
  let block = ref (-1) in
  let indexed_blocks =
    List.map
      (fun b ->
        incr block;
        let name = name_of b in
        if List.mem_assoc name !index_of_blocks then
          failwith (Printf.sprintf "Error in flight plan: Block '%s' defined twice" name);
        index_of_blocks := (name, !block) :: !index_of_blocks;
        stage := -1;
        let indexed_stages = List.map index_stage (Xml.children b) in
        Xml.Element (Xml.tag b, Xml.attribs b@["no", soi !block], indexed_stages))
      (Xml.children xml) in
  Xml.Element (Xml.tag xml, Xml.attribs xml, indexed_blocks)



let print_block = fun out index_of_waypoints (b:Xml.xml) block_num ->
  let n = name_of b in
  (* Block entry *)
  lprintf out "Block(%d) // %s\n" block_num n;
  fp_pre_call out b;

  let excpts, stages =
    List.partition (fun x -> Xml.tag x = "exception") (Xml.children b) in

  List.iter (print_exception out) excpts;

  lprintf out "switch(nav_stage) {\n";
  right ();
  stage := (-1);
  List.iter (print_stage out index_of_waypoints) stages;

  print_stage out index_of_waypoints exit_block;

  left ();
  lprintf out "}\n";

  (* Block exit *)
  fp_post_call out b;
  lprintf out "break;\n\n"



let print_blocks = fun out index_of_waypoints bs ->
  let block = ref (-1) in
  List.iter (fun b -> incr block; print_block out index_of_waypoints b !block) bs

let c_suffix =
  let r = Str.regexp "^[a-zA-Z0-9_]*$" in
  fun s -> Str.string_match r s 0

let define_waypoints_indices = fun out wpts ->
  let i = ref 0 in
  List.iter (fun w ->
    let n = name_of w in
    if c_suffix n then
      Xml2h.define_out out (sprintf "WP_%s" n) (string_of_int !i);
    incr i)
    wpts

let home = fun waypoints ->
  let rec loop i = function
  [] -> failwith "Waypoint 'HOME' required"
    | w::ws ->
      if name_of w = "HOME" then
        (float_attrib w "x", float_attrib w "y")
      else
        loop (i+1) ws in
  loop 0 waypoints


let check_distance = fun (hx, hy) max_d wp ->
  let x = float_attrib wp "x"
  and y = float_attrib wp "y" in
  let d = sqrt ((x-.hx)**2. +. (y-.hy)**2.) in
  if d > max_d then
    fprintf stderr "\nWarning: Waypoint '%s' too far from HOME (%.0f>%.0f)\n\n" (name_of wp) d max_d


(* Check coherence between global ref and waypoints ref *)
(* Returns a patched xml with utm_x0 and utm_y0 set *)
let check_geo_ref = fun wgs84 xml ->
  let get_float = fun x -> float_attrib xml x in
  let utm0 = utm_of WGS84 wgs84 in

  let max_d = min 1000. (get_float "max_dist_from_home") in
  let check_zone = fun u ->
    if (utm_of WGS84 (of_utm WGS84 u)).utm_zone <> utm0.utm_zone then
      prerr_endline "Warning: You are close (less than twice the max distance) to an UTM zone border ! The navigation will not work unless the GPS receiver configured to send the POSLLH message." in
  check_zone { utm0 with utm_x = utm0.utm_x +. 2.*.max_d };
  check_zone { utm0 with utm_x = utm0.utm_x -. 2.*.max_d };

  let wpts = ExtXml.child xml "waypoints" in
  let wpts = ExtXml.subst_attrib "utm_x0" (sof utm0.utm_x) wpts in
  let wpts = ExtXml.subst_attrib "utm_y0" (sof utm0.utm_y) wpts in
  let x = ExtXml.subst_child "waypoints" wpts xml in
  x

let dummy_waypoint =
  Xml.Element ("waypoint",
               ["name", "dummy";
                "x", "0.";
                "y", "0." ],
               [])


let print_inside_polygon_global = fun out pts name ->
  lprintf out "uint8_t i, j;\n";
  lprintf out "bool c = false;\n";
  (* build array of wp id *)
  lprintf out "const uint8_t nb_pts = %s_NB;\n" name;
  lprintf out "const uint8_t wps_id[] = %s;\n\n" name;
  (* start algo *)
  lprintf out "for (i = 0, j = nb_pts - 1; i < nb_pts; j = i++) {\n";
  right ();
  lprintf out "if (((WaypointY(wps_id[i]) > _y) != (WaypointY(wps_id[j]) > _y)) &&\n";
  lprintf out "   (_x < (WaypointX(wps_id[j])-WaypointX(wps_id[i])) * (_y-WaypointY(wps_id[i])) / (WaypointY(wps_id[j])-WaypointY(wps_id[i])) + WaypointX(wps_id[i]))) {\n";
  right ();
  lprintf out "if (c == TRUE) { c = FALSE; } else { c = TRUE; }\n";
  left();
  lprintf out "}\n";
  left();
  lprintf out "}\n";
  lprintf out "return c;\n"


let print_inside_sector = fun out (s, pts) ->
  let (ids, _) = List.split pts in
  let name = "SECTOR_"^(String.uppercase_ascii s) in
  Xml2h.define_out out (name^"_NB") (string_of_int (List.length pts));
  Xml2h.define_out out name ("{ "^(String.concat ", " ids)^" }");
  lprintf out "static inline bool %s(float _x, float _y) {\n" (inside_function s);
  right ();
  print_inside_polygon_global out pts name;
  left ();
  lprintf out "}\n\n"


let parse_wpt_sector = fun indexes waypoints xml ->
  let sector_name = ExtXml.attrib xml "name" in
  let p2D_of = fun x ->
    let name = name_of x in
    try
      let wp = List.find (fun wp -> name_of wp = name) waypoints in
      let i = get_index_waypoint name indexes in
      let x = float_attrib wp "x"
      and y = float_attrib wp "y" in
      (i, {G2D.x2D = x; G2D.y2D = y })
    with
        Not_found -> failwith (sprintf "Error: corner '%s' of sector '%s' not found" name sector_name)
  in
  (sector_name, List.map p2D_of (Xml.children xml))


(** FP variables and ABI auto bindings *)
type fp_var = FP_var of (string * string * string) | FP_binding of (string * string list option * string * string option)

(* get a Hashtbl of ABI messages (name, field_types) *)
let extract_abi_msg = fun filename class_ ->
    let xml = ExtXml.parse_file filename in
    try
      let xml_class = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = class_) xml "msg_class" in
      let t = Hashtbl.create (List.length (Xml.children xml_class)) in
      List.iter (fun x ->
        let name = ExtXml.attrib x "name"
        and field_types = List.map (fun field -> ExtXml.attrib field "type") (Xml.children x) in
        Hashtbl.add t name field_types
      ) (Xml.children xml_class);
      t
    with
        Not_found -> failwith (sprintf "No msg_class '%s' found" class_)

let parse_variables = fun xml ->
  let some_attrib_or_none = fun v n cb ->
    try Some (cb (ExtXml.attrib v n)) with _ -> None
  in
  List.map (fun var ->
    match Xml.tag var with
    | "variable" ->
        let v = ExtXml.attrib var "var"
        and t = ExtXml.attrib_or_default var "type" "float"
        and i = ExtXml.attrib_or_default var "init" "0" in
        FP_var (v, t, i)
    | "abi_binding" ->
        let n = ExtXml.attrib var "name"
        and vs = some_attrib_or_none var "vars" (fun x -> Str.split (Str.regexp "[ ]*,[ ]*") x)
        and i = ExtXml.attrib_or_default var "id" "ABI_BROADCAST"
        and h = some_attrib_or_none var "handler" (fun x -> x) in
        begin match vs, h with
        | Some _, Some _ | None, None -> failwith "Gen_flight_plan: either 'vars' or 'handler' should be defined, not both"
        | _, _ -> FP_binding (n, vs, i, h)
        end
    | _ -> failwith "Gen_flight_plan: unexpected variables tag"
  ) xml

let print_var_decl out abi_msgs = function
  | FP_var (v, t, _) -> lprintf out "extern %s %s;\n" t v
  | _ -> () (* ABI variables are not public *)

let print_var_impl out abi_msgs = function
  | FP_var (v, t, i) -> lprintf out "%s %s = %s;\n" t v i
  | FP_binding (n, Some vs, _, None) ->
      lprintf out "static abi_event FP_%s_ev;\n" n;
      let field_types = Hashtbl.find abi_msgs n in
      List.iter2 (fun abi_t user -> if not (user = "_") then lprintf out "static %s %s;\n" abi_t user) field_types vs
  | FP_binding (n, None, _, Some _) ->
      lprintf out "static abi_event FP_%s_ev;\n" n
  | _ -> ()

let print_auto_init_bindings = fun out abi_msgs variables ->
  let print_cb = function
    | FP_binding (n, Some vs, _, None) ->
        let field_types = Hashtbl.find abi_msgs n in
        lprintf out "static void FP_%s_cb(uint8_t sender_id __attribute__((unused))" n;
        List.iteri (fun i v ->
          if v = "_" then lprintf out ", %s _unused_%d __attribute__((unused))" (List.nth field_types i) i
          else lprintf out ", %s _%s" (List.nth field_types i) v
        ) vs;
        lprintf out ") {\n";
        List.iter (fun v ->
          if not (v = "_") then lprintf out "  %s = _%s;\n" v v
        ) vs;
        lprintf out "}\n\n"
    | _ -> ()
  in
  let print_bindings = function
    | FP_binding (n, _, i, None) ->
        lprintf out "  AbiBindMsg%s(%s, &FP_%s_ev, FP_%s_cb);\n" n i n n
    | FP_binding (n, _, i, Some h) ->
        lprintf out "  AbiBindMsg%s(%s, &FP_%s_ev, %s);\n" n i n h
    | _ -> ()
  in
  List.iter print_cb variables;
  lprintf out "static inline void auto_nav_init(void) {\n";
  List.iter print_bindings variables;
  lprintf out "}\n\n"

(**
 * Print flight plan header
 *)
let print_flight_plan_h = fun xml ref0 xml_file out_file ->
  let out = open_out out_file in

  let waypoints = Xml.children (ExtXml.child xml "waypoints")
  and variables = try Xml.children (ExtXml.child xml "variables") with _ -> []
  and blocks = Xml.children (ExtXml.child xml "blocks")
  and global_exceptions = try Xml.children (ExtXml.child xml "exceptions") with _ -> [] in

  let h_name = "FLIGHT_PLAN_H" in
  fprintf out "/* This file has been generated by gen_flight_plan from %s */\n" xml_file;
  fprintf out "/* Version %s */\n" (Env.get_paparazzi_version ());
  fprintf out "/* Please DO NOT EDIT */\n\n";

  fprintf out "#ifndef %s\n" h_name;
  Xml2h.define_out out h_name "";
  fprintf out "\n";

  (* include general headers *)
  fprintf out "#include \"std.h\"\n";
  fprintf out "#include \"generated/modules.h\"\n";
  fprintf out "#include \"modules/core/abi.h\"\n";
  fprintf out "#include \"autopilot.h\"\n\n";
  (* print variables and ABI bindings declaration *)

  let variables = parse_variables variables in
  let abi_msgs = extract_abi_msg (Env.paparazzi_home ^ "/conf/abi.xml") "airborne" in
  List.iter (fun v -> print_var_decl out abi_msgs v) variables;
  fprintf out "\n";

  (* add custum header part *)
  begin
    try
      let header = ExtXml.child (ExtXml.child xml "header") "0" in
      fprintf out "%s\n\n" (Xml.pcdata header)
  with _ -> ()
  end;

  let name = ExtXml.attrib xml "name" in
  Xml2h.define_string_out out "FLIGHT_PLAN_NAME" name;

  (* flight plan header *)
  let get_float = fun x -> float_attrib xml x in
  let qfu = try get_float "qfu" with Xml.No_attribute "qfu" -> 0.
  and mdfh = get_float "max_dist_from_home"
  and alt = ExtXml.attrib xml "alt" in
  security_height := get_float "security_height";

  (* check altitudes *)
  begin
    try
      if security_height < ref 0. then
        begin
          fprintf stderr "\nError: Security height cannot be negative (%.0f)\n" !security_height;
          exit 1;
        end
    with _ -> ()
  end;
  let home_mode_height = try
    max (get_float "home_mode_height") !security_height
    with _ -> !security_height in
  check_altitude (float_of_string alt) xml;
  check_altitude_srtm (float_of_string alt) xml !fp_wgs84;

  (* print general defines *)
  Xml2h.define_out out "NAV_DEFAULT_ALT" (sprintf "%.0f /* nominal altitude of the flight plan */" (float_of_string alt));
  Xml2h.define_out out "NAV_UTM_EAST0" (sprintf "%.0f" ref0.utm0.utm_x);
  Xml2h.define_out out "NAV_UTM_NORTH0" (sprintf "%.0f" ref0.utm0.utm_y);
  Xml2h.define_out out "NAV_UTM_ZONE0" (sprintf "%d" ref0.utm0.utm_zone);
  Xml2h.define_out out "NAV_LAT0" (sprintf "%Ld /* 1e7deg */" (convert_angle !fp_wgs84.posn_lat));
  Xml2h.define_out out "NAV_LON0" (sprintf "%Ld /* 1e7deg */" (convert_angle !fp_wgs84.posn_long));
  Xml2h.define_out out "NAV_ALT0" (sprintf "%.0f /* mm above msl */" (1000. *. !ground_alt));
  Xml2h.define_out out "NAV_MSL0" (sprintf "%.0f /* mm, EGM96 geoid-height (msl) over ellipsoid */" (1000. *. Egm96.of_wgs84 !fp_wgs84));

  Xml2h.define_out out "QFU" (sprintf "%.1f" qfu);

  let waypoints = dummy_waypoint :: waypoints in
  let (hx, hy) = home waypoints in
  List.iter (check_distance (hx, hy) mdfh) waypoints;
  define_waypoints_indices out waypoints;

  Xml2h.define_out out "WAYPOINTS_UTM" "{ \\";
  List.iter (print_waypoint_utm out ref0 alt) waypoints;
  lprintf out "};\n";
  Xml2h.define_out out "WAYPOINTS_ENU" "{ \\";
  List.iter (print_waypoint_enu out ref0 alt) waypoints;
  lprintf out "};\n";
  Xml2h.define_out out "WAYPOINTS_LLA" "{ \\";
  List.iter (print_waypoint_lla out ref0 alt) waypoints;
  lprintf out "};\n";
  Xml2h.define_out out "WAYPOINTS_LLA_WGS84" "{ \\";
  List.iter (print_waypoint_lla_wgs84 out ref0 alt) waypoints;
  lprintf out "};\n";
  Xml2h.define_out out "WAYPOINTS_GLOBAL" "{ \\";
  List.iter (print_waypoint_global out) waypoints;
  lprintf out "};\n";
  Xml2h.define_out out "NB_WAYPOINT" (string_of_int (List.length waypoints));

  Xml2h.define_out out "FP_BLOCKS" "{ \\";
  List.iter (fun b -> fprintf out " \"%s\" , \\\n" (ExtXml.attrib b "name")) blocks;
  lprintf out "}\n";
  Xml2h.define_out out "NB_BLOCK" (string_of_int (List.length blocks));

  Xml2h.define_out out "GROUND_ALT" (sof !ground_alt);
  Xml2h.define_out out "GROUND_ALT_CM" (sprintf "%.0f" (100.*. !ground_alt));
  Xml2h.define_out out "SECURITY_HEIGHT" (sof !security_height);
  Xml2h.define_out out "SECURITY_ALT" (sof (!security_height +. !ground_alt));
  Xml2h.define_out out "HOME_MODE_HEIGHT" (sof home_mode_height);
  Xml2h.define_out out "MAX_DIST_FROM_HOME" (sof mdfh);

  (* geofencing warnings and errors *)
  begin
    try
      let geofence_max_alt = get_float "geofence_max_alt" in
      if geofence_max_alt < !ground_alt then
        begin
          fprintf stderr "\nError: Geofence max altitude below ground alt (%.0f < %.0f)\n" geofence_max_alt !ground_alt;
          exit 1;
        end
      else if geofence_max_alt < (!ground_alt +. !security_height) then
        begin
          fprintf stderr "\nError: Geofence max altitude below security height (%.0f < (%.0f+%.0f))\n" geofence_max_alt !ground_alt !security_height;
          exit 1;
        end
      else if geofence_max_alt < (!ground_alt +. home_mode_height) then
        begin
          fprintf stderr "\nError: Geofence max altitude below ground alt + home mode height (%.0f < (%.0f+%.0f))\n" geofence_max_alt !ground_alt home_mode_height;
          exit 1;
        end
      else if geofence_max_alt < (float_of_string alt) then
        fprintf stderr "\nWarning: Geofence max altitude below default waypoint alt (%.0f < %.0f)\n" geofence_max_alt (float_of_string alt);
      Xml2h.define_out out "GEOFENCE_MAX_ALTITUDE" (sof geofence_max_alt);
      fprintf stderr "\nWarning: Geofence max altitude set to %.0f\n" geofence_max_alt;
    with
      _ -> ()
  end;

  begin 
    try
      let geofence_max_height = get_float "geofence_max_height" in
      if geofence_max_height < !security_height then
        begin
          fprintf stderr "\nError: Geofence max height below security height (%.0f < %.0f)\n" geofence_max_height !security_height;
          exit 1;
        end
      else if geofence_max_height < home_mode_height then
        begin
          fprintf stderr "\nError: Geofence max height below home mode height (%.0f < %.0f)\n" geofence_max_height home_mode_height;
          exit 1;
        end
      else if (geofence_max_height +. !ground_alt) < (float_of_string alt) then
        fprintf stderr "\nWarning: Geofence max AGL below default waypoint AGL (%.0f < %.0f)\n" (geofence_max_height +. !ground_alt) (float_of_string alt);
      Xml2h.define_out out "GEOFENCE_MAX_HEIGHT" (sof geofence_max_height);
      fprintf stderr "\nWarning: Geofence max AGL set to %.0f\n" geofence_max_height;
    with
      _ -> ()
  end;

  lprintf out "\n";

  (* index of waypoints *)
  let index_of_waypoints =
    let i = ref (-1) in
    List.map (fun w -> incr i; (name_of w, !i)) waypoints in

  (* print sectors *)
  let sectors_element = try ExtXml.child xml "sectors" with Not_found -> Xml.Element ("", [], []) in
  let sectors = List.filter (fun x -> String.lowercase_ascii (Xml.tag x) = "sector") (Xml.children sectors_element) in
  List.iter (fun x -> match ExtXml.attrib_opt x "type" with
      Some _ -> failwith "Error: attribute \"type\" on flight plan tag \"sector\" is deprecated and must be removed. All sectors are now dynamics.\n"
    | _ -> ()
    ) sectors;
  let sectors = List.map (parse_wpt_sector index_of_waypoints waypoints) sectors in
  List.iter (print_inside_sector out) sectors;

  (* geofencing sector *)
  begin
    try
      let geofence_sector = Xml.attrib xml "geofence_sector" in
      lprintf out "\n#define InGeofenceSector(_x, _y) %s(_x, _y)\n" (inside_function geofence_sector)
    with
        _ -> ()
  end;

  (* start "C" part *)
  lprintf out "\n#ifdef NAV_C\n\n";

  (* print variables and ABI initialization *)
  List.iter (fun v -> print_var_impl out abi_msgs v) variables;
  lprintf out "\n";
  print_auto_init_bindings out abi_msgs variables;

  (* print main flight plan state machine *)
  lprintf out "static inline void auto_nav(void) {\n";
  right ();
  List.iter (print_exception out) global_exceptions;
  lprintf out "switch (nav_block) {\n";
  right ();
  print_blocks out index_of_waypoints blocks;
  lprintf out "default: break;\n";
  left ();
  lprintf out "}\n";
  left ();
  lprintf out "}\n";
  lprintf out "#endif // NAV_C\n";

  Xml2h.finish_out out h_name;
  close_out out


(**
 * Dump expanded version of the flight plan
 *)
let dump_fligh_plan = fun xml out_file ->
  let out = open_out out_file in
  let blocks = Xml.children (ExtXml.child xml "blocks") in
  let xml_stages = Xml.Element ("stages", [], indexed_stages blocks) in
  let dump_xml = Xml.Element ("dump", [], [xml; xml_stages]) in
  fprintf out "%s\n" (ExtXml.to_string_fmt dump_xml);
  close_out out

(**
 *
 * MAIN generation function
 *
 **)
(* FIXME: for later, get rid of those references? *)
let reinit = fun () ->
  index_of_blocks := [];
  check_expressions := false;
  margin := 0;
  stage := 0

let generate = fun flight_plan ?(check=false) ?(dump=false) xml_file out_fp ->

  reinit ();
  let xml = flight_plan.Flight_plan.xml in
  fp_wgs84 := georef_of_xml xml;
  let xml = check_geo_ref !fp_wgs84 xml in

  let dir = Filename.dirname xml_file in
  let xml = Fp_proc.process_includes dir xml in
  let xml = Fp_proc.process_paths xml in
  let xml = Fp_proc.process_relative_waypoints xml in

  (* Add a safety last HOME block *)
  let blocks = Xml.children (ExtXml.child xml "blocks") @ [home_block] in
  let xml = ExtXml.subst_child "blocks" (index_blocks (element "blocks" [] blocks)) xml in
  let waypoints = Xml.children (ExtXml.child xml "waypoints") in

  let frame = ExtXml.attrib_or_default xml "wp_frame" "UTM" in
  let frame = match String.uppercase_ascii frame with
    | "UTM" -> UTM
    | "LTP" -> LTP
    | _ -> failwith ("Error: unkown wp_frame \"" ^ frame ^ "\". Use \"utm\" or \"ltp\"")
    in

  ground_alt := float_attrib xml "ground_alt";
  let ref0 = {
    frame;
    utm0 = utm_of WGS84 !fp_wgs84;
    ecef0 = Latlong.ecef_of_geo Latlong.WGS84 !fp_wgs84 !ground_alt;
    geo0 = !fp_wgs84
  } in
  
  let waypoints = match frame with
    | UTM ->
      let rel_utm_of_wgs84 = fun wgs84 ->
        (* force utm zone to be the same that reference point *)
        let utm = utm_of ~zone:ref0.utm0.utm_zone WGS84 wgs84 in
        (utm.utm_x -. ref0.utm0.utm_x, utm.utm_y -. ref0.utm0.utm_y) in
      List.map (localize_waypoint rel_utm_of_wgs84) waypoints
    | LTP -> waypoints
    in
  
  let xml = ExtXml.subst_child "waypoints" (element "waypoints" [] waypoints) xml in

  if dump then dump_fligh_plan xml out_fp
  else print_flight_plan_h xml ref0 xml_file out_fp

