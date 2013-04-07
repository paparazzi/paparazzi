(*
 * Copyright (C) 2009 ENAC, Pascal Brisset
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

(** Creating FIG files ({{:http://www.xfig.org/userman/fig-format.html}format 3.2}) *)

open Printf

let version = "3.2"

type units = int
type funits = float
type point = units * units

type color = int
type filling = int

type line_style =
    Solid | Dashed | Dotted | DashDotted | DashDoubleDotted | DashTripleDotted
let (int_of_line_style:line_style->int) = Obj.magic
let (line_style_of_int:int->line_style) = Obj.magic

type join_style = Miter| Round | Bevel
let (int_of_join_style:join_style->int) = Obj.magic
let join_style_of_int = Obj.magic
type cap_style = Butt | Round | Projecting
let int_of_cap_style = Obj.magic
let cap_style_of_int = Obj.magic


type polyline =
    Polyline | Box | Polygon | ArcBox | PictureBB of int*string
let int_of_polyline = function
Polyline -> 1 | Box -> 2 | Polygon -> 3 | ArcBox -> 4 | PictureBB _ -> 5
let polyline_of_int = fun ?(flipped=0) ?(pict="") i ->
  match i with
      1 -> Polyline
    | 2 -> Box
    | 3 -> Polygon
    | 4 -> ArcBox
    | 5 -> PictureBB (flipped, pict)
    | _ -> invalid_arg "polyline_of_int"

type arc = Open | Closed
let int_of_arc x = Obj.magic x + 1
let arc_of_int x = Obj.magic (x - 1)

type spline = Approximated | Interpolated | XSpline
let int_of_spline = function
(Open, Approximated) -> 0
  | (Closed, Approximated) -> 1
  | (Open, Interpolated) -> 2
  | (Closed, Interpolated) -> 3
  | (Open, XSpline) -> 4
  | (Closed, XSpline) -> 5
let spline_of_int = function
0 -> (Open, Approximated)
  | 1 -> (Closed, Approximated)
  | 2 -> (Open, Interpolated)
  | 3-> (Closed, Interpolated)
  | 4 -> (Open, XSpline)
  | 5-> (Closed, XSpline)
  | _ -> invalid_arg "spline_of_int"

type ellipse = EllipseRadius | EllipseDiameter | CircleRadius | CircleDiameter
let int_of_ellipse x = Obj.magic x + 1
let ellipse_of_int x = (Obj.magic (x-1) : ellipse)


type arrow = {
  arrow_type: int;
  arrow_style:    int;
  arrow_thickness:    float;
  arrow_width:    float;
  arrow_height:   float
}

type depth = int

type graphic_attributes =
    { line_style : line_style;
      line_thickness : int;
      pen_color : color;
      fill_color : color;
      depth : depth;
      area_fill : filling;
      style_val : float;
      cap_style : cap_style;
      forward_arrow : arrow option;
      backward_arrow : arrow option;
    }
type radius = units
type direction = Clockwise | CounterClockwise
let direction_of_int = Obj.magic
let int_of_direction = Obj.magic

type angle = float (* Radian *)
type shape_factor = float
type graphic_object =
    Ellipse of ellipse * direction * angle * point * units * units * point * point
  | Polylines of polyline * join_style * radius * point list
  | Spline of (arc*spline) * point list * shape_factor list
  | Arc of arc * direction * (float * float) * point * point * point

type justification = LeftJustified | CenterJustified | RightJustified
let justification_of_int = Obj.magic
let int_of_justification = Obj.magic
type postscript_font =
  |  TimesRoman
  |  TimesItalic
  |  TimesBold
  |  TimesBoldItalic
  |  AvantGardeBook
  |  AvantGardeBookOblique
  |  AvantGardeDemi
  |  AvantGardeDemiOblique
  |  BookmanLight
  |  BookmanLightItalic
  |  BookmanDemi
  |  BookmanDemiItalic
  |  Courier
  |  CourierOblique
  |  CourierBold
  |  CourierBoldOblique
  |  Helvetica
  |  HelveticaOblique
  |  HelveticaBold
  |  HelveticaBoldOblique
  |  HelveticaNarrow
  |  HelveticaNarrowOblique
  |  HelveticaNarrowBold
  |  HelveticaNarrowBoldOblique
  |  NewCenturySchoolbookRoman
  |  NewCenturySchoolbookItalic
  |  NewCenturySchoolbookBold
  |  NewCenturySchoolbookBoldItalic
  |  PalatinoRoman
  |  PalatinoItalic
  |  PalatinoBold
  |  PalatinoBoldItalic
  |  Symbol
  |  ZapfChanceryMediumItalic
  |  ZapfDingbats

type latex_font =
    DefaultLatex
  | Roman
  | Bold
  | Italic
  | SansSerif
  | Typewriter

type font = Postscript of postscript_font | Latex of latex_font

let int_of_font = function
Postscript f -> Obj.magic f
  | Latex f -> Obj.magic f

let font_of_int ps i =
  if ps then Postscript (Obj.magic i) else Latex (Obj.magic i)

type font_size = int
type font_flags = int

type comments = string list

type fig_object = (comments * raw_object)
and raw_object =
    UserColor of color * (int * int * int)
  | Compound of point * point * fig_object list
  | Text of justification * color * depth * font * font_size * angle * font_flags * funits * funits * point * string
  | GrObj of graphic_attributes * graphic_object

type orientation = Landscape | Portrait
type just = Center | FlushLeft
type units_name = Metric | Inches
type papersize = string
type multiple_page = Single | Multiple

let string_of_orientation = function
Landscape -> "Landscape" | Portrait -> "Portrait"
let string_of_just = function
Center -> "Center" | FlushLeft -> "FlushLeft"
let string_of_units_name = function
Metric -> "Metric" | Inches -> "Inches"
let string_of_multiple_page = function
Single -> "Single" | Multiple -> "Multiple"

let orientation_of_string = function
"Landscape" -> Landscape | "Portrait" -> Portrait | _ -> invalid_arg "orientation_of_string"
let just_of_string = function
"Center" -> Center | "Flush Left" -> FlushLeft | "Flush left" -> FlushLeft | _ -> invalid_arg "just_of_string"
let units_name_of_string = function
"Metric" -> Metric | "Inches" -> Inches | _ -> invalid_arg "units_name_of_string"
let multiple_page_of_string = function
"Single" -> Single | "Multiple" -> Multiple | _ -> invalid_arg "multiple_page_of_string"


type t = {
  version : string;
  orientation : orientation;
  justification : just;
  units : units_name;
  papersize : papersize;
  magnification : float;
  multiple_page : multiple_page;
  transparent_color : int;
  comments : string list;
  resolution : units * int;
  body : fig_object list
}

let black = 0
let blue = 1
let green = 2
let cyan = 3
let red = 4
let magenta = 5
let yellow = 6
let white = 7

let user_color = ref 31
let color = fun ?number r g b ->
  let short = fun x ->
    if x < 0 || x > 255 then
      invalid_arg "Fig.color: color composite out of bound" in
  short r;
  short g;
  short b;
  let n =
    match number with
        None -> incr user_color; !user_color
      | Some x -> x in
  if n < 32 || n > 543 then invalid_arg "Fig.color: color number out of bound";
  (n, (["User color"], UserColor (n, (r, g, b))))

let default_arrow = {
  arrow_type = 1;
  arrow_style = 1;
  arrow_thickness= 1.0;
  arrow_width = 60.0;
  arrow_height = 120.0
}

let filled = 20

let close = fun (f, _) -> close_out f;;

let one = fun x -> if x = None then 0 else 1

let arrow = fun f x ->
  match x with
      None -> ()
    | Some t -> fprintf f "\t%d %d %.2f %.2f %.2f\n" t.arrow_type t.arrow_style t.arrow_thickness t.arrow_width t.arrow_height

let point = fun f (x,y) -> fprintf f " %d %d" x y

let comment = fun lines (c, o) -> (c @ lines, o)

(* Environment *)
let create = fun
  ?(comments = ["Generated by fig.ml"])
  ?(orientation = Landscape)
  ?(justification = Center)
  ?(units = Metric)
  ?(papersize = "A4")
  ?(magnification = 100.0)
  ?(multiple_page = Single)
  ?(transparent_color = -2)
  ?(resolution=(1200,2))
  objects ->
    { version = "3.2";
      orientation = orientation;
      justification = justification;
      units = units;
      papersize = papersize;
      magnification = magnification;
      multiple_page = multiple_page;
      transparent_color = transparent_color;
      comments = comments;
      resolution = resolution;
      body = objects }


(* Polyline *)
let polyline = fun
  ?(sub_type = Polyline)
  ?(line_style = Solid)
  ?(thickness = 1)
  ?(pen_color = black)
  ?(fill_color = white)
  ?(depth = 50)
  ?(area_fill = -1)
  ?(style_val = 4.0)
  ?(join_style = Miter)
  ?(cap_style = Butt)
  ?(radius = 7)
  ?forward_arrow
  ?backward_arrow
  list ->
    let list = (* Checking and fixing nb of points *)
      match sub_type, list with
          Polyline, _::_ -> list
        | Polygon, first::rest ->
          if List.hd (List.rev list) <> first then begin
            prerr_endline "Fig.polyline: closing Polygon";
            list @ [first]
          end else
            list
        | _box, [(x0,y0);(x1, y1)] -> (* Opposed corners *)
          [(x0,y0);(x1, y0);(x1,y1);(x0,y1);(x0,y0)]
        | _ -> invalid_arg "Fig.polyline"
    in
    let attributes = {
      line_style = line_style;
      line_thickness = thickness;
      pen_color = pen_color;
      fill_color = fill_color;
      depth = depth;
      area_fill = area_fill;
      style_val = style_val;
      cap_style = cap_style;
      forward_arrow = forward_arrow;
      backward_arrow = backward_arrow
    } in

    (["Polyline"], GrObj (attributes, Polylines (sub_type, join_style, (if sub_type = ArcBox then radius else -1), list)))


(* Arc *)
let arc = fun
  ?(sub_type = Open)
  ?(line_style = Solid)
  ?(thickness = 1)
  ?(pen_color = black)
  ?(fill_color = white)
  ?(depth = 50)
  ?(area_fill = -1)
  ?(style_val = 0.0)
  ?(cap_style = Butt)
  ?forward_arrow
  ?backward_arrow
  (center_x, center_y) radius alpha1 alpha2 ->
    let attributes = {
      line_style = line_style;
      line_thickness = thickness;
      pen_color = pen_color;
      fill_color = fill_color;
      depth = depth;
      area_fill = area_fill;
      style_val = style_val;
      cap_style = cap_style;
      forward_arrow = forward_arrow;
      backward_arrow = backward_arrow
    } in
    let direction = if alpha2 >0. then Clockwise else CounterClockwise in
    let p1 = (int_of_float (center_x +. (radius *. cos alpha1)),
              int_of_float (center_y +. (radius *. sin alpha1)))
    and p2 = (int_of_float (center_x +. (radius *. cos (alpha1+.alpha2/.2.))),
              int_of_float (center_y +. (radius *. sin (alpha1+.alpha2/.2.))))
    and p3 = (int_of_float (center_x +. (radius *. cos (alpha1+.alpha2))),
              int_of_float (center_y +. (radius *. sin (alpha1+.alpha2)))) in
    (["Arc"], GrObj (attributes, Arc (sub_type, direction, (center_x, center_y), p1, p2, p3)))

(* Ellipse *)
let ellipse = fun
  ?(line_style = Solid)
  ?(thickness = 1)
  ?(pen_color = black)
  ?(fill_color = white)
  ?(depth = 50)
  ?(area_fill = -1)
  ?(style_val = 0.0)
  ?(direction = Clockwise)
  ?(angle = 0.0)
  (center_x, center_y) radius_x radius_y ->
    let attributes = {
      line_style = line_style;
      line_thickness = thickness;
      pen_color = pen_color;
      fill_color = fill_color;
      depth = depth;
      area_fill = area_fill;
      style_val = style_val;
      cap_style = Butt; (* Unused *)
      forward_arrow = None;
      backward_arrow = None
    } in
    (["Ellipse"], GrObj (attributes, Ellipse (EllipseRadius, direction, angle, (center_x, center_y), radius_x, radius_y, (center_x, center_y), (center_x + radius_x, center_y + radius_y))))

let factors = fun points spline ->
  let _f = match spline with (_, Interpolated) -> -1.0 | _ -> 1.0 in
  let rec loop = function
  []  -> []
    | [_] -> [0.]
    | _ :: xs -> -1. :: loop xs in
  match points with
      [] -> []
    | _ :: xs -> 0. :: loop xs


(* Spline *)
let spline = fun
  ?(sub_type = Open, Approximated)
  ?(line_style = Solid)
  ?(thickness = 1)
  ?(pen_color = black)
  ?(fill_color = white)
  ?(depth = 50)
  ?(area_fill = -1)
  ?(style_val = 0.0)
  ?(cap_style = Butt)
  ?forward_arrow
  ?backward_arrow
  list ->
    let list = (* Checking and fixing nb of points *)
      match fst sub_type, list with
          Open, _::_ -> list
        | Closed, first::rest ->
          if List.hd (List.rev list) <> first then begin
            prerr_endline "Fig.spline: closing spline";
            list @ [first]
          end else
            list
        | _ -> invalid_arg "Fig.spline"
    in
    let attributes = {
      line_style = line_style;
      line_thickness = thickness;
      pen_color = pen_color;
      fill_color = fill_color;
      depth = depth;
      area_fill = area_fill;
      style_val = style_val;
      cap_style = cap_style;
      forward_arrow = forward_arrow;
      backward_arrow = backward_arrow
    } in
    (["Spline"], GrObj (attributes, Spline (sub_type, list, factors list sub_type)))

(* Text *)
let bit x d =
  (if x then 1 else 0) lsl d
let font_flags = fun rigid special font hidden ->
  bit rigid 0 +
    bit special 1 +
    bit (match font with Postscript _ -> true | _ -> false) 2 +
    bit hidden 3

let code_string = fun f s ->
  for i = 0 to String.length s - 1 do
    if Char.code s.[i] > 0o177 then
      fprintf f "\\%3o" (Char.code s.[i])
    else
      fprintf f "%c" s.[i]
  done

let text = fun
  ?(sub_type = LeftJustified)
  ?(color = black)
  ?(depth = 50)
  ?(font = Postscript TimesRoman)
  ?(font_size = 12)
  ?(angle = 0.0)
  ?(rigid = true)
  ?(special = false)
  ?(hidden = false)
  (x,y) string ->
      (* Null height and length automatically updated by xfig *)
    let ff = font_flags rigid special font hidden in
    (["Text"], Text (sub_type, color, depth, font, font_size, angle, ff, 0., 0., (x, y), string))

let compound = fun objects ->
  (* Null box automatically updated by xfig *)
  (["Compound"], Compound ((0,0), (0,0), objects))


open Scanf

let rec read_comments = fun s ->
  bscanf s " %0c" (fun c ->
    if c = '#' then
      bscanf s " %s@\n" (fun l ->
        let n = String.length l in
        String.sub l 2 (n-2) :: read_comments s)
    else
      [])



let read_point = fun s ->
  bscanf s " %d %d" (fun x y -> (x, y))

let read_user_color = fun s ->
  bscanf s " %d" (fun n ->
    bscanf s " #%2x%2x%2x" (fun r g b ->
      UserColor (n, (r, g, b))))

let read_ellipse = fun s ->
  bscanf s " %d %d %d %d %d %d %d %d %f %d %f" (fun st ls thick pc fc depth _ps af sv dirct angle ->
    let c = read_point s in
    let (rx, ry) = read_point s in
    let p1 = read_point s in
    let p2 = read_point s in
    GrObj ({line_style = line_style_of_int ls;
            line_thickness = thick;
            pen_color = pc;
            fill_color = fc;
            depth = depth;
            area_fill = af;
            style_val = sv;
            cap_style = Butt; (* Unused *)
            forward_arrow = None;
            backward_arrow = None}, (Ellipse (ellipse_of_int st, direction_of_int dirct, angle, c, rx, ry, p1, p2))))

let read_arrow = fun s flag ->
  if flag = 0 then None else
    bscanf s " %d %d %f %f %f" (fun at s thick w h ->
      Some { arrow_type = at;
             arrow_style= s;
             arrow_thickness= thick;
             arrow_width = w;
             arrow_height= h
           })

let read_picture = fun s ->
  bscanf s " %d %s" (fun flip name -> PictureBB (flip, name))

let rec read_points s n =
  if n = 0 then [] else
    let p = read_point s in
    p :: read_points s (n-1)

let rec read_floats s n =
  if n = 0 then [] else
    bscanf s " %f" (fun f -> f :: read_floats s (n-1))


let read_polyline = fun s ->
  bscanf s " %d %d %d %d %d %d %d %d %f %d %d %d %d %d %d" (fun st ls thick pc fc depth _ps af sv js cs rad faf baf n ->
    let fa = read_arrow s faf in
    let ba = read_arrow s baf in
    let st = if st = 5 then read_picture s else polyline_of_int st in
    let points = read_points s n in
    let com = {line_style = line_style_of_int ls;
               line_thickness = thick;
               pen_color = pc;
               fill_color = fc;
               depth = depth;
               area_fill = af;
               style_val = sv;
               cap_style = cap_style_of_int cs;
               forward_arrow = fa;
               backward_arrow = ba} in
    GrObj (com, Polylines (st, join_style_of_int js, rad, points)))


let read_spline = fun s ->
  bscanf s " %d %d %d %d %d %d %d %d %f %d %d %d %d" (fun st ls thick pc fc depth _ps af sv cs faf baf n ->
    let fa = read_arrow s faf in
    let ba = read_arrow s baf in
    let points = read_points s n in
    let shape_factors = read_floats s n in
    let com = {line_style = line_style_of_int ls;
               line_thickness = thick;
               pen_color = pc;
               fill_color = fc;
               depth = depth;
               area_fill = af;
               style_val = sv;
               cap_style = cap_style_of_int cs;
               forward_arrow = fa;
               backward_arrow = ba} in
    GrObj (com, Spline (spline_of_int st, points, shape_factors)))

let read_arc = fun s ->
  bscanf s " %d %d %d %d %d %d %d %d %f %d %d %d %d %f %f" (fun st ls thick pc fc depth _ps af sv cs dirct faf baf cx cy ->
    let p1 = read_point s in
    let p2 = read_point s in
    let p3 = read_point s in
    let fa = read_arrow s faf in
    let ba = read_arrow s baf in

    let com = { line_style = line_style_of_int ls;
                line_thickness = thick;
                pen_color = pc;
                fill_color = fc;
                depth = depth;
                area_fill = af;
                style_val = sv;
                cap_style = cap_style_of_int cs;
                forward_arrow = fa;
                backward_arrow = ba} in

    GrObj(com, Arc (arc_of_int st, direction_of_int dirct, (cx, cy), p1, p2, p3)))




let bit2 ff = (ff lor 2) land 1 = 1

let read_text = fun s ->
  bscanf s " %d %d %d %d %d %d %f %d %f %f" (fun st c depth _ps ft fs angle ff h l ->
    let p = read_point s in
    bscanf s "%c%s@\n" (fun _space text ->
      let n = String.length text in
      assert (String.sub text (n-4) 4 = "\\001");
      let text = String.sub text 0 (n - 4) in
      Text (justification_of_int st, c, depth, font_of_int (bit2 ff) ft, fs, angle, ff, h, l, p,text)))


let rec read_objects = fun s ->
  let read_object = [| read_user_color; read_ellipse; read_polyline; read_spline; read_text; read_arc; read_compound |] in
  try
    let comments = read_comments s in
    bscanf s " %d" (fun code ->
      if 0 <= code && code <= 6 then
        let o = read_object.(code) s in
        (comments, o) :: read_objects s
      else if code = -6 then []
      else failwith ("read_objects: "^string_of_int code))
  with
      End_of_file -> []
and read_compound = fun s ->
  bscanf s " %d %d %d %d" (fun x1 y1 x2 y2 -> Compound ((x1,y1), (x2,y2), read_objects s))



let read = fun file ->
  let s = Scanning.from_file file in
  bscanf s "#FIG %s@\n%s %s@\n %s %s %f %s %d" (fun v o j u p m multi t ->

    if String.sub v 0 3 <> "3.2" then
      failwith ("Unknown FIG format version: "^v);

    let comments = read_comments s in

    bscanf s "%d %d" (fun resolution coord_system ->
      let os = read_objects s in

      {
        version = v;
        orientation = orientation_of_string o;
        justification = just_of_string j;
        units = units_name_of_string u;
        papersize = p;
        magnification = m;
        multiple_page = multiple_page_of_string multi;
        transparent_color = t;
        comments = comments;
        resolution = (resolution, coord_system);
        body = os
      }))

let fprint_point f (x, y) = fprintf f " %d %d" x y

let arrow_flag = function
None -> 0
  | Some _ -> 1

let fprint_arrow f = function
None -> ()
  | Some a ->
    fprintf f "\t%d %d %.2f %.2f %.2f\n" a.arrow_type a.arrow_style a.arrow_thickness a.arrow_width a.arrow_height

let fprint_picture f = function
PictureBB (flip, name) ->
  fprintf f "%d %s\n" flip name
  | _ -> ()

let write_graphic_object f com = function
Ellipse (ellipse, direction, angle, center, rx, ry, p1, p2) ->
  fprintf f "1 %d %d %d %d %d %d -1 %d %.3f "
    (int_of_ellipse ellipse)
    (int_of_line_style com.line_style) com.line_thickness com.pen_color com.fill_color
    com.depth com.area_fill com.style_val;
  fprintf f "%d %.4f%a%a%a%a\n"
    (int_of_direction direction) angle fprint_point center fprint_point (rx,ry)
    fprint_point p1 fprint_point p2
  | Polylines (polyline,  join_style, radius, points) ->
    fprintf f "2 %d %d %d %d %d %d 0 %d %.3f "
      (int_of_polyline polyline)
      (int_of_line_style com.line_style) com.line_thickness com.pen_color com.fill_color
      com.depth com.area_fill com.style_val;
    fprintf f "%d %d %d " (int_of_join_style join_style) (int_of_cap_style com.cap_style) radius;
    fprintf f "%d %d %d\n" (arrow_flag com.forward_arrow) (arrow_flag com.backward_arrow) (List.length points);
    fprint_arrow f com.forward_arrow;
    fprint_arrow f com.backward_arrow;
    fprint_picture f polyline;
    fprintf f "\t"; List.iter (fprint_point f) points; fprintf f "\n";
  | Spline ((arc,spline), points, factors) ->
    fprintf f "3 %d %d %d %d %d %d 0 %d %f "
      (int_of_spline (arc,spline))
      (int_of_line_style com.line_style) com.line_thickness com.pen_color com.fill_color
      com.depth com.area_fill com.style_val;
    fprintf f "%d " (int_of_cap_style com.cap_style);
    fprintf f "%d %d %d\n" (arrow_flag com.forward_arrow) (arrow_flag com.backward_arrow) (List.length points);
    fprint_arrow f com.forward_arrow;
    fprint_arrow f com.backward_arrow;
    fprintf f "\t"; List.iter (fprint_point f) points; fprintf f "\n";
    fprintf f "\t"; List.iter (fun x -> fprintf f " %.3f" x) factors; fprintf f "\n"

  | Arc (arc, direction, (cx, cy),  p1, p2, p3) ->
    fprintf f "5 %d %d %d %d %d %d 0 %d %f "
      (int_of_arc arc)
      (int_of_line_style com.line_style) com.line_thickness com.pen_color com.fill_color
      com.depth com.area_fill com.style_val;
    fprintf f "%d %d " (int_of_cap_style com.cap_style) (direction_of_int direction);
    fprintf f "%d %d " (arrow_flag com.forward_arrow) (arrow_flag com.backward_arrow);
    fprintf f "%f %f " cx cy;
    fprintf f "%a %a %a\n" fprint_point p1 fprint_point p2 fprint_point p3;
    fprint_arrow f com.forward_arrow;
    fprint_arrow f com.backward_arrow

let rec write_object f (comments, obj) =
  List.iter (fun x -> fprintf f "# %s\n" x) comments;
  match obj with
      UserColor (color, (r, g, b)) -> fprintf f "0 %d #%02x%02x%02x\n" color r g b
    | Compound (p1, p2, objects) ->
      fprintf f "6%a%a\n" fprint_point p1 fprint_point p2;
      List.iter (write_object f) objects;
      fprintf f "-6\n"
    | Text (justification, color, depth, font, font_size, angle, font_flags, h, l, point, string) ->
      fprintf f "4 %d %d %d 0 %d %d %.4f %d %.0f %.0f%a %a\\001\n"
        (int_of_justification justification) color depth (int_of_font font) font_size angle font_flags h l fprint_point point code_string string
    | GrObj (graphic_common, graphic_object) ->
      write_graphic_object f graphic_common  graphic_object

let write_out f { version = v;
                  orientation = o;
                  justification = j;
                  units = u;
                  papersize = p;
                  magnification = m;
                  multiple_page = multi;
                  transparent_color = t;
                  comments = comments;
                  resolution = (resolution, coord_system);
                  body = os
                } =
  fprintf f "#FIG %s\n%s\n%s\n%s\n%s\n%.2f\n%s\n%d\n" v (string_of_orientation o) (string_of_just j) (string_of_units_name u) p m (string_of_multiple_page multi) t;
  List.iter (fun x -> fprintf f "# %s\n" x) comments;
  fprintf f "%d %d\n" resolution coord_system;

  let (colors, others) = List.partition (function (_, UserColor _) -> true | _ -> false) os in
  List.iter (write_object f) colors;
  List.iter (write_object f) others



let write f fig =
  let f = open_out f in
  write_out f fig;
  close_out f
