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

(** This module provides a basic interface to produce FIG files to be edited
with the popular xfig editor and translated into various image formats with
fig2dev (see {{:http://www.xfig.org}Xfig}).

Usage is straighforward and can be illustrated with a simple example:
{[
let (color_id, color_obj) = Fig.color 100 100 100 in
let p1 = Fig.polyline [0,0; 1000, 1000; 1000, 0] in
let p2 = Fig.polyline ~sub_type:Fig.Box ~pen_color:color_id ~thickness:4 [100,100;200,200] in
let p3 = Fig.ellipse ~fill_color:Fig.yellow ~area_fill:Fig.filled (500,500) 200 200 in
let flower =
  List.map (fun a ->
    let angle = 3.14 *. float a /. 3. in
    Fig.text ~font:(Fig.Postscript Fig.Courier) ~angle ~color:Fig.magenta
             (500,500) "  Petal")
    [0; 1; 2; 3; 4; 5] in
let c = Fig.comment ["flower"] (Fig.compound flower) in
let fig = Fig.create ~orientation:Fig.Portrait [p1; p2; p3; color_obj; c] in
Fig.write "foo.fig" fig;;
]}

The [read] function allows to parse a FIG file.
*)

type color = int

val black : color
val blue : color
val green : color
val cyan : color
val red : color
val magenta : color
val yellow : color
val white : color
(** Basic predefined colors *)

type arrow = {
    arrow_type : int;
    arrow_style : int;
    arrow_thickness : float;
    arrow_width : float;
    arrow_height : float
  }
val default_arrow : arrow
(** [{ arrow_type = 1; arrow_style = 1; arrow_thickness= 1.0;
   arrow_width = 60.0; arrow_height = 120.0 }] *)

type units = int
type funits = float
type point = units * units
(** Point in Fig units (inch/1200) *)

type filling = int
(** Filling description. See {{:http://www.xfig.org/userman/fig-format.html}format} description for details. *)

val filled : filling
(** Full saturation of the fill color *)

type line_style =
    Solid | Dashed | Dotted | DashDotted | DashDoubleDotted | DashTripleDotted
type join_style = Miter| Round | Bevel
type cap_style = Butt | Round | Projecting

type polyline =
    Polyline | Box | Polygon | ArcBox | PictureBB of int * string

type ellipse = EllipseRadius | EllipseDiameter | CircleRadius | CircleDiameter
type arc = Open | Closed
type spline = Approximated | Interpolated | XSpline
type direction = Clockwise | CounterClockwise
type angle = float
type radius = units
type depth = int
type font_size = int (* float in the format doc but int in .fig files ! *)
type font_flags = int
type shape_factor = float

type graphic_object =
    Ellipse of ellipse * direction * angle * point * units * units * point * point
  | Polylines of polyline * join_style * radius * point list
  | Spline of (arc*spline) * point list * shape_factor list
  | Arc of arc * direction * (float * float) * point * point * point

type graphic_attributes = {
    line_style : line_style;
    line_thickness : int;
    pen_color : color;
    fill_color : color;
    depth : depth;
    area_fill : filling;
    style_val : float;
    cap_style : cap_style;
    forward_arrow : arrow option;
    backward_arrow : arrow option
  }

type justification = LeftJustified | CenterJustified | RightJustified

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


val color : ?number:int -> int -> int -> int -> (color * fig_object)
(** [color ?number r g b] creates a new user color. Number is
allocated automatically if not specified. It is returned with the
associated fig object *)

val polyline :
    ?sub_type:polyline ->     (** Polyline *)
    ?line_style:line_style -> (** Solid *)
    ?thickness:int ->         (** 1 *)
    ?pen_color:color ->         (** black *)
    ?fill_color:color ->        (** black *)
    ?depth:int ->             (** 50 *)
    ?area_fill:filling ->         (** -1 *)
    ?style_val:float ->       (** 4.0 *)
    ?join_style:join_style -> (** Miter *)
    ?cap_style:cap_style ->   (** Butt *)
    ?radius:int ->            (** 7 for ArcBox, -1 else *)
    ?forward_arrow:arrow ->   (** None *)
    ?backward_arrow:arrow ->  (** None *)
    point list -> fig_object
(** [polyline ?... point_list] *)
(** For Boxes, only opposite corners are required. Polygons are closed
automatically. *)

val arc :
    ?sub_type:arc ->          (** Open *)
    ?line_style:line_style -> (** Solid *)
    ?thickness:int ->         (** 1 *)
    ?pen_color:color ->       (** black *)
    ?fill_color:color ->      (** white *)
    ?depth:int ->             (** 50 *)
    ?area_fill:filling ->         (** -1 *)
    ?style_val:float ->       (** 0.0 *)
    ?cap_style:cap_style ->   (** Butt *)
    ?forward_arrow:arrow ->   (** None *)
    ?backward_arrow:arrow ->  (** None *)
    float * float -> float -> float -> float -> fig_object
(** [arc ?... centre radius alpha1 alpha2]
alpha1 is the angle between the X axis and the line centre->First point
alpha2 is the angle between the line centre->First point and centre->Last point
alpha1 and alpha2 are in radian
*)

val ellipse :
    ?line_style:line_style -> (** Solid *)
    ?thickness:int ->         (** 1 *)
    ?pen_color:color ->       (** black *)
    ?fill_color:color ->      (** white *)
    ?depth:int ->             (** 50 *)
    ?area_fill:filling ->         (** -1 *)
    ?style_val:float ->       (** 0.0 *)
    ?direction:direction ->   (** Clockwise *)
    ?angle:float ->           (** 0.0 *)
    point -> int -> int -> fig_object
(** [ellipse ?... centre radius_x radius_y] Only "ellipse defined by radius"
subtype *)

val spline :
    ?sub_type:arc * spline -> (** Open, Approximated *)
    ?line_style:line_style -> (** Solid *)
    ?thickness:int ->         (** 1 *)
    ?pen_color:color ->       (** black *)
    ?fill_color:color ->      (** white *)
    ?depth:int ->             (** 50 *)
    ?area_fill:filling ->     (** -1 *)
    ?style_val:float ->       (** 0.0 *)
    ?cap_style:cap_style ->   (** Butt *)
    ?forward_arrow:arrow ->   (** None *)
    ?backward_arrow:arrow ->  (** None *)
    point list -> fig_object
(** [ellipse ?... point_list] *)


val text :
    ?sub_type:justification -> (** LeftJustified *)
    ?color:color ->            (** black *)
    ?depth:int ->              (** 50 *)
    ?font:font ->              (** Postscript TimesRoman *)
    ?font_size:font_size ->    (** 12 *)
    ?angle:float ->            (** 0.0 *)
    ?rigid:bool ->             (** true *)
    ?special:bool ->           (** false *)
    ?hidden:bool ->            (** false *)
    point -> string -> fig_object
(** [text ?... position string] *)

val compound : fig_object list -> fig_object
(** [compound objects] creates a compound *)

val comment : comments -> fig_object -> fig_object
(** [comment comments fo] Appends comment lines to an object *)

val create :
    ?comments:comments ->
    ?orientation:orientation ->     (** Landscape *)
    ?justification:just ->          (** Center *)
    ?units:units_name ->            (** Metric *)
    ?papersize:papersize ->         (** A4 *)
    ?magnification:float ->         (** 100.0 *)
    ?multiple_page:multiple_page -> (** Single *)
    ?transparent_color:int ->       (** -2 *)
    ?resolution:units * int ->      (** 1200, 2 *)
      fig_object list -> t
(** [create ?... fig_objects] creates a FIG *)

val write : string -> t -> unit
(** [write filename fig] Dumps in a .fig file *)

val read : string -> t
(** [read filename] Parses a .fig file *)

