(*
 * 2D Geometry
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec
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

(** Module 2D geometry

   By default, polygons are considered open

   {e Yann Le Fablec, version 1.0, 17/04/2003}
 *)

(** {6 Types} *)

(** Type point / 2D vector in Cartesian coordinates *)
type pt_2D = { x2D : float; y2D : float; }

(** Type point / 2D vector in polar coordinates *)
type pt_2D_polar = { r2D : float; theta2D : float; }

(** 2D Vector zero *)
val null_vector : pt_2D

(** A polygon *)
type poly_2D = pt_2D list

(** Types of intersections to cross between two segments
   [\[P1, P2\]] and [\[P3, P4\]] *)
type t_crossing =
    T_IN_SEG1     (** In segment 1 *)
  | T_IN_SEG2     (** In segment 2 *)
  | T_ON_PT1      (** On the first point of the segment 1 *)
  | T_ON_PT2      (** On the second point of the segment 1 *)
  | T_ON_PT3      (** On the first point of the segment 1 *)
  | T_ON_PT4      (** On the second point of the segment 2 *)
  | T_OUT_SEG_PT1 (** Outside the segment 1 before the first period *)
  | T_OUT_SEG_PT2 (** Outside the segment 1, after the second point *)
  | T_OUT_SEG_PT3 (** Outside of segment 2, before the first period *)
  | T_OUT_SEG_PT4 (** Outside of segment 2, after the second point *)

(** Indicates the type of a polygon *)
type t_conv = CONVEX | CONCAVE | CONV_UNDEFINED

(** Indicates the direction of a polygon (clockwise or counter clockwise) *)
type t_ccw = CW | CCW | CCW_UNDEFINED

(** {6 Angles conversions} *)

(** [deg2rad angle_degrers]  gives the corrsponding angle in radians *)
val deg2rad : float -> float

(** [rad2deg angle_radians] gives the corrsponding angle in radians *)
val rad2deg : float -> float

(** {6 Points} *)

(** [point_same A B] tests the strict equality of the two points *)
val point_same : pt_2D -> pt_2D -> bool

(** [distance A B] evaluates the distance between the points [A] and [B] *)
val distance : pt_2D -> pt_2D -> float

(** [point_middle A B] returns the middle of the segment formed by [A] and [B] *)
val point_middle : pt_2D -> pt_2D -> pt_2D

(** [barycenter lst_pts] returns the centroid points *)
val barycenter : pt_2D list -> pt_2D

(** [weighted_barycenter lst_pts lst_poids] returns the centroid points
   weighted by [lst_poids] *)
val weighted_barycenter : pt_2D list -> float list -> pt_2D

(** {6 Inclusion tests} *)

(** [point_in_poly pt poly] tests whether the point is within the polygon *)
val point_in_poly : pt_2D -> poly_2D -> bool

(** [point_in_circle pt (centre_cercle, rayon_cercle)] tests whether point
   is located in the circle *)
val point_in_circle : pt_2D -> pt_2D * float -> bool

(** {6 Vectors} *)

(** [vect_make A B] creates the vector AB *)
val vect_make : pt_2D -> pt_2D -> pt_2D

(** [vect_norm v] returns the norm of the vector *)
val vect_norm : pt_2D -> float

(** [vect_normalize v] normalizes the vector *)
val vect_normalize : pt_2D -> pt_2D

(** [vect_set_norm v norme] change [v] to be its norm *)
val vect_set_norm : pt_2D -> float -> pt_2D

(** [vect_add u v] sum of two vectors *)
val vect_add : pt_2D -> pt_2D -> pt_2D

(** [vect_sub u v] returns the subtraction [v] from [u] *)
val vect_sub : pt_2D -> pt_2D -> pt_2D

(** [vect_mul_scal v scalaire] multiply vector by a scalar *)
val vect_mul_scal : pt_2D -> float -> pt_2D

(** [vect_add_mul_scal lambda p v] return the point [p] translated by
   a vector [lambda.v] *)
val vect_add_mul_scal : float -> pt_2D -> pt_2D -> pt_2D

(** [vect_rotate_rad v angle] rotates the vector by the given angle in radians *)
val vect_rotate_rad : pt_2D -> float -> pt_2D

(** [vect_rotate v angle] rotates the vector by the given angle in degrees *)
val vect_rotate : pt_2D -> float -> pt_2D

(** [vect_rotate_90 v] returns the normal vector [v] (90 degree rotation
   counterclockwise) *)
val vect_rotate_90 : pt_2D -> pt_2D

(** [vect_inverse v] returns the inversion of vector [v] *)
val vect_inverse : pt_2D -> pt_2D

(** {6 Products} *)

(** [dot_product u v] returns the scalar product of two vectors *)
val dot_product : pt_2D -> pt_2D -> float

(** [cross_product u v] returns the cross product of [u] and [v] *)
val cross_product : pt_2D -> pt_2D -> float

(** {6 Intersections of segments/lines} *)

(** [crossing_point A u B v] tests the intersection of two lines: the first
   passing through the point [A] in the direction of vector [u]
   and the second through [B] in the direction of vector [v]

   On output, two possibilities:
   - [None] if there is no intersection
   - [Some (type1, type2, point_intersection)] otherwise. [type1] is the type
   intersection on the first line [type2] is for the second line.
 *)
val crossing_point :
  pt_2D ->
  pt_2D -> pt_2D -> pt_2D -> (t_crossing * t_crossing * pt_2D) option

(** [test_in_segment type_intersection] tests whether the intersection is in the
   segment 1 (including the end) *)
val test_in_segment : t_crossing -> bool

(** [test_on_hl type_intersection] tests whether the intersection is on the half-line
   (including the end) *)
val test_on_hl : t_crossing -> bool

(** [crossing_seg_seg A B C D] ests the intersection of segments
   [\[A,B\]] and [\[C,D\]] *)
val crossing_seg_seg : pt_2D -> pt_2D -> pt_2D -> pt_2D -> bool

(** [crossing_seg_hl A B C u] tests the intersection of the segment [\[A,B\]] and
   the line through [C] and in direction of vector [u] *)
val crossing_seg_hl :
  pt_2D -> pt_2D -> pt_2D -> pt_2D -> bool

(** [crossing_hl_hl A u B v] tests the intersection between the two half-lines *)
val crossing_hl_hl : pt_2D -> pt_2D -> pt_2D -> pt_2D -> bool

(** [crossing_lines A u B v] tests the intersection between the two lines and returns the
   point there is actually intersection *)
val crossing_lines : pt_2D -> pt_2D -> pt_2D -> pt_2D -> bool * pt_2D

(** {6 Distances} *)

(** [distance_point_line P A u] gives the distance between the point [P] and
   the line through [A] in the direction of vector [u] *)
val distance_point_line : pt_2D -> pt_2D -> pt_2D -> float

(** [distance_point_segments_list P lst_points] returns the minimum distance between point [P]
   and the segments formed by the points list [lst_points]. If the distance can
   not be calculated, None is returned) *)
val distance_point_segments_list : pt_2D -> poly_2D -> float option

(** {6 Projections} *)

(** [point_project_on_line P A u] returns the projection of point [P] on
   the line through [A] and in the direction of vector [u] *)
val point_project_on_line : pt_2D -> pt_2D -> pt_2D -> pt_2D

(** [point_project_on_segment P A B] returns the projection, if it exists,
   of point [P] to the segment [\[A, B\]] *)
val point_project_on_segment : pt_2D -> pt_2D -> pt_2D -> pt_2D option

(** [point_project_on_segments_list P lst_points] returns the projection, if it exists,
   of [P] on the segments formed by the points list [lst_points] *)
val point_project_on_segments_list : pt_2D -> poly_2D -> pt_2D option

(** {6 Polygons} *)

(** [poly_is_closed poly] returns [TRUE] if the polygon is closed *)
val poly_is_closed : poly_2D -> bool

(** [poly_close poly] closes the polygon if it is not already. So
   [\[A; B; C; D\]] becomes [\[A; B; C; D; A\]] *)
val poly_close : poly_2D -> poly_2D

(** [poly_close2 poly] does the following:
   [\[A; B; C; D\]] -> [\[|A; B; C; D; A; B|\]].
   Warning: here the returned polygon is not a list of points
   but an array of points *)
val poly_close2 : poly_2D -> pt_2D array

(** [poly_area poly] returns the area of the polygon *)
val poly_area : poly_2D -> float

(** [poly_signed_area poly] returns the signed area of the polygon *)
val poly_signed_area : poly_2D -> float

(** [poly_centroid poly] returns the centroid of the polygon *)
val poly_centroid : poly_2D -> pt_2D

(** [ccw_angle A B C] indicates the direction of the angle between the vectors AB and BC *)
val ccw_angle : pt_2D -> pt_2D -> pt_2D -> t_ccw

(** [poly_test_ccw poly] returns clockwise or counter-clockwise against the polygon *)
val poly_test_ccw : poly_2D -> t_ccw

(** [poly_test_convex poly] indicates the type (convex or not) of the polygon *)
val poly_test_convex : poly_2D -> t_conv

(** [convex_hull poly] provides the convex hull of the list of points [poly] *)
val convex_hull : poly_2D -> pt_2D list

(** [circumcircle A B C] returns the circumscribed circle of the triangle ABC in
   the form [(C, r)] where [C] is the center of the circle and [r] radius *)
val circumcircle : pt_2D -> pt_2D -> pt_2D -> pt_2D * float

(** [crossing_seg_poly A B poly] returns a list of intersections between
   the segment [\[AB\]] and the polygon. Intersections can be
   also in [A], in [B] in the vertices of the polygon *)
val crossing_seg_poly : pt_2D -> pt_2D -> poly_2D -> pt_2D list

(** [crossing_seg_poly_exclusive A B poly] same thing as before except
   [A], [B] and the vertices of the polygon are excluded *)
val crossing_seg_poly_exclusive : pt_2D -> pt_2D -> poly_2D -> pt_2D list

(** {6 Triangulation of polygons} *)

(** [in_tesselation poly] performs triangulation and returns a list of
   triplets indicating the indices of points constituting the triangles *)
val in_tesselation : poly_2D -> (int * int * int) list

(** [geom_tesselation polygone] performs the triangulation of the polygon and returns
   the resulting list of triangles. Here a triangle is a list of 3 points (!).
   Furthermore, the polygon can not contain any item in duplicate.
 *)
val tesselation : poly_2D -> poly_2D list

(** [in_tesselation_fans poly] erforms triangulation (by fans) and returns the table
   containing the points and a triple list indicating the evidence points
   (In the table) constituting the triangles_fan *)
val in_tesselation_fans : poly_2D -> pt_2D array * (int list) list

(** [geom_tesselation_fans polygone] performs the triangulation of the polygon
   triangle_fan OpenGL. Output is a list containing lists of
   points. Each list of points contains 3 points (triangle) or
   more than 3 points (for a triangle_fan).
   Furthermore, the polygon can not contain any duplicate points.
 *)
val tesselation_fans : poly_2D -> poly_2D list


(** {6 Polar Coordinates} *)

val m_pi : float

(** [cart2polar cart] *)
val cart2polar : pt_2D -> pt_2D_polar
val polar2cart : pt_2D_polar -> pt_2D

val wind_dir_from_angle_rad : float -> float
val heading_of_to_angle_rad : float -> float
val norm_angle_rad : float -> float
val norm_heading_rad : float -> float
val oposite_heading_rad : float -> float

(** {6 Misc} *)

val arc_segment : pt_2D -> pt_2D -> pt_2D -> float -> pt_2D*pt_2D*float
(** [arc_segment p0 p1 p2 r] Returns [(c,f,s)]. Point [c] is the center of the
arc of radius [r], tangent to segment [po]-[p1] in [p1]. Point [f] is such
that [f]-[p2] is tangent to the arc. [s] is 1. if [c] is on the left, [-1]
else *)

val arc : ?nb_points:int -> pt_2D -> float -> float -> float -> pt_2D array

type slice = { top : float; left_side : float * float; right_side : float * float }

val slice_polygon : pt_2D array -> slice array
(** Slices a y-monotone polygon clockwise described *)
