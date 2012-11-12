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

(** Module de géométrie 2D

   Par défaut, les polygones sont considérés comme étant ouverts

   {e Yann Le Fablec, version 1.0, 17/04/2003}
 *)

(** {6 Types} *)

(** Type point/vecteur 2D en coordonnees cartesiennes *)
type pt_2D = { x2D : float; y2D : float; }

(** Type point/vecteur 2D en coordonnees polaires *)
type pt_2D_polar = { r2D : float; theta2D : float; }

(** Vecteur nul en 2D *)
val null_vector : pt_2D

(** Un polygone *)
type poly_2D = pt_2D list

(** Types d'intersections pour le croisement entre deux segments
   [\[P1, P2\]] et [\[P3, P4\]] *)
type t_crossing =
    T_IN_SEG1     (** Dans le segment 1 *)
  | T_IN_SEG2     (** Dans le segment 2 *)
  | T_ON_PT1      (** Sur le premier point du segment 1 *)
  | T_ON_PT2      (** Sur le second point du segment 1 *)
  | T_ON_PT3      (** Sur le premier point du segment 1 *)
  | T_ON_PT4      (** Sur le second point du segment 2 *)
  | T_OUT_SEG_PT1 (** En dehors du segment 1, avant le premier point *)
  | T_OUT_SEG_PT2 (** En dehors du segment 1, après le second point *)
  | T_OUT_SEG_PT3 (** En dehors du segment 2, avant le premier point *)
  | T_OUT_SEG_PT4 (** En dehors du segment 2, après le second point *)

(** indique le type d'un polygone *)
type t_conv = CONVEX | CONCAVE | CONV_UNDEFINED

(** indique le sens d'un polygone (horaire ou contre-horaire) *)
type t_ccw = CW | CCW | CCW_UNDEFINED

(** {6 Conversions d'angles} *)

(** [deg2rad angle_degres] donne l'angle correpondant en radians *)
val deg2rad : float -> float

(** [rad2deg angle_radians] donne l'angle correpondant en degrés *)
val rad2deg : float -> float

(** {6 Points} *)

(** [point_same A B] teste l'égalité stricte des deux points *)
val point_same : pt_2D -> pt_2D -> bool

(** [distance A B] évalue la distance entre les points [A] et [B] *)
val distance : pt_2D -> pt_2D -> float

(** [point_middle A B] renvoie le milieu du segment formé par [A] et [B] *)
val point_middle : pt_2D -> pt_2D -> pt_2D

(** [barycenter lst_pts] renvoie le barycentre des points *)
val barycenter : pt_2D list -> pt_2D

(** [weighted_barycenter lst_pts lst_poids] renvoie le barycentre des points
   pondérés par [lst_poids] *)
val weighted_barycenter : pt_2D list -> float list -> pt_2D

(** {6 Tests d'inclusion} *)

(** [point_in_poly pt poly] teste si le point se trouve dans le polygone *)
val point_in_poly : pt_2D -> poly_2D -> bool

(** [point_in_circle pt (centre_cercle, rayon_cercle)] teste si le point
   se trouve dans le cercle indiqué *)
val point_in_circle : pt_2D -> pt_2D * float -> bool

(** {6 Vecteurs} *)

(** [vect_make A B] crée le vecteur AB *)
val vect_make : pt_2D -> pt_2D -> pt_2D

(** [vect_norm v] renvoie la norme du vecteur *)
val vect_norm : pt_2D -> float

(** [vect_normalize v] normalise le vecteur *)
val vect_normalize : pt_2D -> pt_2D

(** [vect_set_norm v norme] change [v] pour que sa norme soit [norme] *)
val vect_set_norm : pt_2D -> float -> pt_2D

(** [vect_add u v] réalise la somme des deux vecteurs*)
val vect_add : pt_2D -> pt_2D -> pt_2D

(** [vect_sub u v] renvoie la soustraction de [v] à [u] *)
val vect_sub : pt_2D -> pt_2D -> pt_2D

(** [vect_mul_scal v scalaire] multiple le vecteur par un scalaire *)
val vect_mul_scal : pt_2D -> float -> pt_2D

(** [vect_add_mul_scal lambda p v] renvoie le point [p] translaté du
   vecteur [lambda.v] *)
val vect_add_mul_scal : float -> pt_2D -> pt_2D -> pt_2D

(** [vect_rotate_rad v angle] tourne le vecteur de l'angle indiqué en radians *)
val vect_rotate_rad : pt_2D -> float -> pt_2D

(** [vect_rotate v angle] tourne le vecteur de l'angle indiqué en degrés *)
val vect_rotate : pt_2D -> float -> pt_2D

(** [vect_rotate_90 v] renvoie le vecteur normal à [v] (rotation de 90 degrés
   dans le sens trigonométrique) *)
val vect_rotate_90 : pt_2D -> pt_2D

(** [vect_inverse v] renvoie le vecteur opposé à [v] *)
val vect_inverse : pt_2D -> pt_2D

(** {6 Produits} *)

(** [dot_product u v] fournit le produit scalaire des deux vecteurs *)
val dot_product : pt_2D -> pt_2D -> float

(** [cross_product u v] renvoie le produit vectoriel de [u] et [v] *)
val cross_product : pt_2D -> pt_2D -> float

(** {6 Intersections de segments/droites} *)

(** [crossing_point A u B v] teste l'intersection de deux droites : la première
   passant par le point [A] et de vecteur directeur [u] et la seconde passant par [B]
   et de vecteur directeur [v].

   En sortie, deux possibilités :
   - [None] s'il n'y a pas d'intersection
   - [Some (type1, type2, point_intersection)] sinon. [type1] désigne le type
   d'intersection sur la première droite et [type2] la meme information pour
   la seconde droite.
 *)
val crossing_point :
  pt_2D ->
  pt_2D -> pt_2D -> pt_2D -> (t_crossing * t_crossing * pt_2D) option

(** [test_in_segment type_intersection] teste si l'intersection est dans le
   segment 1 (extrémités incluses) *)
val test_in_segment : t_crossing -> bool

(** [test_on_hl type_intersection] teste si l'intersection est sur la demi-droite
   (extrémité incluse) *)
val test_on_hl : t_crossing -> bool

(** [crossing_seg_seg A B C D] teste l'intersection des segments
   [\[A,B\]] et [\[C,D\]] *)
val crossing_seg_seg : pt_2D -> pt_2D -> pt_2D -> pt_2D -> bool

(** [crossing_seg_hl A B C u] teste l'intersection entre le segment [\[A,B\]] et
   la droite passant par [C] et de vecteur directeur [u] *)
val crossing_seg_hl :
  pt_2D -> pt_2D -> pt_2D -> pt_2D -> bool

(** [crossing_hl_hl A u B v] teste l'intersection entre les deux demi-droites *)
val crossing_hl_hl : pt_2D -> pt_2D -> pt_2D -> pt_2D -> bool

(** [crossing_lines A u B v] teste l'intersection entre les deux droites et renvoie le
   point s'il y a effectivement intersection *)
val crossing_lines : pt_2D -> pt_2D -> pt_2D -> pt_2D -> bool * pt_2D

(** {6 Distances} *)

(** [distance_point_line P A u] renvoie la distance entre le point [P] et
   la droite passant par [A] de vecteur directeur [u] *)
val distance_point_line : pt_2D -> pt_2D -> pt_2D -> float

(** [distance_point_segments_list P lst_points] renvoie la distance mini entre [P]
   et les segments formés par la liste de points [lst_points]. La distance peut
   ne pas etre définie (auquel cas None est renvoyé) *)
val distance_point_segments_list : pt_2D -> poly_2D -> float option

(** {6 Projections} *)

(** [point_project_on_line P A u] renvoie la projection du point [P] sur
   la droite passant par [A] de vecteur directeur [u] *)
val point_project_on_line : pt_2D -> pt_2D -> pt_2D -> pt_2D

(** [point_project_on_segment P A B] renvoie la projection, si elle existe,
   du point [P] sur le segment [\[A, B\]] *)
val point_project_on_segment : pt_2D -> pt_2D -> pt_2D -> pt_2D option

(** [point_project_on_segments_list P lst_points] renvoie la projection, si elle existe,
   de [P] sur les segments formés par la liste de points [lst_points] *)
val point_project_on_segments_list : pt_2D -> poly_2D -> pt_2D option

(** {6 Polygones} *)

(** [poly_is_closed poly] renvoie [TRUE] si le polygone est fermé *)
val poly_is_closed : poly_2D -> bool

(** [poly_close poly] ferme le polygone s'il ne l'est pas deja. Ainsi
   [\[A; B; C; D\]] devient [\[A; B; C; D; A\]] *)
val poly_close : poly_2D -> poly_2D

(** [poly_close2 poly] effectue l'opération suivante :
   [\[A; B; C; D\]] -> [\[|A; B; C; D; A; B|\]].
   Attention, ici le polygone retourné n'est pas une liste de points
   mais un tableau de points *)
val poly_close2 : poly_2D -> pt_2D array

(** [poly_area poly] renvoie l'aire du polygone *)
val poly_area : poly_2D -> float

(** [poly_signed_area poly] renvoie l'aire signée du polygone *)
val poly_signed_area : poly_2D -> float

(** [poly_centroid poly] renvoie le centroide du polygone *)
val poly_centroid : poly_2D -> pt_2D

(** [ccw_angle A B C] indique le sens de l'angle formé par les vecteurs AB et BC *)
val ccw_angle : pt_2D -> pt_2D -> pt_2D -> t_ccw

(** [poly_test_ccw poly] renvoie le sens horaire ou contre-horaire du polygone *)
val poly_test_ccw : poly_2D -> t_ccw

(** [poly_test_convex poly] indique le type (convexe ou pas) du polygone *)
val poly_test_convex : poly_2D -> t_conv

(** [convex_hull poly] fournit l'enveloppe convexe de la liste de points [poly] *)
val convex_hull : poly_2D -> pt_2D list

(** [circumcircle A B C] renvoie le cercle circonscrit au triangle ABC sous la
   forme [(C, r)] où [C] désigne le centre du cercle et [r] son rayon *)
val circumcircle : pt_2D -> pt_2D -> pt_2D -> pt_2D * float

(** [crossing_seg_poly A B poly] renvoie la liste des intersections entre
   le segment [\[AB\]] et le polygone. Les intersections peuvent se trouver
   en [A], en [B] ou sur des sommets du polygone *)
val crossing_seg_poly : pt_2D -> pt_2D -> poly_2D -> pt_2D list

(** [crossing_seg_poly_exclusive A B poly] meme chose que précédemment sauf
   que [A], [B] et les sommets du polygones sont exclus *)
val crossing_seg_poly_exclusive : pt_2D -> pt_2D -> poly_2D -> pt_2D list

(** {6 Triangulation de polygones} *)

(** [in_tesselation poly] effectue la triangulation et renvoie une liste de
   triplets indiquant les indices des points constituant les triangles *)
val in_tesselation : poly_2D -> (int * int * int) list

(** [geom_tesselation polygone] effectue la triangulation du polygone et renvoie
   la liste des triangles résultants. Ici un triangle est une liste de 3 points (!).
   De plus, le polygone ne doit contenir aucun point en double.
 *)
val tesselation : poly_2D -> poly_2D list

(** [in_tesselation_fans poly] effectue la triangulation (en fans) et renvoie le tableau
   contenant les points et une liste de triplets indiquant les indices des points
   (dans le tableau) constituant les triangles_fan *)
val in_tesselation_fans : poly_2D -> pt_2D array * (int list) list

(** [geom_tesselation_fans polygone] effectue la triangulation du polygone en
   triangle_fan OpenGL. En sortie est renvoyée une liste contenant des listes
   de points. Chacune de ces listes de points contient soit 3 points (triangle)
   soit plus de 3 points (pour un triangle_fan).
   De plus, le polygone ne doit contenir aucun point en double.
 *)
val tesselation_fans : poly_2D -> poly_2D list


(** {6 Coordonnees polaires} *)

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
