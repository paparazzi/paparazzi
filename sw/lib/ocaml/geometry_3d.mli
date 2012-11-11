(*
 * 3D Geometry
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

(** Module de géométrie 3D

   Par défaut, les polygones sont considérés comme étant ouverts

   {b Dépendences : Geometry_2d}

   {e Yann Le Fablec, version 1.0, 22/04/2003}
 *)

(** {6 Types} *)

(** Types pour les différents axes *)
type t_axis3d = T_X3D | T_Y3D | T_Z3D

(** Types pour les intersections *)
type t_crossing3d = T_IN_SEG1 | T_IN_SEG2 | T_ON_PT1 | T_ON_PT2 | T_ON_PT3
| T_ON_PT4 | T_OUT_SEG_PT1 | T_OUT_SEG_PT2 | T_OUT_SEG_PT3 | T_OUT_SEG_PT4

(** Type point/vecteur 3D *)
type pt_3D = { x3D : float; y3D : float; z3D : float; }

(** Vecteur nul en 3D *)
val null_vector : pt_3D

(** Un polygone 3D *)
type poly_3D = pt_3D list

(** Un volume 3D *)
type volume_3D = poly_3D list

(** {6 Points} *)

(** [point_same A B] teste l'égalité stricte des deux points *)
val point_same : pt_3D -> pt_3D -> bool

(** [distance A B] évalue la distance entre les points [A] et [B] *)
val distance : pt_3D -> pt_3D -> float

(** [point_middle A B] renvoie le milieu du segment formé par [A] et [B] *)
val point_middle : pt_3D -> pt_3D -> pt_3D

(** [barycenter lst_pts] renvoie le barycentre des points *)
val barycenter : pt_3D list -> pt_3D

(** [weighted_barycenter lst_pts lst_poids] renvoie le barycentre des points
   pondérés par [lst_poids] *)
val weighted_barycenter : pt_3D list -> float list -> pt_3D

(** [pt_3d_to_pt_2d axis pt] transforme le point 3D en point 2D en supprimant
   la coordonnée indiquée *)
val pt_3d_to_pt_2d : t_axis3d -> pt_3D -> Geometry_2d.pt_2D

(** [point_in_plane P A B C] indique si le point [P] est dans le plan défini
   par les trois points [A], [B] et [C] *)
val point_in_plane : pt_3D -> pt_3D -> pt_3D -> pt_3D -> bool

(** [point_on_segment P A B] indique si le point P se trouve sur le segment [\[A, B\]] *)
val point_on_segment : pt_3D -> pt_3D -> pt_3D -> bool

(** {6 Vecteurs} *)


(** [vect_make A B] crée le vecteur AB *)
val vect_make : pt_3D -> pt_3D -> pt_3D

(** [vect_norm v] renvoie la norme du vecteur *)
val vect_norm : pt_3D -> float

(** [vect_normalize v] normalise le vecteur *)
val vect_normalize : pt_3D -> pt_3D

(** [vect_set_norm v norme] change [v] pour que sa norme soit [norme] *)
val vect_set_norm : pt_3D -> float -> pt_3D

(** [vect_add u v] réalise la somme des deux vecteurs*)
val vect_add : pt_3D -> pt_3D -> pt_3D

(** [vect_sub u v] renvoie la soustraction de [v] à [u] *)
val vect_sub : pt_3D -> pt_3D -> pt_3D

(** [vect_mul_scal v scalaire] multiple le vecteur par un scalaire *)
val vect_mul_scal : pt_3D -> float -> pt_3D

(** [vect_add_mul_scal lambda p v] renvoie le point [p] translaté du
   vecteur [lambda.v] *)
val vect_add_mul_scal : float -> pt_3D -> pt_3D -> pt_3D

(** [vect_inverse v] renvoie le vecteur opposé à [v] *)
val vect_inverse : pt_3D -> pt_3D

(** [normal u v] renvoie la normale unitaire du plan défini par
   les vecteurs [u] et [v] *)
val normal : pt_3D -> pt_3D -> pt_3D


(** {6 Produits} *)

(** [dot_product u v] fournit le produit scalaire des deux vecteurs *)
val dot_product : pt_3D -> pt_3D -> float

(** [cross_product u v] renvoie le produit vectoriel de [u] et [v] *)
val cross_product : pt_3D -> pt_3D -> pt_3D


(** {6 Intersections} *)


(** [test_in_segment type_intersection] teste si l'intersection est dans le
   segment (extrémités incluses) *)
val test_in_segment : t_crossing3d -> bool

(** [test_on_hl type_intersection] teste si l'intersection est sur la demi-droite
   (extrémité incluse) *)
val test_on_hl : t_crossing3d -> bool

(** [crossing_point A u B v] teste l'intersection des droites [(A, u)] et [(B, v)]

   En sortie, deux possibilités :
   - [None] s'il n'y a pas d'intersection
   - [Some (type1, type2, point_intersection)] sinon. [type1] désigne le type
   d'intersection sur la première droite et [type2] la meme information pour
   la seconde droite.
 *)
val crossing_point : pt_3D -> pt_3D -> pt_3D -> pt_3D ->
  (t_crossing3d * t_crossing3d * pt_3D) option

(** [crossing_seg_seg A B C D] teste l'intersection des segments
   [\[A,B\]] et [\[C,D\]] *)
val crossing_seg_seg : pt_3D -> pt_3D -> pt_3D -> pt_3D -> bool

(** [crossing_seg_hl A B C u] teste l'intersection entre le segment [\[A,B\]] et
   la droite passant par [C] et de vecteur directeur [u] *)
val crossing_seg_hl :
  pt_3D -> pt_3D -> pt_3D -> pt_3D -> bool

(** [crossing_hl_hl A u B v] teste l'intersection entre les deux demi-droites *)
val crossing_hl_hl : pt_3D -> pt_3D -> pt_3D -> pt_3D -> bool

(** [crossing_lines A u B v] teste l'intersection entre les deux droites et renvoie le
   point s'il y a effectivement intersection *)
val crossing_lines : pt_3D -> pt_3D -> pt_3D -> pt_3D -> bool * pt_3D

(** [crossing_line_plane A u C D E] teste l'intersection de la droite passant par [A]
   et de vecteur directeur [u] avec le plan défini par les trois points [C], [D] et [E] *)
val crossing_line_plane : pt_3D -> pt_3D -> pt_3D -> pt_3D -> pt_3D -> pt_3D option

(** [crossing_hline_plane A u C D E] teste l'intersection de la demi-droite issue de [A]
   et de vecteur directeur [u] avec le plan défini par les trois points [C], [D] et [E] *)
val crossing_hline_plane : pt_3D -> pt_3D -> pt_3D -> pt_3D -> pt_3D -> pt_3D option


(** {6 Polygones} *)

(** [poly_3d_to_poly_2d axis poly] projete le polygone 3D en 2D en supprimant les
   coordonnées de l'axe indiqué *)
val poly_3d_to_poly_2d : t_axis3d -> poly_3D -> Geometry_2d.poly_2D

(** [poly_3d_to_poly_2d_smallest_span poly] idem en choisissant l'axe le plus approprié
   numériquement (i.e celui d'étendue la plus faible) *)
val poly_3d_to_poly_2d_smallest_span : poly_3D -> Geometry_2d.poly_2D*t_axis3d

(** [span_poly poly] renvoie, pour chaque axe, l'étendue du polygone *)
val span_poly : poly_3D -> float*float*float

(** [point_in_poly pt poly] teste si le point est dans le polygone 3D *)
val point_in_poly : pt_3D -> poly_3D -> bool

(** [point_in_poly_2D pt poly] teste si le point est dans le polygone quand les deux
   sont projetés en 2D (en supprimant la coordonnée la plus appropriée) *)
val point_in_poly_2D : pt_3D -> poly_3D -> bool

(** [poly_area poly] renvoie l'aire du polygone *)
val poly_area : poly_3D -> float

(** [poly_signed_area poly] renvoie l'aire signée du polygone *)
val poly_signed_area : poly_3D -> float

(** [poly_centroid poly] renvoie le centroide du polygone *)
val poly_centroid : poly_3D -> pt_3D

(** [poly_normal poly] renvoie la normale unitaire au plan contenant le polygone *)
val poly_normal : poly_3D -> pt_3D


(** {6 Triangulation de polygones} *)


(** [geom_tesselation polygone] effectue la triangulation du polygone et renvoie
   la liste des triangles résultants. Ici un triangle est une liste de 3 points (!).
   De plus, le polygone ne doit contenir aucun point en double.
 *)
val tesselation : poly_3D -> poly_3D list

(** [geom_tesselation_fans polygone] effectue la triangulation du polygone en
   triangle_fan OpenGL. En sortie est renvoyée une liste contenant des listes
   de points. Chacune de ces listes de points contient soit 3 points (triangle)
   soit plus de 3 points (pour un triangle_fan).
   De plus, le polygone ne doit contenir aucun point en double.
 *)
val tesselation_fans : poly_3D -> poly_3D list


(** {6 Volumes} *)


(** [point_in_volume P volume] teste si le point [P] se trouve dans le volume
   (ce dernier peut ne pas etre convexe) *)
val point_in_volume : pt_3D -> volume_3D -> bool
