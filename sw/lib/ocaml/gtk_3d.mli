(*
 * $Id$
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

(** 3D display widget

   {b Dépendences : Platform, Gtk_tools_GL, Geometry_2d, Geometry_3d}
 *)

(** {6 Exceptions} *)

exception NO_SUCH_3D_OBJECT of int
(** Exception levée lors de la recherche d'un objet d'identifiant inconnu *)

exception NOT_A_3D_OUTLINE of int
(** Exception levée lorsqu'un autre objet est passé alors qu'un contour
   pays est attendu *)

exception NOT_A_3D_LINE of int
(** Idem avec une ligne *)

exception NOT_A_3D_POINT of int
(** Idem avec un point *)

type glcolor = float * float * float
(** Une couleur OpenGL *)

type t_arrow = ARROW1 | ARROW2
(** Types de fleche *)

(** [create_texture_from_image texture_filename] crée une texture à partir d'un fichier image. L'identifiant
   de la texture créée est renvoyé. *)
val create_texture_from_image : string -> GlTex.texture_id

(** [widget_3d pack with_status_bar name] crée un widget d'affichage 3D

   - [pack] indique où mettre le widget
   - [with_status_bar] permet la création d'une barre d'infos optionnelle
   - [name] désigne le nom à donner à la zone d'affichage (éventuellement affichée
   en haut à gauche de la zone)
*)
class widget_3d :
  (GObj.widget -> unit) ->
  bool ->
  string ->
  object

    (** {6 Gestion de l'affichage} *)

    method display_func : unit
    (** force l'affichage (utile après un ajout ou une modification d'objet) *)
    method change_and_redraw : unit
    (** prend en compte des modifications sur la position utilisateur (rotation
	   de la scène, éloignement) et force l'affichage *)
    method incr_dist : float -> unit
    (** [incr_dist delta] effectue une modification sur l'éloignement de
	   l'utilisateur par rapport à la scène *)
    method incr_rotx : float -> unit
    (** [incr_rotx delta_angle] tourne la scène de [delta_angle] (en degrés)
       suivant l'axe X *)
    method incr_rotz : float -> unit
    (** [incr_rotz delta_angle] tourne la scène de [delta_angle] (en degrés)
       suivant l'axe Z *)

    method  move_x : float -> unit
    (** [move_x d] déplacement de la vue sur l'axe X *)

    method  move_y : float -> unit
    (** [move_y d] déplacement de la vue sur l'axe Y *)

    method move_view : Gl.point3 -> unit
    (** [move_view point] place l'utilisateur à la position indiquée par [point] *)
    method rotate_view : float * float * float -> unit
    (** [rotate_view (angle_x, angle_y, angle_z)] effectue les rotations de la scène
	   indiquées pour chaque axe *)

    method incr_light_rot : float -> unit
    (** [incr_light_rot delta_angle] ajoute [delta_angle] à la rotation sur l'axe Z
	   de la source de lumière *)

    method set_name : string -> unit
	(** [set_name name] modifie le nom associé au widget 3D *)
    method set_show_name : bool -> unit
	(** [set_show_name show] affiche/masque le nom associé au widget 3D *)

    method set_back_color : glcolor -> unit
	(** [set_back_color couleur] met à jour la couleur de fond de la zone de dessin 3D *)

    (** {6 Rosace} *)

    method rosace_off : unit
    (** masque la rosace *)
    method rosace_on : unit
    (** affiche la rosace *)
    method rosace_switch : unit
    (** change l'état d'affichage de la rosace suivant la valeur de [show_rosace] *)

    (** {6 Ajout/suppression d'objets} *)

    method destroy_all_objects : unit
    (** Destruction de tous les objets existants *)

    method add_object_arrow :
	Geometry_3d.pt_3D -> Geometry_3d.pt_3D -> bool -> float -> float -> glcolor ->
	  bool -> t_arrow -> int
	(** [add_object_arrow ptA ptB sens ep lg color filled arrow_type] crée une fleche :
	   - [ptA] indique le point où se trouve la pointe de la fleche ou la
	   seconde extrémité
	   - [ptB] second point qui donne la direction de la fleche
	   - [sens] est vrai si [ptA] désigne la pointe de la fleche, faut s'il désigne
	   l'autre extrémité de la fleche
	   - [ep] épaisseur de la flèche
	   - [lg] sa longueur
	   - [color] la couleur
	   - [filled] indique si la fleche est remplie ou en fil de fer
	   - [arrow_type] désigne le type de la fleche

	   L'identifiant de l'objet créé est renvoyé
	 *)
    method add_object_line :
      Geometry_3d.pt_3D list -> glcolor -> int -> bool -> bool -> int
	(** [add_object_line lst_points color line_width with_bars fill] crée un objet
	   [LINE_3D] où

	   - [lst_points] indique la liste des points de la ligne
	   - [color] désigne sa couleur
	   - [line_width] épaisseur
	   - [with_bars] indique si la ligne est affichée avec des barres verticales
	   - [fill] indique si la surface définie par la ligne et sa projection au
	   sol est affichée

	   L'identifiant de l'objet créé est renvoyé
	 *)

    method add_object_outline :
      Geometry_3d.pt_3D list -> glcolor -> glcolor -> bool -> int
	(** [add_object_outline contour color_in color_out filled] crée un objet
	   [OUTLINE_3D] et renvoie son identifiant *)
    method add_object_point :
      Geometry_3d.pt_3D -> Geometry_3d.pt_3D -> string -> glcolor -> bool -> int
    (** [add_object_point pos pos2 name color with_name] crée un [POINT_3D] avec :
	   - [pos] indique la position 3D du point
	   - [pos2] indique la position 3D où doit etre affiché son nom
	   - [name] le nom associé au point
	   - [color] la couleur du point
	   - [with_name] indique si le nom du point doit etre affiché

	   L'identifiant de l'objet créé est renvoyé
	 *)
    method add_object_volume_simple :
      Geometry_2d.pt_2D list -> float -> float -> glcolor -> bool -> int
	(** [add_object_volume_simple contour zmin zmax color filled] crée un objet
	   [VOLUME_3D] où :

	   - [contour] désigne la surface (2D) définissant les faces inférieure
	   et supérieure du volume
	   - [zmin] et [zmax] indiquent repsectivement l'altitude min et l'altitude max
	   du volume
	   - [color] désigne la couleur à lui appliquer
	   - [filled] permet d'afficher le volume plein ou en mode fil de fer
	 *)

	method add_object_enveloppe :
        Geometry_3d.pt_3D list ->
          Geometry_3d.pt_3D list -> glcolor -> bool -> int
	(** [add_object_enveloppe contour_haut contour_bas color filled] *)

    method add_object_enveloppe_double :
        Geometry_3d.pt_3D list ->
          Geometry_3d.pt_3D list -> glcolor -> glcolor -> bool -> int
	(** [add_object_enveloppe_double contour_haut contour_bas
	   color_out color_in filled] *)

	method add_object_surface : (Geometry_3d.pt_3D*glcolor) array array -> bool -> int
	(** [add_object_surface points filled] ajoute une surface, pleine ou pas suivant
	   [filled]. Elle est définie par la matrice de point [points] qui contient les
	   coordonnées et la couleur de chaque point de la grille *)

    method add_object_surface_with_texture : Geometry_3d.pt_3D array array -> GlTex.texture_id -> int
	(** [add_object_surface_with_texture points texture_id] effectue la même opération
	   que la fonction précédente sauf qu'ici, on ne précise pas de couleur pour les
	   différents points mais une texture à appliquer sur la surface obtenue. *)

    method delete_object : int -> unit
	(** [delete_object id] supprime l'objet dont l'identifiant est [id]. Si cet
	   objet n'existe pas, l'exception {!Gtk_3d.NO_SUCH_3D_OBJECT} est levée *)

    (** {6 Manipulation des objets} *)

    method display : int -> unit
    method object_get_color : int -> glcolor
    method object_get_fill : int -> bool
    method object_get_visibility : int -> bool
    method object_set_color : int -> glcolor -> unit
    method object_set_fill : int -> bool -> unit
    method object_set_visibility : int -> bool -> unit
    method outline_get_in_color : int -> glcolor
    method outline_set_in_color : int -> glcolor -> unit
    method point_get_with_name : int -> bool
    method point_set_with_name : int -> bool -> unit
    method line_get_width : int -> int
    method line_get_with_bars : int -> bool
    method line_set_width : int -> int -> unit
    method line_set_with_bars : int -> bool -> unit

	(** {6 Lighting/Smoothing} *)

    method lights_off : unit
    (** annule l'utilisation de la lumière *)
    method lights_on : unit
	(** force l'utilisation de la lumière *)
    method lights_switch : unit
    (** change l'état d'utilisation de la lumière *)

    method smooth_off : unit
    (** annule l'utilisation du lissage *)
	method smooth_on : unit
    (** force l'utilisation du lissage *)
    method smooth_switch : unit
    (** change l'état d'utilisation du lissage *)
  end
