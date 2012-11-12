(*
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

   {b D�pendences : Platform, Gtk_tools_GL, Geometry_2d, Geometry_3d}
 *)

(** {6 Exceptions} *)

exception NO_SUCH_3D_OBJECT of int
(** Exception lev�e lors de la recherche d'un objet d'identifiant inconnu *)

exception NOT_A_3D_OUTLINE of int
(** Exception lev�e lorsqu'un autre objet est pass� alors qu'un contour
   pays est attendu *)

exception NOT_A_3D_LINE of int
(** Idem avec une ligne *)

exception NOT_A_3D_POINT of int
(** Idem avec un point *)

type glcolor = float * float * float
(** Une couleur OpenGL *)

type t_arrow = ARROW1 | ARROW2
(** Types de fleche *)

(** [create_texture_from_image texture_filename] cr�e une texture � partir d'un fichier image. L'identifiant
   de la texture cr��e est renvoy�. *)
val create_texture_from_image : string -> GlTex.texture_id

(** [widget_3d pack with_status_bar name] cr�e un widget d'affichage 3D

   - [pack] indique o� mettre le widget
   - [with_status_bar] permet la cr�ation d'une barre d'infos optionnelle
   - [name] d�signe le nom � donner � la zone d'affichage (�ventuellement affich�e
   en haut � gauche de la zone)
*)
class widget_3d :
  (GObj.widget -> unit) ->
  bool ->
  string ->
  object

    (** {6 Gestion de l'affichage} *)

    method display_func : unit
    (** force l'affichage (utile apr�s un ajout ou une modification d'objet) *)
    method change_and_redraw : unit
    (** prend en compte des modifications sur la position utilisateur (rotation
	   de la sc�ne, �loignement) et force l'affichage *)
    method incr_dist : float -> unit
    (** [incr_dist delta] effectue une modification sur l'�loignement de
	   l'utilisateur par rapport � la sc�ne *)
    method incr_rotx : float -> unit
    (** [incr_rotx delta_angle] tourne la sc�ne de [delta_angle] (en degr�s)
       suivant l'axe X *)
    method incr_rotz : float -> unit
    (** [incr_rotz delta_angle] tourne la sc�ne de [delta_angle] (en degr�s)
       suivant l'axe Z *)

    method  move_x : float -> unit
    (** [move_x d] d�placement de la vue sur l'axe X *)

    method  move_y : float -> unit
    (** [move_y d] d�placement de la vue sur l'axe Y *)

    method move_view : Gl.point3 -> unit
    (** [move_view point] place l'utilisateur � la position indiqu�e par [point] *)
    method rotate_view : float * float * float -> unit
    (** [rotate_view (angle_x, angle_y, angle_z)] effectue les rotations de la sc�ne
	   indiqu�es pour chaque axe *)

    method incr_light_rot : float -> unit
    (** [incr_light_rot delta_angle] ajoute [delta_angle] � la rotation sur l'axe Z
	   de la source de lumi�re *)

    method set_name : string -> unit
	(** [set_name name] modifie le nom associ� au widget 3D *)
    method set_show_name : bool -> unit
	(** [set_show_name show] affiche/masque le nom associ� au widget 3D *)

    method set_back_color : glcolor -> unit
	(** [set_back_color couleur] met � jour la couleur de fond de la zone de dessin 3D *)

    (** {6 Rosace} *)

    method rosace_off : unit
    (** masque la rosace *)
    method rosace_on : unit
    (** affiche la rosace *)
    method rosace_switch : unit
    (** change l'�tat d'affichage de la rosace suivant la valeur de [show_rosace] *)

    (** {6 Ajout/suppression d'objets} *)

    method destroy_all_objects : unit
    (** Destruction de tous les objets existants *)

    method add_object_arrow :
	Geometry_3d.pt_3D -> Geometry_3d.pt_3D -> bool -> float -> float -> glcolor ->
	  bool -> t_arrow -> int
	(** [add_object_arrow ptA ptB sens ep lg color filled arrow_type] cr�e une fleche :
	   - [ptA] indique le point o� se trouve la pointe de la fleche ou la
	   seconde extr�mit�
	   - [ptB] second point qui donne la direction de la fleche
	   - [sens] est vrai si [ptA] d�signe la pointe de la fleche, faut s'il d�signe
	   l'autre extr�mit� de la fleche
	   - [ep] �paisseur de la fl�che
	   - [lg] sa longueur
	   - [color] la couleur
	   - [filled] indique si la fleche est remplie ou en fil de fer
	   - [arrow_type] d�signe le type de la fleche

	   L'identifiant de l'objet cr�� est renvoy�
	 *)
    method add_object_line :
      Geometry_3d.pt_3D list -> glcolor -> int -> bool -> bool -> int
	(** [add_object_line lst_points color line_width with_bars fill] cr�e un objet
	   [LINE_3D] o�

	   - [lst_points] indique la liste des points de la ligne
	   - [color] d�signe sa couleur
	   - [line_width] �paisseur
	   - [with_bars] indique si la ligne est affich�e avec des barres verticales
	   - [fill] indique si la surface d�finie par la ligne et sa projection au
	   sol est affich�e

	   L'identifiant de l'objet cr�� est renvoy�
	 *)

    method add_object_outline :
      Geometry_3d.pt_3D list -> glcolor -> glcolor -> bool -> int
	(** [add_object_outline contour color_in color_out filled] cr�e un objet
	   [OUTLINE_3D] et renvoie son identifiant *)
    method add_object_point :
      Geometry_3d.pt_3D -> Geometry_3d.pt_3D -> string -> glcolor -> bool -> int
    (** [add_object_point pos pos2 name color with_name] cr�e un [POINT_3D] avec :
	   - [pos] indique la position 3D du point
	   - [pos2] indique la position 3D o� doit etre affich� son nom
	   - [name] le nom associ� au point
	   - [color] la couleur du point
	   - [with_name] indique si le nom du point doit etre affich�

	   L'identifiant de l'objet cr�� est renvoy�
	 *)
    method add_object_volume_simple :
      Geometry_2d.pt_2D list -> float -> float -> glcolor -> bool -> int
	(** [add_object_volume_simple contour zmin zmax color filled] cr�e un objet
	   [VOLUME_3D] o� :

	   - [contour] d�signe la surface (2D) d�finissant les faces inf�rieure
	   et sup�rieure du volume
	   - [zmin] et [zmax] indiquent repsectivement l'altitude min et l'altitude max
	   du volume
	   - [color] d�signe la couleur � lui appliquer
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
	   [filled]. Elle est d�finie par la matrice de point [points] qui contient les
	   coordonn�es et la couleur de chaque point de la grille *)

    method add_object_surface_with_texture : Geometry_3d.pt_3D array array -> GlTex.texture_id -> int
	(** [add_object_surface_with_texture points texture_id] effectue la m�me op�ration
	   que la fonction pr�c�dente sauf qu'ici, on ne pr�cise pas de couleur pour les
	   diff�rents points mais une texture � appliquer sur la surface obtenue. *)

    method delete_object : int -> unit
	(** [delete_object id] supprime l'objet dont l'identifiant est [id]. Si cet
	   objet n'existe pas, l'exception {!Gtk_3d.NO_SUCH_3D_OBJECT} est lev�e *)

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
    (** annule l'utilisation de la lumi�re *)
    method lights_on : unit
	(** force l'utilisation de la lumi�re *)
    method lights_switch : unit
    (** change l'�tat d'utilisation de la lumi�re *)

    method smooth_off : unit
    (** annule l'utilisation du lissage *)
	method smooth_on : unit
    (** force l'utilisation du lissage *)
    method smooth_switch : unit
    (** change l'�tat d'utilisation du lissage *)
  end
