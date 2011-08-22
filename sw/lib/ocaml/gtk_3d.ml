(*
 * $Id$
 *
 * 3D display widget
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

(* Scale + lighting = pb si pas de GlNormalize *)
(* Ne pas oublier de faire un area#make_current () avant d'appeler des  *)
(* commandes OpenGL car s'il y a plusieurs fenetres ouvertes, il arrive *)
(* que les commandes ne soient pas envoyees dans la bonne... *)

(* Modules gtk/Gdk *)
open GMain
open GdkKeysyms
open GlGtk
open Gtk_tools_GL
open Platform
open Gtkgl_Hack

(* Modules locaux *)
open Geometry_2d
open Geometry_3d

(* Nombre de widgets 3D crees *)
let nb_objects = ref 0

(* Temps en millisecondes pour l'animation *)
let tps_anim = 40

(* Exception levee lors de la recherche d'un objet d'identifiant inconnu *)
exception NO_SUCH_3D_OBJECT of int
(* Exception levee lorsqu'un autre objet est passe alors qu'un contour
   pays est attendu *)
exception NOT_A_3D_OUTLINE of int
(* Idem avec une ligne *)
exception NOT_A_3D_LINE of int
(* Idem avec un point *)
exception NOT_A_3D_POINT of int

(* Debugging *)
let debug_3d = false
(* Les fontes ne sont disponibles que sous Linux... *)
let fonts_available = not platform_is_win32

(* Une couleur OpenGL *)
type glcolor = float*float*float

let glcolor_white = (1., 1., 1.)
let glcolor_black = (0., 0., 0.)
let color_rosace  = (1., 0., 0.)

(* Point 2D *)
type glpoint2d = float*float

(* Point 3D *)
type glpoint3d = float*float*float

(* Point ou vecteur 3D nul *)
let pt_null      = (0., 0., 0.)

(* Point ou vecteur 3D indefini *)
let pt_undefined = (9999.99, 9999.99, 9999.99)

(* Objet non compile *)
let not_compiled_obj = Obj.magic ~-1

(* Type pour designer les axes *)
type t_coord = X_AXIS | Y_AXIS | Z_AXIS

(* Type des action possibles a la souris dans un widget 3D *)
type t_action = ACTION_NONE | ACTION_ZOOM of (int*int) | ACTION_ROTATE of (int*int)

(* Definition des curseurs utilises *)
let cursor_standard  = Gdk.Cursor.create `LEFT_PTR
let cursor_zoom_up   = Gdk.Cursor.create `BASED_ARROW_UP
let cursor_zoom_down = Gdk.Cursor.create `BASED_ARROW_DOWN
let cursor_rotate    = Gdk.Cursor.create `EXCHANGE

(* Valeurs OpenGL pour utiliser une source de lumiere *)
let lights = [`lighting; `light0; `color_material]

(* Types de fleche *)
type t_arrow = ARROW1 | ARROW2

(* Type de polygone *)
type t_triangulation =
    NO_TRI of glpoint3d list               (* polygone convexe non triangule       *)
  | TRI_WITH_FANS of (glpoint3d list) list (* triangule avec des triangles fans    *)
  | TRI_STD of glpoint3d list              (* triangule avec des triangles simples *)

(* Volume 3D *)
type vol3d = {
    vol3d_contour            : glpoint3d list ;  (* Faces verticales (quad_strip) *)
    vol3d_up                 : t_triangulation ; (* Face horizontale superieure   *)
    vol3d_down               : t_triangulation ; (* Face horizontale inferieure   *)
    mutable vol3d_color      : glcolor ;         (* Couleur du volume             *)
    mutable vol3d_filled     : bool              (* Volume plein ou fil de fer    *)
  }


(* Volume d'une enveloppe 3D *)
type env3d ={
    env3d_contour        : glpoint3d list ;  (* Faces laterales (quad_strip)  *)
    mutable env3d_color  : glcolor ;         (* Couleur du volume             *)
    mutable env3d_filled : bool              (* Volume plein ou fil de fer    *)
  }

(* Volume d'une enveloppe 3D *)
type env3d_double ={
    env3d_double_contour_out       : glpoint3d list ; (* Faces laterales externes (quad_strip) *)
    env3d_double_contour_in        : glpoint3d list ; (* Faces laterales internes (quad_strip) *)
    mutable env3d_double_color_out : glcolor ;        (* Couleur des faces externes *)
    mutable env3d_double_color_in  : glcolor ;        (* Couleur des faces internes *)
    mutable env3d_double_filled    : bool             (* Volume plein ou fil de fer *)
  }


(* Contour 3D *)
type out3d = {
    out3d_contour            : glpoint3d list ;  (* Liste des points du contour *)
    mutable out3d_color_in   : glcolor ;         (* Couleur interieure          *)
    mutable out3d_color_out  : glcolor ;         (* Couleur du contour          *)
    mutable out3d_filled     : bool              (* Contour plein ou fil de fer *)
  }

(* Ligne 3D *)
type line3d = {
    line3d_points            : glpoint3d list ;  (* Points de la ligne          *)
    mutable line3d_width     : int ;             (* Epaisseur                   *)
    mutable line3d_color     : glcolor ;         (* Couleur                     *)
    mutable line3d_with_bars : bool ;            (* Barres verticales           *)
    mutable line3d_filled    : bool              (* Surface jusqu'au sol        *)
  }

(* Fleche 3D *)
type arr3d = {
    arr3d_contour        : (glpoint3d list) list ; (* Faces verticales (quad_strip) *)
    arr3d_pt             : glpoint3d ; (* Deplacement et rotation de la fleche car  *)
    arr3d_vect           : glpoint3d ; (* elle est creee le long de l'axe X. Il faut*)
    arr3d_angle_xy       : float ;     (* donc la remettre dans la bonne direction  *)
    arr3d_angle_z        : float ;
    mutable arr3d_color  : glcolor ;   (* Couleur de la fleche                      *)
    mutable arr3d_filled : bool        (* Fleche pleine ?                           *)
  }

(* Point 3D *)
type point3d ={
    p3d_pos               : glpoint3d ; (* Position 3D du point     *)
    p3d_pos2              : glpoint3d ; (* Position du nom du point *)
    p3d_name              : string ;    (* Nom du point             *)
    mutable p3d_with_name : bool ;      (* Affichage du nom         *)
    mutable p3d_color     : glcolor     (* Couleur du point         *)
  }

(* Surface triangulee en 3D *)
type surf3d = {
    s3d_pts            : (glpoint3d*glcolor) array array ;
    mutable s3d_filled : bool
  }

(* Surface triangulee en 3D avec une texture *)
type surf3d_tex = {
    s3d_tex_pts        : glpoint3d array array ; (* Tableau des points de la surface *)
    s3d_tex_texture_id : GlTex.texture_id        (* Id de la texture a appliquer     *)
  }

(* Type contenant tous les objets 3D possibles *)
type tobj3d =
    VOLUME1_3D of vol3d   | OUTLINE_3D of out3d
  | LINE_3D of line3d     | ARROW_3D of arr3d
  | POINT_3D of point3d   | ENVELOPPE_3D_DOUBLE of env3d_double
  | ENVELOPPE_3D of env3d | SURFACE_3D of surf3d
  | SURFACE_3D_TEX of surf3d_tex

(* Stockage d'un objet 3D *)
type obj3d = {o_obj              : tobj3d ;   (* L'objet                *)
	      o_id               : int ;      (* Son identifiant unique *)
	      mutable o_compiled : GlList.t ; (* L'objet compile        *)
	      mutable o_show     : bool       (* Objet affiche ou pas   *)
	    }

(* module OImages = OImage *)
(* module Images = Image *)

(* ============================================================================= *)
(* = Creation d'une texture a partir d'une image                               = *)
(* ============================================================================= *)
let make_image filename =
  let img = GdkPixbuf.from_file filename in
  let w = GdkPixbuf.get_width img
  and h = GdkPixbuf.get_height img
  and region = GdkPixbuf.get_pixels img in
  let n_channels = GdkPixbuf.get_n_channels img in
  let row = GdkPixbuf.get_rowstride img in

  assert(GdkPixbuf.get_bits_per_sample img = 8);
  assert(row = n_channels * w);

  let image = GlPix.create `ubyte ~format:`rgb ~width:w ~height:h in
  for i = 0 to h - 1 do
    for j = 0 to w - 1 do
      let pos = i*row + j*n_channels in
      let red = Gpointer.get_byte region ~pos in
      let green = Gpointer.get_byte region ~pos:(pos+1) in
      let blue = Gpointer.get_byte region ~pos:(pos+2) in
      Raw.sets (GlPix.to_raw image) ~pos:(3*(i*w+j))
        [| red; green; blue |]
    done
  done;
  image

let create_texture_from_image texture_filename =
  let texture = make_image texture_filename in
  let id = GlTex.gen_texture () in
  GlTex.bind_texture `texture_2d id;
  GluMisc.build_2d_mipmaps texture;
  id

(* ============================================================================= *)
(* = Infos OpenGL                                                              = *)
(* ============================================================================= *)
(* renvoie les informations relatives a la version d'OpenGL utilisee *)
let get_gl_infos () =
  let l = [("Vendor", `vendor);   ("Renderer", `renderer);
	   ("Version", `version); ("Extensions", `extensions)] in
  let s = ref "" in
  List.iter (fun (str, t) -> s:=!s^str^" : "^(GlMisc.get_string t)^"\n") l ;
  !s

(* ============================================================================= *)
(* = Affichages si mode debug                                                  = *)
(* ============================================================================= *)
(* [do_msg msg] affiche le message [msg] si {!Gtk_3d.debug_3d} est vrai *)
let do_msg msg = if debug_3d then begin Printf.printf "%s\n" msg;flush stdout end

(* ============================================================================= *)
(* = Angle modulo 360                                                          = *)
(* ============================================================================= *)
(* [mod_360 angle] angle modulo 360 degres *)
let mod_360 angle = mod_float angle 360.

(* ============================================================================= *)
(* = Manipulations de coordonnees                                              = *)
(* ============================================================================= *)
(* [get_coord (x, y, z) axis] renvoie la composante [x], [y] ou [z] suivant
   l'axe indique *)
let get_coord (x, y, z) axis = match axis with X_AXIS->x | Y_AXIS->y | Z_AXIS->z

(* [add_coord (x, y, z) axis delta] ajoute [delta] a la coordonnee definie
   par [axis] *)
let add_coord (x, y, z) axis d =
  match axis with
    X_AXIS -> (x+.d, y, z) | Y_AXIS -> (x, y+.d, z) | Z_AXIS -> (x, y, z+.d)

(* [add_coord_360 (x, y, z) axis delta] meme chose modulo 360 *)
let add_coord_360 (x, y, z) axis d =
  match axis with
    X_AXIS -> (mod_360 (x+.d), y, z)
  | Y_AXIS -> (x, mod_360 (y+.d), z)
  | Z_AXIS -> (x, y, mod_360 (z+.d))

(* ============================================================================= *)
(* = Encapsulation de fonctions 3D                                             = *)
(* ============================================================================= *)
let glpoint3d_of_pt_3d u = (u.x3D, u.y3D, u.z3D)
let glpoint3d_to_pt_3d (x, y, z) = {x3D=x; y3D=y; z3D=z}
let glpoint3d_of_pt_2d z u = (u.x2D, u.y2D, z)
let glpoint3d_of_pt_3d_lst l = List.map glpoint3d_of_pt_3d l

(* ============================================================================= *)
(* = Normale unitaire a un triplet de points                                   = *)
(* ============================================================================= *)
(* [geom_normal A B C] renvoie la normale unitaire a un triplet de point *)
let geom_normal p1 p2 p3 =
  let p1 = glpoint3d_to_pt_3d p1 and p2 = glpoint3d_to_pt_3d p2
  and p3 = glpoint3d_to_pt_3d p3 in
  let n = Geometry_3d.normal (Geometry_3d.vect_make p1 p2)
      (Geometry_3d.vect_make p1 p3) in
  glpoint3d_of_pt_3d n

let geom_scal_mult n (x, y, z) = (x*.n, y*.n, z*.n)

(* Fermeture d'un polygone *)
let geom_close_poly l = if l=[] then [] else l@[List.hd l]


(* ============================================================================= *)
(* = Encapsulation de fonctions OpenGL                                         = *)
(* ============================================================================= *)
(* polygones remplis *)
let set_gl_fillpoly   () = GlDraw.polygon_mode ~face:`both `fill
(* polygones vides (contour uniquement) *)
let unset_gl_fillpoly () = GlDraw.polygon_mode ~face:`both `line

(* [set_color color] met a jour de la couleur de dessin/remplissage *)
let set_color color = GlDraw.color color
(* [set_faded_color color pct] applique la couleur plus sombre de [pct]% *)
let set_faded_color (r, g, b) pct =
  let p = (float_of_int pct)/.100. in GlDraw.color (r*.p, g*.p, b*.p)

(* [set_3d_points type_objet_opengl lst_pts] creation d'une liste de points 3D *)
let set_3d_points typ l =
  GlDraw.begins typ ; List.iter GlDraw.vertex3 l ; GlDraw.ends ()

let set_3d_points_with_color typ l =
  GlDraw.begins typ ;
  List.iter (fun ((p, c), n) ->
    set_color c; GlDraw.normal3 n; GlDraw.vertex3 p) l ;
  GlDraw.ends ()

let set_3d_points_with_texture typ l (dx, dy) (x1, y1) =
  GlDraw.begins typ ;
  List.iter (fun ((x,y,z), n) ->
    GlDraw.normal3 n; GlTex.coord2 ((x-.x1)/.dx, (y-.y1)/.dy) ;
    GlDraw.vertex3 (x,y,z)) l ;
  GlDraw.ends ()

(* [set_3d_points_quad_strip_with_normal lst_pts] creation d'une liste de
   points 3D [quad_strip] avec les normales *)
let set_3d_points_quad_strip_with_normal l =
  let t = Array.of_list l in
  GlDraw.begins `quad_strip ;
  Array.iteri (fun i pt ->
    (* Tous les 2 points -> normale au quad_strip courant *)
    if i mod 2=0 then
      if i=0 or i=List.length l-2 then GlDraw.normal3 (geom_normal pt t.(1) t.(2))
      else GlDraw.normal3 (geom_normal pt t.(i+1) t.(i+2)) ;
    GlDraw.vertex3 pt) t ;
  GlDraw.ends ()

(* normale vers le haut *)
let normal_up   () = GlDraw.normal3 (0., 0., 1.)
(* normale vers le bas *)
let normal_down () = GlDraw.normal3 (0., 0., -.1.)

(* [rotate axis angle] effectue une rotation suivant un axe donne *)
let rotate axis angle =
  match axis with
    X_AXIS -> GlMat.rotate ~angle ~x:1.0 ()
  | Y_AXIS -> GlMat.rotate ~angle ~y:1.0 ()
  | Z_AXIS -> GlMat.rotate ~angle ~z:1.0 ()

(* [rotate_some angles axis_list] rotation suivant les axes
   indiques par [axis_list].
   La liste [angles] donne la valeur pour chaque axe concerne *)
let rotate_some angles axis_list =
  List.iter (fun axis -> rotate axis (get_coord angles axis)) axis_list
(* [rotate_all angles] rotation suivant tous les axes *)
let rotate_all angles = rotate_some angles [X_AXIS; Y_AXIS; Z_AXIS]


(* ============================================================================= *)
(* = Calcul des normales pour les surfaces 3D avec ou sans texture             = *)
(* ============================================================================= *)
let get_surface_normals tt =
  let geom_normal_in p1 p2 p3 =
    let p1 = glpoint3d_to_pt_3d p1 and p2 = glpoint3d_to_pt_3d p2
    and p3 = glpoint3d_to_pt_3d p3 in
    Geometry_3d.normal (Geometry_3d.vect_make p1 p2) (Geometry_3d.vect_make p1 p3)
  in

  Array.mapi (fun i t0 ->
    Array.mapi (fun j p ->
      if i>0 && i < Array.length tt-1 then begin
	if j>0 && j < Array.length t0-1 then begin
	  let n1 = geom_normal_in p tt.(i-1).(j) tt.(i).(j-1)
	  and n2 = geom_normal_in p tt.(i-1).(j+1) tt.(i-1).(j)
	  and n3 = geom_normal_in p tt.(i).(j+1) tt.(i-1).(j+1)
	  and n4 = geom_normal_in p tt.(i+1).(j) tt.(i).(j+1)
	  and n5 = geom_normal_in p tt.(i+1).(j-1) tt.(i+1).(j)
	  and n6 = geom_normal_in p tt.(i).(j-1) tt.(i+1).(j-1) in
	  let n = Geometry_3d.vect_add n1 n2 in
	  let n = Geometry_3d.vect_add n n3 in
	  let n = Geometry_3d.vect_add n n4 in
	  let n = Geometry_3d.vect_add n n5 in
	  let n = Geometry_3d.vect_add n n6 in
	  let n = Geometry_3d.vect_normalize n in
	  glpoint3d_of_pt_3d n
	end else (0., 0., 1.)
      end else (0., 0., 1.)) t0) tt

(* ============================================================================= *)
(* = Manipulation des objets                                                   = *)
(* ============================================================================= *)
(* [get_object_color objet] renvoie la couleur de l'objet *)
let get_object_color obj =
  match obj with
    OUTLINE_3D o          -> o.out3d_color_out
  | LINE_3D l             -> l.line3d_color
  | VOLUME1_3D v          -> v.vol3d_color
  |	ENVELOPPE_3D e        -> e.env3d_color
  |	ENVELOPPE_3D_DOUBLE e -> e.env3d_double_color_out
  | ARROW_3D a            -> a.arr3d_color
  | POINT_3D p            -> p.p3d_color
  | SURFACE_3D _s          -> glcolor_white
  | SURFACE_3D_TEX _s      -> glcolor_white

(* [set_object_color objet color] met a jour la couleur de l'objet *)
let set_object_color obj color =
  match obj with
    OUTLINE_3D o          -> o.out3d_color_out <- color
  | LINE_3D l             -> l.line3d_color    <- color
  | VOLUME1_3D v          -> v.vol3d_color     <- color
  | ENVELOPPE_3D e        -> e.env3d_color     <- color
  | ENVELOPPE_3D_DOUBLE e -> e.env3d_double_color_out<- color
  | ARROW_3D a            -> a.arr3d_color     <- color
  | POINT_3D p            -> p.p3d_color       <- color
  | _                     -> ()

(* [get_outline_in_color objet id] renvoie la couleur de remplissage d'un objet
   de type [OUTLINE_3D]. Si l'objet n'est pas de ce type, l'exception
   {!Gtk_3d.NOT_A_3D_OUTLINE} est levee *)
let get_outline_in_color obj id =
  match obj with OUTLINE_3D o -> o.out3d_color_in
  | _ -> raise (NOT_A_3D_OUTLINE id)
(* [set_outline_in_color objet color id] met a jour la couleur de remplissage pour
   un objet de type [OUTLINE_3D]. Si l'objet n'est pas de ce type, l'exception
   {!Gtk_3d.NOT_A_3D_OUTLINE} est levee *)
let set_outline_in_color obj color id =
  match obj with OUTLINE_3D o -> o.out3d_color_in <- color
  | _ -> raise (NOT_A_3D_OUTLINE id)

(* [get_object_fill objet] indique si l'objet est rempli ou en fil de fer *)
let get_object_fill obj =
  match obj with
    OUTLINE_3D o          -> o.out3d_filled
  | LINE_3D l             -> l.line3d_filled
  | VOLUME1_3D v          -> v.vol3d_filled
  |	ENVELOPPE_3D e        -> e.env3d_filled
  |	ENVELOPPE_3D_DOUBLE e -> e.env3d_double_filled
  | ARROW_3D a            -> a.arr3d_filled
  | POINT_3D _p            -> false
  | SURFACE_3D s          -> s.s3d_filled
  | SURFACE_3D_TEX _s      -> true

(* [set_object_filled objet filled] force l'objet en mode plein ou fil de fer *)
let set_object_fill obj filled =
  match obj with
    OUTLINE_3D o          -> o.out3d_filled  <- filled
  | LINE_3D l             -> l.line3d_filled <- filled
  | VOLUME1_3D v          -> v.vol3d_filled  <- filled
  |	ENVELOPPE_3D e        -> e.env3d_filled <- filled
  |	ENVELOPPE_3D_DOUBLE e -> e.env3d_double_filled <- filled
  | ARROW_3D a            -> a.arr3d_filled <- filled
  | POINT_3D _p            -> ()
  | SURFACE_3D s          -> s.s3d_filled <- filled
  | SURFACE_3D_TEX _s      -> ()

(* [get_line_width objet id] renvoie l'epaisseur d'un objet ligne. Si l'objet
   passe n'est pas du type [LINE_3D] alors l'exception {!Gtk_3d.NOT_A_3D_LINE}
   est levee *)
let get_line_width obj id =
  match obj with LINE_3D l -> l.line3d_width | _ -> raise (NOT_A_3D_LINE id)
(* [set_line_width objet width id] met a jour l'epaisseur d'un objet ligne. Si l'objet
   passe n'est pas du type [LINE_3D] alors l'exception {!Gtk_3d.NOT_A_3D_LINE}
   est levee *)
let set_line_width obj width id =
  match obj with LINE_3D l -> l.line3d_width<-width|_ -> raise (NOT_A_3D_LINE id)
(* [get_line_bars objet id] indique si l'objet est affiche avec des barres
   verticales si cet objet est du type [LINE_3D]. Si ca n'est pas le cas,
   l'exception {!Gtk_3d.NOT_A_3D_LINE} est levee *)
let get_line_bars obj id =
  match obj with LINE_3D l -> l.line3d_with_bars | _ -> raise (NOT_A_3D_LINE id)
(* [set_line_bars objet bars id] met a jour l'affichage des barres
   verticales d'un objet de type [LINE_3D]. S'il n'est pas de ce type alors
   l'exception {!Gtk_3d.NOT_A_3D_LINE} est levee *)
let set_line_bars obj bars id =
  match obj with LINE_3D l ->l.line3d_with_bars<-bars|_ ->raise (NOT_A_3D_LINE id)
(* [get_point_name objet id] indique si le nom de l'objet [POINT_3D] est
   affiche. Si l'objet passe n'est pas du type [POINT_3D],
   l'exception {!Gtk_3d.NOT_A_3D_POINT} est levee*)
let get_point_name obj id =
  match obj with POINT_3D p -> p.p3d_with_name  | _ -> raise (NOT_A_3D_POINT id)
(* [set_point_name objet name id] met a jour l'affichage ou pas du nom.
   Si l'objet passe n'est pas du type [POINT_3D],
   l'exception {!Gtk_3d.NOT_A_3D_POINT} est levee*)
let set_point_name obj name id =
  match obj with POINT_3D p ->p.p3d_with_name <-name|_ ->raise (NOT_A_3D_POINT id)



(* ============================================================================= *)
(* = Objet d'affichage 3D                                                      = *)
(* = pack            = ou mettre le widget                                     = *)
(* = with_status_bar = creation d'une barre d'infos optionnelle                = *)
(* = n               = nom de la fenetre                                       = *)
(* ============================================================================= *)
(* [widget_3d pack with_status_bar name] cree un widget d'affichage 3D

   - [pack] indique où mettre le widget
   - [with_status_bar] permet la creation d'une barre d'infos optionnelle
   - [name] designe le nom a donner a la zone d'affichage (eventuellement affichee
   en haut a gauche de la zone)
 *)
class widget_3d pack with_status_bar n =
  (* Creation de la GtkGlArea avec ou sans barre d'infos *)
  let (area, setstatus) =
    if with_status_bar then begin
      let v = Gtk_tools.create_vbox pack in
      let a = GlGtk.area [`RGBA; `DOUBLEBUFFER; `DEPTH_SIZE 1] ~packing:v#add () in
      let s = GMisc.statusbar ~packing:v#pack () in
      let ss = s#new_context ~name:"status" in
      (a, fun msg -> ignore(ss#push msg))
    end else
      (GlGtk.area [`RGBA; `DOUBLEBUFFER; `DEPTH_SIZE 1] ~packing:pack (),
       fun _msg -> ())
  in

  object (self)

      (* Numero du widget 3D *)
    val nb = !nb_objects

	(* Nom de la fenetre eventuellement affiche en haut a gauche *)
    val mutable name      = n
	(* Indique si le nom de la fenetre doit etre affiche *)
    val mutable show_name = true

	(* Contient la largeur de la fenetre apres redimensionnement *)
    val mutable width    = -1
	(* Contient la hauteur de la fenetre apres redimensionnement *)
    val mutable height   = -1
	(* Position utilisateur *)
    val mutable depl     = (0., 0., -.1.5)
	(* Rotation utilisateur *)
    val mutable rot      = (290., 0., 0.)
	(* Position de la source de lumiere *)
    val mutable lightpos = (-.0.3, -.0.3, 0.6)
	(* Rotation de la source de lumiere *)
    val mutable lightrot = pt_null

	(* Couleur du fond *)
    val mutable back_color = glcolor_black

	(* Point central de la scene *)
    val mutable extents = pt_null
	(* Rayon de la scene *)
    val mutable rs = 1.
	(* Extremes minis *)
    val mutable extreme_min = pt_undefined
	(* Extremes maxis *)
    val mutable extreme_max = pt_undefined

	(* indique si l'initialisation a ete faite *)
    val mutable done_init = false
	(* Action souris en cours s'il y en a une *)
    val mutable current_action = ACTION_NONE
	(* indique si les lumieres sont utilisees *)
    val mutable use_lights  = true
	(* indique si la source de lumiere est affichee *)
    val mutable show_light  = false
	(* smoothing ? *)
    val mutable use_smooth  = true

	(* repertoire pour les captures ecran *)
    val mutable screenshot_path   = "Captures/"
	(* nom par defaut de la capture *)
    val mutable screenshot_name   = "capture3d.png"

	(* [triangle_fan] ou [triangle] pour afficher les surfaces triangulees *)
    val mutable use_fans_for_tesselation = true

	(* increment d'eloignement lie aux touches *)
    val dist_incr = -.0.1
	(* increment de rotation lie aux touches *)
    val rot_incr  = 2.

	(* Rotation pendant l'animation *)
    val rot_anim = 0.1

	(* Compteur des objets pour les identifiants uniques *)
    val mutable cpt_obj  = 0
	(* Liste des objets 3D definis dans le widget 3D *)
    val mutable objects  = ([]:obj3d list)

	(* indique si la rosace doit etre affichee *)
    val mutable show_rosace = true
	(* Objet rosace *)
    val mutable rosace = None

	(* Fonte OpenGL si disponible (i.e sous Unix) *)
    val mutable fontbase = Obj.magic ~-1

	(* Timer utilise pour l'animation *)
    val mutable animation_timer = None

	(* cree un nouvel identifiant *)
    method private get_new_id = let n = cpt_obj in cpt_obj<-cpt_obj+1; n

      (* Indique a OpenGL que la fenetre est la fenetre courante
	 dans laquelle doivent etre effectuees les commandes OpenGL
	 A faire absolument avant d'ajouter des objets pour que la
	 bonne fenetre recoive les commandes OpenGL qui suivent...  *)
    method private make_current = area#make_current ()

	(* Mise a jour de la barre d'infos *)
    method private set_status =
      let msg = ref "" in
      let add txt = msg:=if !msg="" then txt else !msg^" "^txt in
      let (x, y, z) = rot in
      add (Printf.sprintf "X=%.0f Y=%.0f Z=%.0f" x y z) ;
      let (_, _, z) = depl in
      add (Printf.sprintf "Dist=%.1f" (-.z)) ;
      add (if use_lights then "Lights on " else "Lights off") ;
      add (if use_smooth then "Smooth on " else "Smooth off") ;
      setstatus !msg

	(* force l'utilisation de la lumiere *)
    method lights_on     = use_lights <- true;  self#update_lights
	(* annule l'utilisation de la lumiere *)
    method lights_off    = use_lights <- false; self#update_lights
	(* change l'etat d'utilisation de la lumiere *)
    method lights_switch = use_lights <- not use_lights; self#update_lights
	(* met a jour le widget pour utiliser ou pas les lumieres suivant
	   la valeur de [use_lights] *)
    method private update_lights =
      do_msg (if use_lights then "Lights on" else "Lights off") ;
      List.iter (if use_lights then Gl.enable else Gl.disable) lights ;
      self#setup; self#display_func

	(* force l'utilisation du lissage *)
    method smooth_on     = use_smooth <- true;  self#update_smooth
	(* annule l'utilisation du lissage *)
    method smooth_off    = use_smooth <- false; self#update_smooth
	(* change l'etat d'utilisation du lissage *)
    method smooth_switch = use_smooth <- not use_smooth; self#update_smooth
	(* met a jour le widget 3D pour appliquer ou pas le lissage suivant la
	   valeur de [use_smooth] *)
    method private update_smooth =
      do_msg (if use_smooth then "Smooth on" else "Smooth off") ;
      GlDraw.shade_model (if use_smooth then `smooth else `flat) ;
      self#set_status ; self#display_func

	(* affiche la rosace *)
    method rosace_on     = show_rosace <- true; self#display_func
	(* masque la rosace *)
    method rosace_off    = show_rosace <- false; self#display_func
	(* change l'etat d'affichage de la rosace suivant la valeur de [show_rosace] *)
    method rosace_switch = show_rosace <- not show_rosace; self#display_func

	(* Modification de la vue et redessin *)
    method change_and_redraw = self#setup; self#display_func

	(* Rotation/deplacement de la position utilisateur *)
    method rotate_view r = rot<-r;  self#change_and_redraw
    method move_view d   = depl<-d; self#change_and_redraw

	(* Modification du curseur *)
    method private set_cursor c =
      Gtk_tools.set_cursor area#misc#window c
    method private reset_cursor =
      Gtk_tools.set_cursor area#misc#window cursor_standard

	(* [scale_point point] met a l'echelle un point *)
    method private scale_point pt = geom_scal_mult (1./.rs) pt
	(* [scale_points lst_points] met a l'echelle une liste de points *)
    method private scale_points l = List.map self#scale_point l

	(* [draw_triangulation t] dessine une face polygonale triangulee ou
	   pas en effectuant en plus la mise a l'echelle *)
    method private draw_triangulation t =
      match t with
	NO_TRI l        -> set_3d_points `polygon (self#scale_points l)
      | TRI_STD l       -> set_3d_points `triangles (self#scale_points l)
      | TRI_WITH_FANS l -> List.iter (fun l ->
	  set_3d_points (if List.length l>3 then `triangle_fan else `triangles)
	    (self#scale_points l)) l

	    (* [create_pyramid (x, y, z) size] cree une pyramide de hauteur
	       [size] centree en [(x, y, z)] *)
    method private create_pyramid (x, y, z) size =
      let d = size*.sqrt(3.)/.2. in
      let a=d/.3. and b = (2.*.d)/.3. in
      let p0 = (x, y, z+.b) and p1 = (x, y+.b, z-.a)
      and p2 = (x-.size/.2., y-.a, z-.a) and p3 = (x+.size/.2., y-.a, z-.a) in
      set_3d_points `triangle_fan [p0; p1; p2; p3; p1] ;
      set_3d_points `triangles    [p2; p1; p3]

	(* [compile_one objet] met a l'echelle et compile l'objet *)
    method private compile_one o =
      GlList.delete o.o_compiled ;
      let compiled = GlList.create `compile in
      (match o.o_obj with
	OUTLINE_3D o ->
	  let pts = self#scale_points o.out3d_contour in
	  if o.out3d_filled then begin
	    set_gl_fillpoly () ; set_color o.out3d_color_in ;
	    (* Ici il faudrait trianguler le polygone si necessaire... *)
	    set_3d_points `polygon pts
	  end ;
	  GlDraw.line_width 2. ;
	  unset_gl_fillpoly () ; set_color o.out3d_color_out ;
	  set_3d_points `polygon pts
      | LINE_3D l    ->
	  Gl.disable `cull_face ;
	  let pts = self#scale_points l.line3d_points in
	  if l.line3d_filled or l.line3d_with_bars then begin
	    let lfill = List.flatten (List.map (fun (a, b, c) ->
	      [(a, b, 0.); (a, b, c)]) pts) in
	    if l.line3d_filled then begin
	      set_gl_fillpoly () ; set_faded_color l.line3d_color 70 ;
	      set_3d_points `quad_strip lfill
	    end ;
	    if l.line3d_with_bars then begin
	      GlDraw.line_width 2. ;
	      unset_gl_fillpoly () ; set_faded_color l.line3d_color 85 ;
	      set_3d_points `quad_strip lfill
	    end
	  end ;
	  GlDraw.line_width (float_of_int l.line3d_width) ;
	  set_color l.line3d_color ;
	  set_3d_points `line_strip pts ;
	  Gl.enable `cull_face
      | VOLUME1_3D v  ->
	  if v.vol3d_filled then set_gl_fillpoly () else begin
	    GlDraw.line_width 2. ;	unset_gl_fillpoly ()
	  end ;
	  (* Faces verticales *)
	  GlLight.color_material ~face:`front `ambient_and_diffuse ;
	  set_color v.vol3d_color ;
	  set_3d_points_quad_strip_with_normal (self#scale_points v.vol3d_contour) ;

	  (* Dessin des faces inferieure et superieure avec une couleur plus sombre *)
	  if v.vol3d_filled then begin
	    GlLight.color_material ~face:`front `ambient_and_diffuse ;
	    set_faded_color v.vol3d_color 75 ;
	    normal_up () ;   self#draw_triangulation v.vol3d_up ;
	    normal_down () ; self#draw_triangulation v.vol3d_down
	  end
      |	ENVELOPPE_3D e  ->
	  (* Faces tjs visibles: Gl.disable `cull_face ; *)
	  if e.env3d_filled then set_gl_fillpoly () else begin
	    GlDraw.line_width 2. ;	unset_gl_fillpoly ()
	  end ;

	  (* Faces verticales  *)
	  GlLight.color_material ~face:`front `ambient_and_diffuse ;
	  set_color e.env3d_color ;
	  set_3d_points_quad_strip_with_normal (self#scale_points e.env3d_contour);
          (*si faces env tjs visibles:  Gl.enable `cull_face *)
      |	ENVELOPPE_3D_DOUBLE e  ->
	  (* Faces tjs visibles: Gl.disable `cull_face ; *)
	  if e.env3d_double_filled then set_gl_fillpoly () else begin
	    GlDraw.line_width 2. ;	unset_gl_fillpoly ()
	  end ;

	  (* Faces verticales externes *)
	  GlLight.color_material ~face:`front `ambient_and_diffuse ;
	  set_color e.env3d_double_color_out ;
	  set_3d_points_quad_strip_with_normal
	    (self#scale_points e.env3d_double_contour_out);

	  (* Faces verticales internes *)
	  GlLight.color_material ~face:`front `ambient_and_diffuse ;
	  set_color e.env3d_double_color_in ;
	  set_3d_points_quad_strip_with_normal
	    (self#scale_points e.env3d_double_contour_in)

            (*si faces env tjs visibles: Gl.enable `cull_face *)
      | ARROW_3D a ->
	  if a.arr3d_filled then set_gl_fillpoly () else begin
	    GlDraw.line_width 2. ;	unset_gl_fillpoly ()
	  end ;
	  GlLight.color_material ~face:`front `ambient_and_diffuse ;
	  set_color a.arr3d_color ;

	  GlMat.push () ;
	  (* Deplace la fleche sur la pointe de la fleche *)
	  GlMat.translate3 (self#scale_point a.arr3d_pt) ;
	  (* Tourne la fleche pour qu'elle soit orientee comme voulu *)
	  GlMat.rotate3 a.arr3d_vect ~angle:a.arr3d_angle_xy ;
	  (* Tourne selon Z pour remettre la fleche comme il faut *)
	  rotate Z_AXIS a.arr3d_angle_z ;

	  (* Mise a l'echelle des points de la fleche *)
	  let l = List.map self#scale_points a.arr3d_contour in
	  (* Faces 'verticales' *)
	  set_3d_points_quad_strip_with_normal (List.flatten l) ;
	  if a.arr3d_filled then begin
	    GlLight.color_material ~face:`front `ambient_and_diffuse ;
	    set_faded_color a.arr3d_color 75 ;
	    let (up, down) =
	      (List.rev_map (fun l -> List.hd (List.tl l)) l,
	       List.map (fun l -> List.hd l) l)	in
	    (* Faces 'horizontales' *)
	    normal_up () ;   set_3d_points `polygon up ;
	    normal_down () ; set_3d_points `polygon down
	  end ;
	  GlMat.pop ()
      | POINT_3D p ->
	  let (x0, y0, z0) = self#scale_point p.p3d_pos
	  and (x, y, z)    = self#scale_point p.p3d_pos2 in
	  set_gl_fillpoly () ; set_color p.p3d_color ;
	  let size = 0.02 in
	  self#create_pyramid (x0, y0, z0) size ;

	  if p.p3d_with_name then begin
	    GlDraw.line_width 2.0 ;
	    set_3d_points `line_strip [(x, y, z); (x0, y0, z0+.size)] ;
	    if fonts_available then begin
	      GlPix.raster_pos ~x ~y ~z:(z+.0.01) () ;
	      Gtkgl_Hack.gl_print_string fontbase p.p3d_name
	    end
	  end ;

      | SURFACE_3D_TEX s ->
	  normal_up () ;
	  let tt = Array.map (Array.map self#scale_point) s.s3d_tex_pts in
	  (* On recentre les coordonnees dans la texture pour la voir en entier *)
	  (* sur toute la surface *)
	  let (x1,y1,_) = tt.(0).(0) and ttt = tt.(Array.length tt-1) in
	  let (x2,y2,_) = ttt.(Array.length ttt-1) in
	  let delta = (x2-.x1, y2-.y1) and first = (x1, y1) in

	  let t_normals = get_surface_normals tt in
	  Gl.enable `texture_2d; GlDraw.shade_model `flat ;
	  (* On utilise la texture indiquee *)
	  GlTex.bind_texture `texture_2d s.s3d_tex_texture_id;
	  Array.iteri (fun i t0 ->
	    if i<Array.length tt-1 then begin
	      let l = ref [] in
	      Array.iteri (fun j p ->
		l:=(tt.(i+1).(j), t_normals.(i+1).(j))::(p, t_normals.(i).(j))::!l) t0 ;
	      set_3d_points_with_texture `triangle_strip (List.rev !l) delta first
	    end) tt ;
	  Gl.disable `texture_2d ;
	  GlDraw.shade_model (if use_smooth then `smooth else `flat)

      | SURFACE_3D s ->
	  if s.s3d_filled then set_gl_fillpoly () else begin
	    GlDraw.line_width 2. ;	unset_gl_fillpoly ()
	  end ;
	  GlLight.color_material ~face:`front `ambient_and_diffuse ;
	  normal_up () ;
	  let t = Array.map (Array.map (fun (p, c) -> (self#scale_point p, c))) s.s3d_pts in

	  let t_normals = get_surface_normals (Array.map (Array.map fst) t) in
	  Array.iteri (fun i t0 ->
	    if i<Array.length t-1 then begin
	      let l = ref [] in
	      Array.iteri (fun j p ->
		l:=(t.(i+1).(j), t_normals.(i+1).(j))::(p, t_normals.(i).(j))::!l) t0 ;
	      set_3d_points_with_color `triangle_strip (List.rev !l)
	    end) t
      ) ;
      GlList.ends () ;
      o.o_compiled <- compiled

	  (* recompilation de tous les objets contenus dans le widget *)
    method private recompile_all_objects = List.iter self#compile_one objects
	(* [make_and_compile objet] compile uniquement l'objet indique *)
    method private make_and_compile o = self#make_current ; self#compile_one o

	(* [add_object objet] ajoute l'objet : compilation de cet objet et eventuelle
	   recompilation des autres s'il sort des extremes precedents *)
    method private add_object o =
      self#make_current ;
      let new_o = {o_obj=o; o_id=self#get_new_id;
		   o_compiled=not_compiled_obj; o_show=true} in
      objects <- new_o::objects ;
      (* Recherche des valeurs extremes de cet objet si necessaire *)
      let old_rs = rs in
      let (do_it, l) =
	match o with
	  OUTLINE_3D _o          -> (false, [])
	| LINE_3D l             -> (true, l.line3d_points)
	| VOLUME1_3D v          -> (true, v.vol3d_contour)
	| ENVELOPPE_3D e        -> (true, e.env3d_contour)
	| ENVELOPPE_3D_DOUBLE e -> (true, e.env3d_double_contour_out)
	| ARROW_3D _a            -> (false, [])
	| POINT_3D p            -> (true, [p.p3d_pos; p.p3d_pos2])
	| SURFACE_3D s          ->
	    let l = Array.to_list (Array.map (fun t -> Array.to_list t) s.s3d_pts) in
	    (true, List.map fst (List.flatten l))
	| SURFACE_3D_TEX s      ->
	    let l = Array.to_list (Array.map (fun t -> Array.to_list t) s.s3d_tex_pts) in
	    (true, List.flatten l)
      in
      if do_it then self#get_extremes l ;
      if old_rs<>rs then
	(* Il faut recompiler les autres objets car les extremes ont change... *)
	self#recompile_all_objects
      else
	(* Compilation uniquement de cet objet *)
	self#compile_one new_o ;

      (* Renvoie l'identifiant du nouvel objet cree *)
      new_o.o_id

	(* [get_extremes pts_list] recherche les points extremes d'un objet, met
	   a jour les valeurs pour la scene et reaffiche si necessaire *)
    method private get_extremes pts_list =
      if pts_list <> [] then begin
	let (p1, p2) =
	  if extreme_min=pt_undefined && extreme_max=pt_undefined then
	    (List.hd pts_list, List.hd pts_list)
	  else (extreme_min, extreme_max)
	in

	let minmax coord l =
	  let dep = (get_coord p1 coord, get_coord p2 coord) in
	  List.fold_left (fun (i, a) e0 ->
	    let e = get_coord e0 coord in (min e i, max e a)) dep l
	in

	let (minx, maxx) = minmax X_AXIS pts_list
	and (miny, maxy) = minmax Y_AXIS pts_list
	and (minz, maxz) = minmax Z_AXIS pts_list in
	let cc i a = (a +. i) /. 2. in
	let r = max (maxx -. minx) (maxy -. miny) in
	rs <- max r (maxz -. minz) ;
	extreme_min <- (minx, miny, minz) ;
	extreme_max <- (maxx, maxy, maxz) ;
	let new_extents = self#scale_point (-.(cc minx maxx), -.(cc miny maxy),
					    -.(cc minz maxz)) in
	if new_extents<>extents then begin
	  extents<-new_extents ; self#setup
	end
      end

	  (* Volume simple du type secteur ou seul le contour et les niveaux *)
	  (* inf et sup suffisent *)
    method add_object_volume_simple contour zmin zmax color filled =
      (* Le contour doit etre oriente dans le sens contre-horaire (cull_face) *)
      let contour =
	if poly_test_ccw contour = CW then contour else List.rev contour in
      let l0 = geom_close_poly ((List.map (fun p -> [glpoint3d_of_pt_2d zmin p;
						     glpoint3d_of_pt_2d zmax p])
				) contour) in

      (* Triangulation ou pas des faces superieures et inferieures *)
      let (down, up) =
	if poly_test_convex contour = CONVEX then
	  (NO_TRI (List.map (fun l -> List.hd l) l0),
	   NO_TRI (List.rev_map (fun l -> List.hd (List.tl l)) l0))
	else begin
	  if use_fans_for_tesselation then begin
	    let fans = Geometry_2d.tesselation_fans contour in
	    let l1 = List.map (List.map (glpoint3d_of_pt_2d zmin)) fans
	    and l2 = List.map (fun lst ->
	      let l = List.map (glpoint3d_of_pt_2d zmax) lst in
	      (* Le premier point ne doit pas se trouver en dernier *)
	      (* car c'est le pivot du triangle_fan *)
	      (List.hd l)::(List.rev (List.tl l))) fans in
	    (TRI_WITH_FANS l1, TRI_WITH_FANS l2)
	  end else begin
	    let triangles = List.flatten (Geometry_2d.tesselation contour) in
	    let l1 = List.map (glpoint3d_of_pt_2d zmin) triangles
	    and l2 = List.rev_map (glpoint3d_of_pt_2d zmax) triangles in
	    (TRI_STD l1, TRI_STD l2)
	  end
	end
      in
      self#add_object (VOLUME1_3D {vol3d_contour = List.flatten l0;
				   vol3d_up      = up;
				   vol3d_down    = down;
				   vol3d_color   = color;
				   vol3d_filled  = filled})
	(* Enveloppe 3D : faces laterales liant un contour_haut et un contour_bas *)
	(* 2 contours = meme nombre de points, oriente dans le sens contre-horaire *)
    method add_object_enveloppe contour_haut contour_bas color filled =
      let add_p p_bas p_haut res =
	(glpoint3d_of_pt_3d p_bas)::(glpoint3d_of_pt_3d p_haut)::res
      in
      let cw_poly c =
	let contour_2d = List.map (pt_3d_to_pt_2d T_Z3D) c in
	if (poly_test_ccw contour_2d = CW) then c else List.rev c
      in
      let l_bas =  geom_close_poly (cw_poly contour_bas) in
      let l_haut = geom_close_poly (cw_poly  contour_haut) in
      let l0 =
	try List.fold_right2 add_p l_bas l_haut []
	with x-> Printf.printf "\nadd_object_enveloppe.fold... : "; raise x
      in
      try
	self#add_object (ENVELOPPE_3D {env3d_contour = l0;
				       env3d_color   = color;
				       env3d_filled  = filled})
      with x ->
	Printf.printf "\nadd_object_enveloppe. self#add_object "; raise x

	  (* Enveloppe 3D : faces laterales liant un contour_haut et un contour_bas *)
	  (* 2 contours = meme nombre de points, oriente dans le sens contre-horaire *)
    method add_object_enveloppe_double contour_haut contour_bas
	color_out color_in filled =
      let add_p p_bas p_haut res =
	(glpoint3d_of_pt_3d p_bas)::(glpoint3d_of_pt_3d p_haut)::res
      in
      let cw_poly c =
	let contour_2d = List.map (pt_3d_to_pt_2d T_Z3D) c in
	if (poly_test_ccw contour_2d = CW) then c else List.rev c
      in
      let l_bas =  geom_close_poly (cw_poly contour_bas) in
      let l_haut = geom_close_poly (cw_poly  contour_haut) in
      let l0 =
	try List.fold_right2 add_p l_bas l_haut []
	with x-> Printf.printf "\nadd_object_enveloppe.fold... : "; raise x
      in
      let l_inside =
	try List.fold_right2 add_p (List.rev l_bas) (List.rev l_haut) []
	with x-> Printf.printf "\nadd_object_enveloppe.foldinside... : "; raise x
      in
      try
	self#add_object (ENVELOPPE_3D_DOUBLE {env3d_double_contour_out = l0;
					      env3d_double_contour_in  = l_inside;
					      env3d_double_color_out   = color_out;
					      env3d_double_color_in    = color_in;
					      env3d_double_filled      = filled})
      with x ->
	Printf.printf "\nadd_object_enveloppe. self#add_object "; raise x

	  (* Fleche (flux) dont la pointe est placee en pt0 si sens est vrai *)
	  (* Ici, inutile de tesseler les faces inf et sup *)
    method add_object_arrow pt0 vdir sens ep lg color filled
	arrow_type =
      let pts =
	if arrow_type=ARROW1 then
	  [(0., 0.); (2.*.ep, ep); (2.*.ep, ep/.2.); (lg, ep/.2.);
	   (lg, (-.ep)/.2.); (2.*.ep, (-.ep)/.2.); (2.*.ep, -.ep)]
	else
	  [(0., 0.); (2.5*.ep, ep); (2.*.ep, ep/.2.); (lg, ep/.2.);
	   (lg, (-.ep)/.2.); (2.*.ep, (-.ep)/.2.); (2.5*.ep, -.ep)]
      in

      let pts =	if sens then pts else List.map (fun (x, y) -> (x-.lg, y)) pts in

      let pts = List.map (fun (x, y) -> {x2D=x; y2D=y}) pts in

      let pts = if poly_test_ccw pts = CW then pts else List.rev pts in
      let l = List.map (fun p ->
	[glpoint3d_of_pt_2d ((-.ep)/.2.) p; glpoint3d_of_pt_2d (ep/.2.) p]) pts in
      let dd =
	if sens then Geometry_3d.vect_make pt0 vdir
	else Geometry_3d.vect_make vdir pt0
      in
      let h = Geometry_3d.vect_norm dd
      and d = sqrt(dd.x3D*.dd.x3D+.dd.y3D*.dd.y3D) in
      let alpha =
	if dd.y3D>0. then acos (dd.x3D/.d) else	-. (acos (dd.x3D/.d))
      in
      let beta =
	if dd.z3D>0. then acos (d/.h) else	-.(acos (d/.h))
      in

      self#add_object (ARROW_3D {arr3d_contour  = geom_close_poly l;
				 arr3d_pt       = glpoint3d_of_pt_3d pt0 ;
				 arr3d_vect     = (dd.y3D, -.dd.x3D, 0.) ;
				 arr3d_angle_xy = rad2deg beta ;
				 arr3d_angle_z  = rad2deg alpha ;
				 arr3d_color    = color;
				 arr3d_filled   = filled})

    method add_object_outline contour cin cout filled =
      let l = geom_close_poly (glpoint3d_of_pt_3d_lst contour) in
      self#add_object (OUTLINE_3D {out3d_contour   = l ;
				   out3d_color_in  = cin ;
				   out3d_color_out = cout ;
				   out3d_filled    = filled})

    method add_object_line l color line_width with_bars fill =
      self#add_object (LINE_3D {line3d_points    = glpoint3d_of_pt_3d_lst l;
				line3d_width     = line_width;
				line3d_color     = color;
				line3d_with_bars = with_bars;
				line3d_filled    = fill})

    method add_object_point pos pos2 name color with_name =
      self#add_object (POINT_3D {p3d_pos       = glpoint3d_of_pt_3d pos;
				 p3d_pos2      = glpoint3d_of_pt_3d pos2;
				 p3d_name      = name;
				 p3d_with_name = with_name;
				 p3d_color     = color})

    method add_object_surface_with_texture tab texture_id =
      let t = Array.map (Array.map glpoint3d_of_pt_3d) tab in
      self#add_object (SURFACE_3D_TEX {s3d_tex_pts = t;
				       s3d_tex_texture_id = texture_id})

    method add_object_surface tab fill =
      let t = Array.map (Array.map (fun (p, c) -> (glpoint3d_of_pt_3d p, c))) tab in
      self#add_object (SURFACE_3D {s3d_pts = t; s3d_filled = fill})

	(* cree l'objet rosace *)
    method private create_rosace =
      self#make_current ;
      let compiled = GlList.create `compile in
      GlDraw.line_width 2.0 ;
      GlDraw.color color_rosace ;
      GlDraw.begins `line_strip ;
      List.iter GlDraw.vertex2 [(0.0, -0.05); (0.0, 0.05)] ;
      GlDraw.ends () ;
      GlDraw.begins `line_strip ;
      List.iter GlDraw.vertex2 [(-0.05, 0.0); (0.05, 0.0)] ;
      GlDraw.ends () ;
      GlDraw.begins `line_strip ;
      List.iter GlDraw.vertex2 [(-0.01, 0.04); (0.0, 0.05); (0.01, 0.04)] ;
      GlDraw.ends () ;
      if fonts_available then begin
	(* Affichage du Nord sur la rosace si la fonte est disponible *)
	GlPix.raster_pos ~x:(-.0.01) ~y:0.07 ~z:0.0 () ;
	Gtkgl_Hack.gl_print_string fontbase "N"
      end ;
      GlList.ends () ;
      rosace <- Some compiled ;
      compiled

	(* initialisation lors de la creation du widget *)
    method private init_func () =
      if not done_init then begin
	do_msg "Init 3D" ;
	List.iter Gl.enable [`depth_test; `cull_face] ;
	GlDraw.cull_face `back;
	GlDraw.front_face `ccw;

	List.iter (if use_lights then Gl.enable else Gl.disable) lights ;
	GlDraw.shade_model (if use_smooth then `smooth else `flat) ;

	if fonts_available then fontbase <- load_bitmap_font "8x13" ;

	GlPix.store (`unpack_alignment 1);
	List.iter (GlTex.parameter ~target:`texture_2d)
	  [ `wrap_s `repeat; `wrap_t `repeat; `mag_filter `linear; `min_filter `linear ];
	GlTex.env (`mode `decal);

	done_init <- true
      end

    method private reshape_func ~width:w ~height:h =
      if not done_init then self#init_func () ;
      do_msg (Printf.sprintf "Reshape 3D w=%d h=%d" w h) ;
      width <- w; height <- h ;
      GlDraw.viewport ~x:0 ~y:0 ~w ~h;
      self#setup

	(* met a jour de la vue et de la source de lumiere *)
    method private setup =
      do_msg "Setup" ;
      GlMat.mode `projection;
      GlMat.load_identity ();
      let aspect = float width /. float height and view_fovs = 1. in
      GluMat.perspective ~fovy:(45. *. view_fovs) ~aspect
	~z:(0.1, (rs*.sqrt(2.)+.1.));

      GlMat.mode `modelview;
      GlMat.load_identity ();

      (* Deplace et tourne la position de l'utilisateur *)
      GlMat.translate3 depl ;
      rotate_all rot ;

      (* Place les objets au centre de la scene *)
      GlMat.translate3 extents ;

      (* Positionnement de la lumiere *)
      if use_lights then begin
	GlMat.push ();
	rotate_all lightrot ;
	let (x, y, z)= lightpos in
	List.iter (GlLight.light ~num:0) [`position (x, y, z, 1.);
					  `ambient  (1., 1., 1., 1.);
					  `diffuse  (1., 1., 1., 1.)] ;
	GlMat.pop ()
      end ;

      self#set_status

    method set_name n        = name <- n; self#display_func
	method set_show_name set = show_name <- set; self#display_func

    method display = fun o ->
      GlList.call (self#get_object o).o_compiled; Gl.flush (); area#swap_buffers ()
    (* force l'affichage (apres ajout d'un objet par exemple) *)
    method display_func =
      do_msg "Display 3D" ;
      self#make_current ;
      (* Efface l'ecran *)
      GlClear.color back_color ~alpha:0.0; GlClear.clear [`color;`depth];
      (* Passage des objets *)
      List.iter (fun o -> if o.o_show then GlList.call o.o_compiled) objects ;

      let aspect = float width /. float height in

      (* Affichage du nom si c'est demande *)
      if fonts_available && name<>"" && show_name then begin
	GlMat.push ();
	GlMat.load_identity ();
	set_color (1., 1., 1.) ;
	GlPix.raster_pos ~x:(-.4.*.aspect) ~y:(4.) ~z:(-.10.0) () ;
	Gtkgl_Hack.gl_print_string fontbase name ;
	GlMat.pop ()
      end ;

      (* Affichage de la rosace si necessaire *)
      if show_rosace then begin
	(* Creation de la rosace si ca n'est pas deja fait... *)
	let r = match rosace with None -> self#create_rosace|Some r -> r in
	GlMat.push ();
	GlMat.load_identity ();
	GlMat.translate3 (-.0.5*.aspect, 0.5, -.1.5) ;
	rotate_some rot [X_AXIS; Z_AXIS] ;
	GlList.call r ;
	GlMat.pop ()
      end ;

      (* Affichage de la sphere representant la lumiere *)
      if show_light then begin
	GlMat.push ();
	rotate_all lightrot ;
	GlMat.translate3 lightpos;
	set_color glcolor_white ;
	let radius = 0.03 in
	GluQuadric.sphere ~radius ~stacks:5 ~slices:5 ();
	GlMat.pop ()
      end ;

      (* Affichage *)
      Gl.flush (); area#swap_buffers ()


	(* [mouse_press ev] traite un evenement correspondant a l'appui sur un bouton *)
    method private mouse_press ev =
      let mouse_pos = Gtk_tools.get_mouse_pos_click ev in
      match (Gtk_tools.test_mouse_but ev) with
	Gtk_tools.B_GAUCHE ->
	  self#set_cursor cursor_rotate ;
	  current_action <- (ACTION_ROTATE mouse_pos); true
      | Gtk_tools.B_MILIEU ->
	  current_action <- (ACTION_ZOOM mouse_pos); true
      | Gtk_tools.B_DROIT  -> false
      | _ -> false

	    (* [mouse_move ev] traite les mouvements souris *)
    method private mouse_move ev =
      area#misc#grab_focus () ;
      let mouse_pos = Gtk_tools.get_mouse_pos_move ev in
      match current_action with
	ACTION_NONE -> false
      | ACTION_ZOOM old_pos ->
	  let dy = (snd mouse_pos)-(snd old_pos) in
	  self#set_cursor (if dy<0 then cursor_zoom_up else cursor_zoom_down) ;
	  self#incr_dist (dist_incr*.(float_of_int dy)) ;
	  current_action <- (ACTION_ZOOM mouse_pos) ; true
      | ACTION_ROTATE old_pos ->
	  let dz = (fst mouse_pos)-(fst old_pos)
	  and dx = (snd mouse_pos)-(snd old_pos) in
	  let r = add_coord_360 rot X_AXIS (float_of_int dx) in
	  self#rotate_view (add_coord_360 r Z_AXIS (float_of_int dz)) ;
	  current_action <- (ACTION_ROTATE mouse_pos) ; true

	    (* [mouse_release ev] traite un evenement de relachement de bouton *)
    method private mouse_release _ev =
      (match current_action with ACTION_NONE -> () | _ -> self#reset_cursor) ;
      current_action <- ACTION_NONE ;
      true

	(* Mouvement de la molette de la souris sous Windows *)
    method private mouse_wheel ev =
      match GdkEvent.Scroll.direction ev with
	`UP    -> self#incr_dist (-.dist_incr) ; true
      | `DOWN  -> self#incr_dist dist_incr ; true
      | `LEFT  -> false
      | `RIGHT -> false

    method private start_stop_animation =
      match animation_timer with
	None ->
	  let timeout _ = self#incr_rotz rot_anim; true in
	  animation_timer <- Some (Timeout.add ~ms:tps_anim ~callback:timeout)
      | Some t -> Timeout.remove t ; animation_timer <- None

	    (* [key_pressed key] teste la touche pressee dans la zone de dessin
	       et effectue l'action associee le cas echeant *)
    method private key_pressed key =
      let keys_list =
	[([_Page_Up],   fun () -> self#incr_dist (-.dist_incr)) ;
	 ([_Page_Down], fun () -> self#incr_dist dist_incr) ;
	 ([_KP_Down; _KP_2],  fun () -> self#incr_rotx rot_incr) ;
	 ([_KP_Up; _KP_8],    fun () -> self#incr_rotx (-.rot_incr)) ;
	 ([_KP_Left; _KP_4],  fun () -> self#incr_rotz (-.rot_incr)) ;
	 ([_KP_Right; _KP_6], fun () -> self#incr_rotz rot_incr) ;
	 ([_Down],      fun () -> self#move_y (dist_incr/.3.)) ;
	 ([_Up],        fun () -> self#move_y ((-.dist_incr)/.3.)) ;
	 ([_Left],      fun () -> self#move_x (dist_incr/.3.)) ;
	 ([_Right],     fun () -> self#move_x ((-.dist_incr)/.3.)) ;
	 ([_l],         fun () -> self#lights_switch) ;
	 ([_s],         fun () -> self#smooth_switch) ;
	 ([_r],         fun () -> self#rosace_switch) ;
	 ([_Home],      fun () -> self#rotate_view pt_null) ;
	 ([_a],         fun () -> self#incr_light_rot rot_incr) ;
	 ([_z],         fun () -> self#incr_light_rot (-.rot_incr)) ;
	 ([_L],         fun () -> show_light <- not show_light; self#change_and_redraw) ;
	 ([_space],     fun () -> self#start_stop_animation);
	 ([_Escape],    fun () -> self#redo_all);
	 ([_i], fun () -> Printf.printf "%s" (get_gl_infos ()); flush stdout);
	 ([_n],         fun () -> self#set_show_name (not show_name))] in
      (* Recherche la fonction associee a la touche presse s'il y en a une *)
      let rec check_keys lst =
	match lst with
	  (keys, func)::reste ->
	    if List.mem key keys then begin func ();true end else check_keys reste
	| [] -> false
      in
      check_keys keys_list

	(* Modification de la position de l'utilisateur et des angles de vue *)
    method incr_dist d = self#move_view   (add_coord depl Z_AXIS d)
    method incr_rotx d = self#rotate_view (add_coord_360 rot X_AXIS d)
    method incr_rotz d = self#rotate_view (add_coord_360 rot Z_AXIS d)

	method move_x d = self#move_view (add_coord depl X_AXIS d)
	method move_y d = self#move_view (add_coord depl Y_AXIS d)

    (* tourne la lumiere *)
	method incr_light_rot d = lightrot<-add_coord_360 lightrot Z_AXIS d;
	  self#change_and_redraw

    (* Manipulations generales des objets *)
	method private get_object id =
	  try List.find (fun o -> o.o_id=id) objects
	  with Not_found -> raise (NO_SUCH_3D_OBJECT id)
	method delete_object id =
	  let found = ref None in
	  let new_l = List.fold_left (fun l obj ->
		if obj.o_id=id then begin found:=Some obj; l end else obj::l) [] objects in
	  match !found with
		None -> raise (NO_SUCH_3D_OBJECT id)
	  | Some obj -> objects <- List.rev new_l; GlList.delete obj.o_compiled
	method object_set_color id new_color =
	  let o = self#get_object id in
	  set_object_color o.o_obj new_color; self#make_and_compile o
	method object_get_color id =
	  get_object_color (self#get_object id).o_obj
	method object_set_visibility id visible =
	  let o = self#get_object id in o.o_show <- visible; self#make_and_compile o
	method object_get_visibility id = (self#get_object id).o_show
	method object_set_fill id filled =
	  let o = self#get_object id in
	  set_object_fill o.o_obj filled; self#make_and_compile o
	method object_get_fill id =
	  get_object_fill (self#get_object id).o_obj

    (* Fonctions specifiques a certains objets *)
	method line_get_width id =
	  get_line_width (self#get_object id).o_obj id
	method line_set_width id width =
	  let o = self#get_object id in
	  set_line_width o.o_obj width id; self#make_and_compile o
	method line_get_with_bars id =
	  get_line_bars (self#get_object id).o_obj id
	method line_set_with_bars id withbars =
	  let o = self#get_object id in
	  set_line_bars o.o_obj withbars id; self#make_and_compile o
	method outline_set_in_color id new_color =
	  let o = self#get_object id in
	  set_outline_in_color o.o_obj new_color id; self#make_and_compile o
	method outline_get_in_color id =
	  get_outline_in_color (self#get_object id).o_obj id
	method point_get_with_name id =
	  get_point_name (self#get_object id).o_obj id
	method point_set_with_name id withname =
	  let o = self#get_object id in
	  set_point_name o.o_obj withname id; self#make_and_compile o

	method destroy_all_objects =
	  List.iter (fun o -> GlList.delete o.o_compiled) objects ;
	  objects <- [] ;
	  (match rosace with None -> () | Some r -> GlList.delete r) ;
	  unload_bitmap_font fontbase

	method private redo_all =
	  self#setup; self#recompile_all_objects;
	  (match rosace with None -> () | Some r -> GlList.delete r) ;
	  if show_rosace then ignore(self#create_rosace) ;
	  self#display_func

	method set_back_color c = back_color <- c ; self#display_func

    initializer
      ignore(area#connect#realize ~callback:self#init_func) ;
      ignore(area#connect#display ~callback:(fun () -> self#display_func)) ;
      ignore(area#connect#reshape ~callback:self#reshape_func) ;
      area#misc#realize ();

      incr nb_objects ;

      (* Indispensable pour que lorsque l'on entre dans la fenetre, les *)
      (* prochaines commandes OpenGL soient bien appliquees dans la vue *)
      (* correspondante dans le cas ou plusieurs vues ont ete ouvertes  *)
      ignore(area#event#connect#focus_in (fun _ -> self#make_current; false)) ;

      (* Callbacks des evenements souris *)
      Gtk_tools_GL.glarea_mouse_connect area
	self#mouse_press self#mouse_move self#mouse_release ;

      let scroll_cb = fun ev ->
	match GdkEvent.get_type ev with
	| `SCROLL -> self#mouse_wheel (GdkEvent.Scroll.cast ev)
	| _ -> false in

      (* Reactions aux mouvements de la molette souris *)
      ignore(area#event#connect#any ~callback:scroll_cb) ;

      (* Attachement des callbacks pour les evenements clavier *)
      Gtk_tools_GL.glarea_key_connect area self#key_pressed (fun _k -> (); false)
  end

(* =============================== FIN ========================================= *)
