(*
 * OpenGL utils
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
(* = YLF 28/10/2001                                                                 = *)
(* =                                                                                = *)
(* = Derniere update : 28/10/2002                                                   = *)
(* =                                                                                = *)
(* =                                                                                = *)
(* = 28/10/2002 : create_draw_glarea_base et                              = *)
(* =              et connect_draw_glarea_simple                           = *)
(* = 15/05/2002 : gl_to_gtk_color et gtk_to_gl_color            = *)
(* = 12/04/2002 : create_draw_glarea_simple                               = *)
(* =                                                                                = *)
(* ================================================================================== *)

open Gtk_tools

(* ============================================================================= *)
(* = Creation d'une drawing area OpenGL                                        = *)
(* =                                                                           = *)
(* = width        = hauteur de la zone                                         = *)
(* = height       = hauteur de la zone                                         = *)
(* = pack_method  = maniere de placer la zone (ex. : hbox#pack)                = *)
(* ============================================================================= *)
let create_draw_glarea_base width height pack_method =
  (* Creation des widgets *)
  GlGtk.area [`DEPTH_SIZE 1; `RGBA; `DOUBLEBUFFER]
    ~width:width ~height:height ~packing:pack_method ()

(* ============================================================================= *)
(* = Connection des fonctions de base a une drawing area OpenGL                = *)
(* =                                                                           = *)
(* = area         = la zone                                                    = *)
(* = init_func    = fonction d'initialisation (realize)                        = *)
(* = display_func = fonction de dessin dans la zone                            = *)
(* = reshape_func = appelee lors d'un changement de taille                     = *)
(* ============================================================================= *)
let connect_draw_glarea_simple area
    init_func display_func reshape_func =
  (* La nouvelle fonction de dessin appelle celle qui est passee en parametre quand *)
  (* necessaire et fait le flush ensuite *)
  let draw = (fun () -> if (area:GlGtk.area)#misc#visible then begin
    display_func (); Gl.flush (); area#swap_buffers ()
  end) in

  (* Connection des fonctions *)
  ignore(area#connect#realize ~callback:init_func) ;
  ignore(area#connect#display ~callback:draw) ;
  ignore(area#connect#reshape ~callback:reshape_func) ;

  (* Renvoie la nouvelle fonction de dessin *)
  draw

(* ============================================================================= *)
(* = Creation d'une drawing area OpenGL                                        = *)
(* =                                                                           = *)
(* = width        = hauteur de la zone                                         = *)
(* = height       = hauteur de la zone                                         = *)
(* = pack_method  = maniere de placer la zone (ex. : hbox#pack)                = *)
(* = init_func    = fonction d'initialisation (realize)                        = *)
(* = display_func = fonction de dessin dans la zone                            = *)
(* = reshape_func = appelee lors d'un changement de taille                     = *)
(* ============================================================================= *)
let create_draw_glarea_simple width height pack_method
    init_func display_func reshape_func =
  (* Creation des widgets *)
  let area = create_draw_glarea_base width height pack_method in

  let draw = connect_draw_glarea_simple area
    init_func display_func reshape_func in

  (* Renvoie la zone de dessin et la nouvelle fonction de dessin *)
  (area, draw)

(* ============================================================================= *)
(* = Connexion evenements souris a une zone de dessin OpenGL                   = *)
(* =                                                                           = *)
(* = area          = la zone de dessin                                         = *)
(* = mouse_press   = fonction appelee lors d'un click souris                   = *)
(* = mouse_move    = fonction appelee lors d'un deplacement                    = *)
(* = mouse_release = fonction appelee lors du relachement d'un bouton          = *)
(* ============================================================================= *)
let glarea_mouse_connect area mouse_press mouse_move mouse_release =
  (area:GlGtk.area)#event#add [`POINTER_MOTION; `BUTTON_PRESS; `BUTTON_RELEASE] ;
  area#event#set_extensions `ALL;
  ignore(area#event#connect#button_press   ~callback:mouse_press) ;
  ignore(area#event#connect#motion_notify  ~callback:mouse_move) ;
  ignore(area#event#connect#button_release ~callback:mouse_release)

(* ============================================================================= *)
(* = Connexion evenements clavier a une zone de dessin                         = *)
(* =                                                                           = *)
(* = area        = la zone de dessin                                           = *)
(* = key_press   = fonction appelee lors de l'appui sur une touche             = *)
(* = key_release = fonction appelee lors du relachement d'une touche           = *)
(* ============================================================================= *)
let glarea_key_connect area key_press key_release =
  (* Par defaut l'evenement key_release n'est pas associe au widget *)
  (area:GlGtk.area)#event#add [`KEY_RELEASE] ;
  ignore(area#event#connect#key_press
           ~callback:(fun ev -> key_press   (GdkEvent.Key.keyval ev))) ;
  ignore(area#event#connect#key_release
           ~callback:(fun ev -> key_release (GdkEvent.Key.keyval ev))) ;
  area#misc#set_can_focus true ;
  area#misc#grab_focus ()

(* ============================================================================= *)
(* = Passage de couleur GTK vers GL                                            = *)
(* =                                                                           = *)
(* = color = couleur GTK (`NAME ou `RGB) a transformer                         = *)
(* ============================================================================= *)
let gtk_to_gl_color color =
  let t = GDraw.color color in
  ((float_of_int (Gdk.Color.red t))/.65535.0,
   (float_of_int (Gdk.Color.green t))/.65535.0,
   (float_of_int (Gdk.Color.blue t))/.65535.0)

(* ============================================================================= *)
(* = Passage de couleur GL vers GTK                                            = *)
(* =                                                                           = *)
(* = (r, g, b) = couleur GL a transformer en equivalent GTK                    = *)
(* ============================================================================= *)
let gl_to_gtk_color (r, g, b) =
  `RGB(int_of_float (r*.65535.0), int_of_float (g*.65535.0),
       int_of_float (b*.65535.0))

(* =============================== FIN ========================================= *)
