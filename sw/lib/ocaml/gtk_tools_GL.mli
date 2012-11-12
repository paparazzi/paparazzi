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

(** Module de gestion des zones de dessin OpenGL

   {b D�pendences : Platform}

 *)


(** {6 Drawing Areas OpenGL} *)


(** [create_draw_glarea_base width height pack_method] cr�e une zone de
   dessin OpenGL de largeur [width] et de hauteur [height]. Cette zone est
   plac�e comme indiqu� dans [pack_method]. La zone cr��e est renvoy�e *)
val create_draw_glarea_base :
  int -> int -> (GObj.widget -> unit) -> GlGtk.area

(** [connect_draw_glarea_simple area init_func display_func reshape_func]
   connecte les signaux de base � une zone de dessin cr��e avec
   {!Gtk_tools_GL.create_draw_glarea_base}. La fonction de redessin
   est renvoy�e *)
val connect_draw_glarea_simple :
  GlGtk.area ->
  (unit -> unit) ->
  (unit -> 'a) -> (width:int -> height:int -> unit) -> unit -> unit

(** [create_draw_glarea_simple width height pack_method
   init_func display_func reshape_func] cr�e une zone de dessin OpenGL et y
   connecte les signaux. Un couple contenant la zone et la fonction de
   redessin est renvoy� *)
val create_draw_glarea_simple :
  int ->
  int ->
  (GObj.widget -> unit) ->
  (unit -> unit) ->
  (unit -> 'a) ->
  (width:int -> height:int -> unit) -> GlGtk.area * (unit -> unit)


(** {6 Signaux des Drawing Areas OpenGL} *)


(** [glarea_mouse_connect area mouse_press mouse_move mouse_release]
   connecte les �v�nements souris � la zone de dessin [area] *)
val glarea_mouse_connect :
  GlGtk.area ->
  (GdkEvent.Button.t -> bool) ->
  (GdkEvent.Motion.t -> bool) -> (GdkEvent.Button.t -> bool) -> unit

(** [glarea_key_connect area key_press key_release] connecte les
   �v�nements claviers � la zone de dessin [area] *)
val glarea_key_connect :
  GlGtk.area -> (Gdk.keysym -> bool) -> (Gdk.keysym -> bool) -> unit


(** {6 Couleurs Gtk <-> OpenGL} *)


(** [gtk_to_gl_color color] cr�e une couleur OpenGL (r, g, b) � partir
   de [color]. Les composantes RGB sont dans l'intervalle [\[0.0, 1.0\]] *)
val gtk_to_gl_color : GDraw.color -> float * float * float

(** [gl_to_gtk_color (r, g, b)] fonction inverse de la precedente *)
val gl_to_gtk_color :
  float * float * float -> [> `RGB of int * int * int]
