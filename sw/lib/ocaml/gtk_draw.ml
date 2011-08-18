(*
 * $Id$
 *
 * GTK drawing in a pixmap
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


(* ============================================================================= *)
(* = Fonction de casting, indispensable pour pouvoir utiliser n'importe quel   = *)
(* = type de drawable ('window, 'pixmap) pour dessiner                         = *)
(* ============================================================================= *)
let gd_do_cast p = (p:GDraw.drawable)

(* ============================================================================= *)
(* = GDraw.color -> (r, g, b) en entiers                                       = *)
(* ============================================================================= *)
let gd_rgb_of_color color =
  let t = GDraw.color color in
  (Gdk.Color.red t, Gdk.Color.green t, Gdk.Color.blue t)

(* ============================================================================= *)
(* = GDraw.color -> (r, g, b) en flottants                                     = *)
(* ============================================================================= *)
let gd_float_rgb_of_color color =
  let (r, g, b) = gd_rgb_of_color color in
  (float_of_int r, float_of_int g, float_of_int b)

(* ============================================================================= *)
(* = Creation d'une GDraw.color a partir de ces composantes (r, g, b)          = *)
(* ============================================================================= *)
let gd_color_of_rgb (r, g, b) = `RGB (r, g, b)

(* ============================================================================= *)
(* = Creation d'une GDraw.color a partir de ces composantes (r, g, b)          = *)
(* ============================================================================= *)
let gd_color_of_float_rgb (r, g, b) = `RGB (int_of_float r, int_of_float g,
											int_of_float b)
(* ============================================================================= *)
(* = Mise a jour de la couleur de dessin                                       = *)
(* ============================================================================= *)
let gd_set_color p color =
  (gd_do_cast p)#set_foreground color

(* ============================================================================= *)
(* = Mise a jour de la couleur de dessin a n% de la valeur indiquee            = *)
(* ============================================================================= *)
let gd_set_faded_color p color pct =
  let pct = (float_of_int pct)/.100. and (r, g, b) = gd_float_rgb_of_color color in
  (gd_do_cast p)#set_foreground (gd_color_of_float_rgb (r*.pct, g*.pct, b*.pct))

(* ============================================================================= *)
(* = Modification du style de trace                                            = *)
(* ============================================================================= *)
(* On est oblige de preciser width, alors qu'il ne doit normalement pas servir *)
(* Si on ne le met pas, les pointilles n'apparaissent pas partout !!!          *)
let gd_set_style_solid p =
  (gd_do_cast p)#set_line_attributes ~style:`SOLID ~width:1 ()
let gd_set_style_dash p =
  (gd_do_cast p)#set_line_attributes ~style:`ON_OFF_DASH ~width:1 ()
let gd_set_style_double_dash p =
  (gd_do_cast p)#set_line_attributes ~style:`DOUBLE_DASH ~width:1 ()

(* ============================================================================= *)
(* = Modification du mode de trace                                             = *)
(* ============================================================================= *)
(*let gd_set_mode_xor p = (gd_do_cast p)#set_gc_xor
let gd_set_mode_std p = (gd_do_cast p)#set_gc_copy*)

(* ============================================================================= *)
(* = Modification de l'epaisseur du trace                                      = *)
(* ============================================================================= *)
let gd_set_line_width p width = (gd_do_cast p)#set_line_attributes ~width:width ()

(* ============================================================================= *)
(* = Centrage d'un texte a afficher                                            = *)
(* ============================================================================= *)
let gd_center_text (x, y) string font =
  let (w, h) = Gtk_tools.string_width_height font string in
  (x-w/2, y+h/2)

(* ============================================================================= *)
(* = Dessin d'un texte non centre                                              = *)
(* ============================================================================= *)
let gd_draw_non_centered_text p (x, y) text font =
  (gd_do_cast p)#string text ~font:font ~x:x ~y:y

(* ============================================================================= *)
(* = Dessin d'un texte centre                                                  = *)
(* ============================================================================= *)
let gd_draw_text p pos_text text font =
  let (x, y) = gd_center_text pos_text text font in
  (gd_do_cast p)#string text ~font:font ~x:x ~y:y

(* ============================================================================= *)
(* = Dessin d'un segment                                                       = *)
(* ============================================================================= *)
let gd_draw_segment p (x1, y1) (x2, y2) =
  (gd_do_cast p)#line ~x:x1 ~y:y1 ~x:x2 ~y:y2

(* ============================================================================= *)
(* = Dessin d'un pixel                                                         = *)
(* ============================================================================= *)
let gd_draw_point p (x, y) = (gd_do_cast p)#point ~x:x ~y:y

(* ============================================================================= *)
(* = Dessin d'une ligne                                                        = *)
(* ============================================================================= *)
let gd_draw_line p pts = (gd_do_cast p)#lines pts

(* ============================================================================= *)
(* = Dessin d'un polygone                                                      = *)
(* ============================================================================= *)
let gd_draw_polygon p pts = (gd_do_cast p)#polygon ~filled:false pts

(* ============================================================================= *)
(* = Dessin d'un polygone plein                                                = *)
(* ============================================================================= *)
let gd_draw_filled_polygon p pts = (gd_do_cast p)#polygon ~filled:true pts

(* ============================================================================= *)
(* = Dessin d'un cercle                                                        = *)
(* ============================================================================= *)
let gd_draw_circle p (x, y) r =
  (gd_do_cast p)#arc ~filled:false ~x:(x-r) ~y:(y-r) ~width:(2*r) ~height:(2*r) ()

(* ============================================================================= *)
(* = Dessin d'un cercle plein                                                  = *)
(* ============================================================================= *)
let gd_draw_filled_circle p (x, y) r =
  (gd_do_cast p)#arc ~filled:true ~x:(x-r) ~y:(y-r) ~width:(2*r) ~height:(2*r) ()

(* ============================================================================= *)
(* = Dessin d'un rectangle                                                     = *)
(* ============================================================================= *)
let gd_draw_rect p (x1, y1, x2, y2) =
  (gd_do_cast p)#rectangle ~filled:false ~x:x1 ~y:y1
	~width:(x2-x1) ~height:(y2-y1) ()

(* ============================================================================= *)
(* = Dessin d'un rectangle plein                                               = *)
(* ============================================================================= *)
let gd_draw_filled_rect p (x1, y1, x2, y2) =
  (gd_do_cast p)#rectangle ~filled:true ~x:x1 ~y:y1	~width:(x2-x1) ~height:(y2-y1) ()

(* ============================================================================= *)
(* = Dessin d'un triangle                                                      = *)
(* ============================================================================= *)
let gd_draw_triangle p (x, y) size =
  let size0 = int_of_float ((float_of_int size) *. 1.5) and
	  size1 = int_of_float ((float_of_int size) *. 0.5) in
  (gd_do_cast p)#polygon ~filled:false
	[(x, y-size); (x-size0, y+size1); (x+size0, y+size1)]

(* ============================================================================= *)
(* = Dessin d'un triangle plein                                                = *)
(* ============================================================================= *)
let gd_draw_filled_triangle p (x, y) size =
  let size0 = int_of_float ((float_of_int size) *. 1.5) and
	  size1 = int_of_float ((float_of_int size) *. 0.5) in
  (gd_do_cast p)#polygon ~filled:true
	[(x, y-size); (x-size0, y+size1); (x+size0, y+size1)]

(* ============================================================================= *)
(* = Efface une pixmap                                                         = *)
(* ============================================================================= *)
let gd_clear p color width height =
  gd_set_color p color ;
  gd_draw_filled_rect (gd_do_cast p) (0, 0, width, height)

(* ============================================================================= *)
(* = Place le fond statique dans la pixmap de dessin                           = *)
(* ============================================================================= *)
let gd_set_background_pixmap p dest = (gd_do_cast dest)#put_pixmap ~x:0 ~y:0 p

(* ============================================================================= *)
(* = Place une pixmap transparente dans le dessin                              = *)
(* ============================================================================= *)
let gd_put_transp_pixmap p dest x y =
  (* Indispensable d'utiliser le masque pour la transparence *)
  (match p#mask with
	None -> () |
	Some m -> (gd_do_cast dest)#set_clip_origin ~x:x ~y:y; dest#set_clip_mask m) ;

  (* Mise en place du pixmap transparent *)
  dest#put_pixmap ~x:x ~y:y p#pixmap ;

  (* On enleve le masque *)
  (match p#mask with None -> () | Some _m -> prerr_endline "TODO (Gtk_draw): dest#unset_clip_mask")

