(*
 * Images utils
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

(** Module de capture d'écran/gestion d'images

   {b Dépendences : Platform}
 *)

open GDraw

(** Formats de sauvegarde pris en compte *)
type format_capture = PNG | GIF | JPEG | TIFF | BMP | PPM | POSTSCRIPT

(** Formats disponibles suivant qu'on est sous Unix/Windows *)
val formats_capture_dispos : format_capture list

(** [is_format_capture_dispo format] indique si le format est disponible *)
val is_format_capture_dispo : format_capture -> bool

(** Etats lors de la sauvegarde *)
type progress_save = INIT | SAVING | FINISHED

(** [extended_string_of_format_capture format] fournit une chaine correspondant
   au format : [PNG] -> "PNG"; [POSTSCRIPT] -> "Postscript" *)
val extended_string_of_format_capture : format_capture -> string

(** [string_of_format_capture format] fournit une chaine correspondant
   au format : [PNG] -> "PNG"; [POSTSCRIPT] -> "PS" *)
val string_of_format_capture : format_capture -> string

(** [format_capture_of_string chaine] renvoie le type {!Gtk_image.format_capture}
   correspondant à la chaine *)
val format_capture_of_string : string -> format_capture

(** [string_of_extension format] renvoie l'extension correspondant au format.
   i.e [PNG] -> ".png" *)
val string_of_extension : format_capture -> string

(** [set_filename_extension fichier format] ajoute l'extension correspondant au
   format à la chaine [fichier] si nécessaire *)
val set_filename_extension : string -> format_capture -> string

(** [update_extension_capture fichier ancien_format nouveau_format] modifie
   l'extension du fichier pour qu'elle corresponde à [nouveau_format] *)
val update_extension_capture :
  string -> format_capture -> format_capture -> string

(** [capture_part pixmap_ou_fenetre x y largeur hauteur fichier format progress_func]
   effectue la capture partielle à partir de [pixmap_ou_fenetre]. [x] et [y]
   designent le coin en haut à gauche et [largeur] et [hauteur] la taille de
   l'image à capturer *)
val capture_part :
  [> `drawable ] Gobject.obj -> int -> int -> int -> int ->
  string -> format_capture -> (progress_save -> float -> unit) option -> unit

(** [capture_part_with_caption fenetre pixmap_ou_fenetre x y largeur hauteur
   fichier format progress_func legende]
   effectue la capture partielle à partir de [pixmap_ou_fenetre]. [x] et [y]
   designent le coin en haut à gauche et [largeur] et [hauteur] la taille de
   l'image à capturer. Une legende est ajoutée sur la capture si
   [legende=Some (texte_legende, couleur_texte, couleur_contour, couleur_fond,
   fonte)].
   [fenetre] designe ici la fenetre mère du drawable [pixmap_ou_fenetre]
 *)
val capture_part_with_caption :
  < misc : < visual_depth : int; window : Gdk.window; .. >; .. > ->
  Gdk.pixmap ->
  int ->
  int ->
  int ->
  int ->
  string ->
  format_capture ->
  (progress_save -> float -> unit) option ->
  (string * GDraw.color * GDraw.color * GDraw.color * Gdk.font) option ->
  unit

(** [capture_complete fenetre pixmap_ou_fenetre largeur hauteur
   fichier format progress_func legende_optionnelle] sauvegarde tout le contenu
   de [pixmap_ou_fenetre] *)
val capture_complete :
  < misc : < visual_depth : int; window : Gdk.window; .. >; .. > ->
  Gdk.pixmap ->
  int ->
  int ->
  string ->
  format_capture ->
  (progress_save -> float -> unit) option ->
  (string * GDraw.color * GDraw.color * GDraw.color * Gdk.font) option ->
  unit

(** [gtk_image_load filename window format] charge une image au format [format]
   et renvoie une pixmap contenant l'image lue *)
val gtk_image_load : string -> GWindow.window -> format_capture -> GDraw.pixmap
