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

(* Modules CamlImages *)
open Images
open Png
open Gif
open Jpeg
open Xpm
open Tiff

open Platform

(* Formats de sauvegarde pris en compte *)
type format_capture = PNG | GIF | JPEG | TIFF | BMP | PPM | POSTSCRIPT

(* Formats disponibles suivant qu'on est sous Unix/Windows *)
let formats_capture_dispos =
  if platform_is_unix then [PNG; GIF; JPEG; TIFF; BMP; PPM; POSTSCRIPT]
  else [BMP; TIFF; PPM] (* Windows *)

(* Indique si le format typ est disponible *)
let is_format_capture_dispo typ =
  try ignore(List.find (fun t -> t=typ) formats_capture_dispos) ; true
  with Not_found -> false

(* Etats lors de la sauvegarde *)
type progress_save = INIT | SAVING | FINISHED

(* [extended_string_of_format_capture format] fournit une chaine correspondant
   au format : [PNG] -> "PNG"; [POSTSCRIPT] -> "Postscript" *)
let extended_string_of_format_capture format =
  match format with
      PNG -> "PNG"
    | GIF -> "GIF"
    | JPEG -> "JPEG"
    | TIFF -> "TIFF"
    | BMP -> "BMP"
    | PPM -> "PPM"
    | POSTSCRIPT -> "Postscript"

(* [string_of_format_capture format] fournit une chaine correspondant
   au format : [PNG] -> "PNG"; [POSTSCRIPT] -> "PS" *)
let string_of_format_capture format =
  match format with
      PNG -> "PNG"
    | GIF -> "GIF"
    | JPEG -> "JPG"
    | TIFF -> "TIFF"
    | BMP -> "BMP"
    | PPM -> "PPM"
    | POSTSCRIPT -> "PS"

(* [format_capture_of_string chaine] renvoie le type {!Capture.format_capture}
   correspondant a la chaine *)
let format_capture_of_string s =
  match s with
      "PNG"  -> PNG
    | "GIF"  -> GIF
    | "JPG"  -> JPEG
    | "TIFF" -> TIFF
    | "BMP"  -> BMP
    | "PPM"  -> PPM
    | "PS"   -> POSTSCRIPT
    | _      -> PNG

(* [string_of_extension format] renvoie l'extension correspondant au format.
   i.e [PNG] -> ".png" *)
let string_of_extension format =
  match format with
      PNG -> ".png"
    | GIF -> ".gif"
    | JPEG -> ".jpg"
    | TIFF -> ".tiff"
    | BMP -> ".bmp"
    | PPM -> ".ppm"
    | POSTSCRIPT -> ".ps"

(* ============================================================================= *)
(* = Transforme un entier en valeur (R,G,B)                                    = *)
(* ============================================================================= *)
(* [rgb_of_int entier] transforme un entier en valeur (r, g, b) *)
let rgb_of_int v =
  let r = v/65536 in
  let reste = v-(r*65536) in
  let g = reste/256 and
      b = reste mod 256 in
  {r=r; g=g; b=b}

(* ============================================================================= *)
(* = Transforme une Gdk.Image en Image                                         = *)
(* ============================================================================= *)
(* [image_of_gdkimage gdkimge largeur hauteur progress_func] transforme une
   [Gdk.Image] en [Image] *)
let image_of_gdkimage gdkimg width height progress_func =
  let total = 1.0/.float_of_int (height) and cpt = ref 0.0 in

  let img = Rgb24.create width height in
  for y = 0 to height-1 do
    for x = 0 to width-1 do
      Rgb24.set img x y (rgb_of_int (Gdk.Image.get_pixel gdkimg x y)) ;
    done ;
    cpt := !cpt +. 1.0 ;

    (* Appel a la fonction de progression si necessaire *)
    match progress_func with
        None -> ()
      | Some f -> f INIT (!cpt*.total)
  done ;
  Rgb24(img)

(* ============================================================================= *)
(* = Recuperation de la fonction de sauvegarde correspondant au format voulu   = *)
(* ============================================================================= *)
(* [get_save_func format] renvoie la fonction de sauvegarde correspondant au
   format indique *)
let get_save_func format =
  match format with
      PNG -> Png.save
    | GIF -> Gif.save_image
    | JPEG -> Jpeg.save
    | TIFF -> Tiff.save
    | BMP -> Bmp.save
    | PPM -> Ppm.save
    | POSTSCRIPT -> Ps.save

(* ============================================================================= *)
(* = Recuperation de la fonction de chargement correspondant au format voulu   = *)
(* ============================================================================= *)
let get_load_func format =
  match format with
      PNG -> Png.load
    | JPEG -> Jpeg.load
    | TIFF -> Tiff.load
    | BMP -> Bmp.load
    | PPM -> Ppm.load
    | POSTSCRIPT -> Ps.load
    | GIF -> (* Cas particulier pour les images GIF *)
      let save_gif filename opts =
        let sequence = Gif.load filename opts in
        let frame = List.hd sequence.frames in
        Index8 frame.frame_bitmap
      in
      save_gif

(* ============================================================================= *)
(* = Recuperation du nom avec extension pour le format voulu                   = *)
(* ============================================================================= *)
let set_filename_extension filename format =
  let extension = string_of_extension format in
  let lg = String.length extension in
  (* Test si l'extension est deja presente. Si elle ne l'est pas, on l'ajoute... *)
  if (String.length filename) > lg &&
    (String.sub filename ((String.length filename)-lg) lg) = extension then
    filename
  else
    filename^extension

(* ============================================================================= *)
(* = Fonction de remplacement d'une extension                                  = *)
(* =                                                                           = *)
(* = filename   = nom du fichier courant                                       = *)
(* = old_format = ancien format                                                = *)
(* = new_format = nouveau format                                               = *)
(* ============================================================================= *)
let update_extension_capture filename old_format new_format =
  let old_ext = string_of_extension old_format in
  let lg = String.length old_ext in
  if (String.sub filename ((String.length filename)-lg) lg) = old_ext then begin
    (* Il faut supprimer l'ancienne extension *)
    let f = String.sub filename 0 ((String.length filename)-lg) in
    set_filename_extension f new_format ;
  end else
    (* Ajout de la nouvelle extension *)
    set_filename_extension filename new_format

(* ============================================================================= *)
(* = Effectue la capture proprement dite                                       = *)
(* =                                                                           = *)
(* = drawable      = la pixmap ou fenetre contenant l'image a sauver           = *)
(* = x             = coordonnee x du point en haut a gauche                    = *)
(* = y             = coordonnee y du point en haut a gauche                    = *)
(* = width         = largeur de l'image                                        = *)
(* = height        = hauteur de l'image                                        = *)
(* = filename      = nom du fichier                                            = *)
(* = format        = format de sauvegarde (de type format_capture)             = *)
(* = progress_func = fonction appelee lors de la progression (float -> unit)   = *)
(* ============================================================================= *)
let capture_part draw x y width height filename format progress_func =
  (* Creation d'un Gdk.Image a partir d'un Drawable pour pouvoir utiliser *)
  (* la fonction get *)
  let gdk_image = Gdk.Image.get draw x y width height in

  (* Transformation en Image *)
  let image = image_of_gdkimage gdk_image width height progress_func in
  (* On n'a plus besoin de la Gdk.Image *)
  Gdk.Image.destroy gdk_image ;

  (* Sauvegarde de l'Image dans un fichier au format voulu *)
  let save_func = get_save_func format in

  (* Ajout de l'extension appropriee si necessaire au nom de fichier *)
  let filename_save = set_filename_extension filename format in

  (* Appel a la fonction de progression si necessaire *)
  begin
    match progress_func with
        None -> save_func filename_save [] image
      | Some f ->
        save_func filename_save [Save_Progress(f SAVING)] image ;
        (* Fin de la sauvegarde *)
        f FINISHED 0.0
  end

(* ============================================================================= *)
(* = Fonction principale de capture a partir d'un bout de pixmap               = *)
(* =                                                                           = *)
(* = window        = la fenetre mere du drawable suivant                       = *)
(* = drawable      = la pixmap ou fenetre contenant l'image a sauver           = *)
(* = x             = coordonnee x du point en haut a gauche                    = *)
(* = y             = coordonnee y du point en haut a gauche                    = *)
(* = width         = largeur de l'image                                        = *)
(* = height        = hauteur de l'image                                        = *)
(* = filename      = nom du fichier                                            = *)
(* = format        = format de sauvegarde (de type format_capture)             = *)
(* = progress_func = fonction appelee lors de la progression (float -> unit)   = *)
(* = caption       = None ou Some(texte legende, couleur)                      = *)
(* ============================================================================= *)
let capture_part_with_caption window drawable x y width height filename
    format progress_func caption =
  let draw =
    match caption with
        None -> drawable (* Pas de legende *)
      | Some (caption_text, caption_color, contour_color, back_color, font) ->
        (* Copie de la pixmap initiale pour pouvoir y rajouter la legende *)
        let depth = window#misc#visual_depth and w = window#misc#window in
        let pix = Gdk.Pixmap.create ~window:w ~width:(width+x) ~height:(height+y)
          ~depth:depth () in
        let pixmap = new GDraw.pixmap pix in
        pixmap#put_pixmap ~x:x ~y:x drawable ;

        (* Ajout de la legende *)
        let taille_texte = Gdk.Font.string_width font caption_text and
            taille_texte2 = Gdk.Font.string_height font caption_text in
        let x0 = x+(width/2)-taille_texte/2-10 and
            y0 = y+5 and
            taille_x = taille_texte+20 and
            taille_y = taille_texte2+10 in

        pixmap#set_foreground back_color ;
        pixmap#rectangle ~filled:true ~x:x0 ~y:y0 ~width:taille_x ~height:taille_y () ;
        pixmap#set_foreground contour_color ;
        pixmap#rectangle ~filled:false ~x:x0 ~y:y0 ~width:taille_x ~height:taille_y () ;
        pixmap#set_foreground caption_color ;
        pixmap#string caption_text ~font:font
          ~x:(x0+taille_x/2-taille_texte/2) ~y:(y0+taille_y/2+taille_texte2/2) ;

        (* Renvoie le nouveau drawable qui contient la legende *)
        pix
  in
  capture_part draw x y width height filename format progress_func

(* ============================================================================= *)
(* = Fonction principale de capture a partir d'une pixmap complete             = *)
(* =                                                                           = *)
(* = window        = la fenetre mere du drawable suivant                       = *)
(* = drawable      = la pixmap ou fenetre contenant l'image a sauver           = *)
(* = width         = largeur de l'image                                        = *)
(* = height        = hauteur de l'image                                        = *)
(* = filename      = nom du fichier                                            = *)
(* = format        = format de sauvegarde (de type format_capture)             = *)
(* = progress_func = fonction appelee lors de la progression                   = *)
(* =                 du type : (progress_save * float -> unit) option          = *)
(* = caption       = None ou Some(texte legende, couleur)                      = *)
(* ============================================================================= *)
let capture_complete window drawable width height filename format progress_func caption =
  capture_part_with_caption window drawable 0 0 width height filename
    format progress_func caption

(* ============================================================================= *)
(* = Creation d'une image Rgb24 quel que soit le format d'origine              = *)
(* ============================================================================= *)
let gtk_image_rgb24_of_image image =
  match image with
      Images.Index8 i  -> Index8.to_rgb24 i
    | Images.Rgb24  i  -> i
    | Images.Index16 i -> Index16.to_rgb24 i
    | Images.Rgba32  i -> Rgb24.of_rgba32 i
    | Images.Cmyk32  _i -> Printf.printf "Pb : Image Cmyk32 !!!\n"; flush stdout ; exit 1

(* ============================================================================= *)
(* = Lecture d'une image et creation d'une pixmap                              = *)
(* ============================================================================= *)
let gtk_image_load filename win format =
  (* Chargement de l'image *)
  let load_func = get_load_func format in
  let image = load_func filename [] in

  (* Recuperation de sa taille *)
  let (w, h) = Images.size image in

  (* Creation d'une pixmap de meme taille *)
  let create_pixmap window width height =
    let depth = (window:GWindow.window)#misc#visual_depth and w = window#misc#window in
    let pix = Gdk.Pixmap.create ~window:w ~width:width ~height:height ~depth:depth () in
    let pixmap = new GDraw.pixmap pix in
    (pix, pixmap)
  in

  let (_pix, pixmap) = create_pixmap win w h in

  (* Creation d'une image Rgb24 quel que soit le format d'origine *)
  let rgb = gtk_image_rgb24_of_image image in

  (* Transfert de l'image Rgb24 dans la pixmap *)
  for y = 0 to h-1 do
    for x = 0 to w-1 do
      let {Images.r=r; Images.g=g; Images.b=b} = Rgb24.get rgb x y in
      pixmap#set_foreground (`RGB (r*256, g*256, b*256)) ;
      pixmap#point ~x:x ~y:y
    done ;
  done ;

  (* On renvoie la pixmap qui contient a present l'image lue *)
  pixmap

(* =============================== FIN ========================================= *)
