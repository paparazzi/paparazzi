(*
 * $Id$
 *
 * Lablgtk2 utils
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

(* Numero de version *)
let version = "4.10"

(* ================================================================================== *)
(* = Version : 4.10                                                                 = *)
(* = Derniere update : 26/08/2004                                                   = *)
(* =                                                                                = *)
(* = 26/08/2004 : Fonctions de creation de fenetres on top                          = *)
(* =    v4.10                                                                       = *)
(* = 10/08/2004 : Nouvelles fonctions de modification des couleurs de widgets       = *)
(* =    v4.09                                                                       = *)
(* = 09/08/2004 : Ajout de rectangle_pixmap                               = *)
(* =    v4.08                                                                       = *)
(* = 30/07/2004 : Ajout de set_columns_sizes                              = *)
(* =    v4.07                                                                       = *)
(* = 16/12/2003 : Scroll des adjustments avec scroll_adjustement.         = *)
(* =    v4.06     Utilisation de cette fonction pour scroller la fenetre d'affichage= *)
(* =              d'un fichier (gtk_tools_display_file), les listes et les text_edit= *)
(* = 03/11/2003 : Ajout de la molette souris                                        = *)
(* =    v4.05                                                                       = *)
(* = 01/10/2003 : Utilisation de lablgtk-20030326                                   = *)
(* =    v4.04     Encapsulation de GToolbox.popup dans popup car les      = *)
(* =              coordonnees x et y d'avant ne servent en fait a rien              = *)
(* = 25/08/2003 : text_entry_select_text entry                            = *)
(* =    v4.03                                                                       = *)
(* = 04/08/2003 : Widget de selection de fonte : select_font_dlg          = *)
(* =    v4.02                                                                       = *)
(* = 29/07/2003 : Modif a create_option_menu qui renvoie une seconde fct  = *)
(* =    v4.01     permettant de mettre a jour l'option et d'activer le menu         = *)
(* =              correspondant                                                     = *)
(* =              Alignement du texte dans les boutons et padding                   = *)
(* =              Padding dans un label                                             = *)
(* =              but_set_width et but_set_height               = *)
(* = 24/07/2003 : but_set_label, set_widget_front_color et      = *)
(* =    v4.0      set_button_front_color                                  = *)
(* = 23/07/2003 : create_stipple_pixmap_from_data                         = *)
(* =    v3.99                                                                       = *)
(* = 22/07/2003 : calendar et calendar_window                   = *)
(* =    v3.98     set_widget_back_color                                   = *)
(* =              create_sized_label_align* et set_label_align* = *)
(* = 21/07/2003 : calendar                                                = *)
(* =    v3.97                                                                       = *)
(* = 18/06/2003 : create_text_edit, text_edit_clear             = *)
(* =              text_edit_get_text et text_edit_get_lines     = *)
(* =              create_spaced_vframe et create_spaced_hframe  = *)
(* = 16/06/2003 : animated_msg_box                                        = *)
(* = 12/06/2003 : Ajout des fonctions de creation de menus qui peuvent etre actives = *)
(* =    v3.94     ou desactives                                                     = *)
(* = 27/05/2003 : Fonctions de creation de menus                                    = *)
(* =    v3.93                                                                       = *)
(* = 07/04/2003 : get_widget_size widget                                  = *)
(* =    v3.92                                                                       = *)
(* = 03/03/2003 : Eval_string modifiee pour les noms finissant par 'x'              = *)
(* =    v3.91     Le callback de selection des managed lists recoit (row, col, dbl) = *)
(* =              au lieu du double click                                           = *)
(* = 26/02/2003 : connect_win_focus_change                                = *)
(* =    v3.9                                                                        = *)
(* = 10/02/2003 : Passage des rgb<->color dans gtk_draw                             = *)
(* =    v3.8                                                                        = *)
(* = 05/02/2003 : int_spinner_connect/gtk_tools_float_spinner_connect     = *)
(* =    v3.7                                                                        = *)
(* = 30/01/2003 : check_dbl_click                                         = *)
(* = 29/01/2003 : pixmap_from_file                                        = *)
(* = 28/01/2003 : Correction du widget latlon : le menu N/S ou E/W n'etait pas a sa = *)
(* =              bonne valeur au depart si lat<0 ou lon<0                          = *)
(* =              Les fonctions de mise en pointilles marchent mal par defaut.      = *)
(* =              il faut ajouter le parametre width pour que ca marche correctement= *)
(* = 22/01/2003 : color_to_rgb et rgb_to_color,                 = *)
(* =              create_hom_hbox et create_spaced_hbox         = *)
(* = 21/01/2003 : create_infos_win                                        = *)
(* = 15/01/2003 : create_vslider_simple, slider_connect et      = *)
(* =              create_vslider                                          = *)
(* = 21/11/2002 : ajout de create_togglebutton,                           = *)
(* =              create_managed_list, connect_managed_list,    = *)
(* =              create_buttons et create_buttons_connect      = *)
(* =              create_hpaned, create_vpaned                  = *)
(* = 20/11/2002 : create_notebook et notebook_add_page          = *)
(* = 19/11/2002 : ajout de create_ops_compare et                          = *)
(* =              create_time_select                                      = *)
(* = 31/10/2002 : ajout de check_overwrite a la boite de selection de fichier       = *)
(* =              et ajout de question_box                                = *)
(* = 21/10/2002 : set_cursor                                              = *)
(* = 17/10/2002 : list_connect_up_down_keys                               = *)
(* = 15/10/2002 : create_hbox/vbox                                        = *)
(* = 11/10/2002 : create_progress_bar                                     = *)
(* = 26/09/2002 : list_connect_check_dbl_click                            = *)
(* = 18/09/2002 : connect_popup_menu, connect_func_popup_menu   = *)
(* = 16/09/2002 : modifs a create_list_(with_hor_scroll)                  = *)
(* = 13/09/2002 : create_modal_window                                     = *)
(* = 11/09/2002 : display_file                                            = *)
(* = 11/09/2002 : create_window_with_menubar_help                         = *)
(* = 05/09/2002 : show_log,gtk_tools_hide_log,gtk_tools_add_log           = *)
(* = 04/09/2002 : create_optionmenu                                       = *)
(* = 03/09/2002 : disconnect                                              = *)
(* = 18/07/2002 : create_bbox                                             = *)
(* = 18/07/2002 : create_button, create_(sized_)label           = *)
(* = 18/07/2002 : create_hframe, create_frame -> create_vframe            = *)
(* = 17/07/2002 : get_screen_size                                         = *)
(* = 17/07/2002 : get_registered_window                                   = *)
(* = 17/07/2002 : get_window_geometry, window_modify_connect    = *)
(* =              et set_window_position                                  = *)
(* = 16/07/2002 : create_list_with_hor_scroll                             = *)
(* = 11/07/2002 : create_latlon_selection, latlon_selection_get = *)
(* =              update_latlon_selection et                              = *)
(* =              latlon_selection_change                                 = *)
(* = 10/07/2002 : create_text_entry_simple, text_entry_connect  = *)
(* =              et text_entry_connect_modify                            = *)
(* = 10/07/2002 : create_list/gtk_tools_list_connect                      = *)
(* = 10/07/2002 : register_window/gtk_tools_show_registered_window        = *)
(* = 08/07/2002 : area_key_connect et modif de area_connect en  = *)
(* =              area_mouse_connect                                      = *)
(* = 08/07/2002 : create_radiobuttons                                     = *)
(* = 16/05/2002 : screenshot_box + screenshot_box_with_caption  = *)
(* = 14/05/2002 : create_color_selection_button                           = *)
(* = 14/05/2002 : create_scrolled_box et change_scrolled_box    = *)
(* = 20/04/2002 : create_text_entry                                       = *)
(* = 19/04/2002 : create_int_spinner et create_float_spinner    = *)
(* = 19/04/2002 : create_checkbutton_simple, create_checkbutton = *)
(* = 19/04/2002 : Modif dans insert_timer                                 = *)
(* = 23/01/2002 : string_width_height                                     = *)
(* = 23/01/2002 : legende dans screenshot_box                             = *)
(* = 17/01/2002 : create_pixbutton                                        = *)
(* = 17/01/2002 : create_frame                                            = *)
(* = 17/01/2002 : select_colors                                           = *)
(* = 16/01/2002 : set_sensitive & set_sensitive_list            = *)
(* = 16/01/2002 : error_box : boite de message d'erreur                   = *)
(* = 15/01/2002 : screenshot_box : boite de capture d'ecran               = *)
(* = 14/01/2002 : create_popup_menu : ajout de separateurs                = *)
(* = 14/01/2002 : set_widget_font                                         = *)
(* = 10/01/2002 : create_draw_area_simple/gtk_tools_area_connect          = *)
(* = 07/01/2002 : boite de selection de couleur                                     = *)
(* = 04/01/2002 : passage a lablgtk version 1.2.3 (anciennement 1.2.1)              = *)
(* = 02/01/2002 : window_set_front                                        = *)
(* =                                                                                = *)
(* ================================================================================== *)

(* Modules Gtk/Gdk *)
open GMain
open GdkKeysyms
open Gdk
open GToolbox (* Acces a message_box *)

(* Modules locaux *)
open Platform
open Ocaml_tools
open Gtk_tools_icons

(* ============================================================================= *)
(* = Force la mise a jour de l'interface                                       = *)
(* ============================================================================= *)
let force_update_interface () =
  while Glib.Main.pending () do	ignore(Glib.Main.iteration false) done

(* ============================================================================= *)
(* = Initialisation des couleurs                                               = *)
(* ============================================================================= *)
let init_colors () =
  (* Mise en place des couleurs correctes *)
  Gdk.Rgb.init () ;
  GtkBase.Widget.set_default_visual (Gdk.Rgb.get_visual ()) ;
  GtkBase.Widget.set_default_colormap (Gdk.Rgb.get_cmap ())

(* ============================================================================= *)
(* = Lancement de la mainloop pour l'interface                                 = *)
(* ============================================================================= *)
let main_loop () = Main.main ()

(* ============================================================================= *)
(* = Initialisation des tooltips                                               = *)
(* ============================================================================= *)
let init_tooltips () = GData.tooltips ()

(* ============================================================================= *)
(* = Ajout d'un tooltip a un widget                                            = *)
(* =                                                                           = *)
(* = widget = le bouton concerne                                               = *)
(* = texte  = le texte de l'aide contextuelle                                  = *)
(* ============================================================================= *)
let add_tooltips tooltips widget texte =
  (tooltips:GData.tooltips)#set_tip widget#coerce ~text:texte

(* ============================================================================= *)
(* = Renvoie la taille de l'ecran                                              = *)
(* ============================================================================= *)
let get_screen_size () = (Gdk.Screen.width (), Gdk.Screen.height ())

(* ============================================================================= *)
(* = Deconnexion d'un signal                                                   = *)
(* =                                                                           = *)
(* = wid = l'objet concerne                                                    = *)
(* = id  = identifiant recu lors de la connexion du signal                     = *)
(* ============================================================================= *)
let disconnect wid id = wid#misc#disconnect id

(* ============================================================================= *)
(* = Activation/Desactivation d'un widget                                      = *)
(* =                                                                           = *)
(* = widget    = le widget a modifier                                          = *)
(* = sensitive = true si active, false sinon                                   = *)
(* ============================================================================= *)
let set_sensitive widget sensitive =
  ignore(widget#misc#set_sensitive sensitive)

(* ============================================================================= *)
(* = Activation/Desactivation d'une liste de widgets                           = *)
(* =                                                                           = *)
(* = widget_list = la liste des widgets a modifier                             = *)
(* = sensitive   = true si active, false sinon                                 = *)
(* ============================================================================= *)
let set_sensitive_list widget_list sensitive =
  List.iter (fun widget -> set_sensitive widget sensitive) widget_list

(* ============================================================================= *)
(* = Modification du curseur dans une fenetre                                  = *)
(* =                                                                           = *)
(* = window = la fenetre Gdk.window concernee                                  = *)
(* = cursor = le curseur (cree avec Gdk.Cursor.create)                         = *)
(* ============================================================================= *)
let set_cursor window cursor = Gdk.Window.set_cursor window cursor

(* ============================================================================= *)
(* = Taille d'un widget                                                        = *)
(* = widget = le widget dont on desire connaitre la taille                     = *)
(* ============================================================================= *)
let get_widget_size widget =
  let rect = widget#misc#allocation in (rect.Gtk.width, rect.Gtk.height)

(* Boutons evenements souris *)
type bouton_souris = B_GAUCHE | B_DROIT | B_MILIEU | B_NONE

(* ============================================================================= *)
(* = Test d'un bouton souris apres un evenement de type click                  = *)
(* =                                                                           = *)
(* = event = l'evenement souris                                                = *)
(* ============================================================================= *)
let test_mouse_but event =
  match GdkEvent.Button.button event with
	1 -> B_GAUCHE |	2 -> B_MILIEU |	3 -> B_DROIT |_ -> B_NONE

(* ============================================================================= *)
(* = Renvoie la position de la souris apres un evenement de type click         = *)
(* =                                                                           = *)
(* = event = l'evenement souris                                                = *)
(* ============================================================================= *)
let get_mouse_pos_click event =
  (int_of_float (GdkEvent.Button.x event), int_of_float (GdkEvent.Button.y event))

(* ============================================================================= *)
(* = Renvoie la position de la souris apres un evenement de type deplacement   = *)
(* =                                                                           = *)
(* = event = l'evenement souris                                                = *)
(* ============================================================================= *)
let get_mouse_pos_move event =
  (int_of_float (GdkEvent.Motion.x event), int_of_float (GdkEvent.Motion.y event))

(* ============================================================================= *)
(* = Fonction de test d'un double click                                        = *)
(* =                                                                           = *)
(* = event = l'evenement concerne                                              = *)
(* ============================================================================= *)
let check_dbl_click event =
  match GdkEvent.get_type event with `TWO_BUTTON_PRESS -> true | _ -> false

(* ============================================================================= *)
(* = Renvoie la fonte fixed                                                    = *)
(* ============================================================================= *)
let get_fixed_font () =
  if platform_is_unix then Gdk.Font.load_fontset "fixed"
  else Gdk.Font.load_fontset
	  "-misc-arial-medium-r-normal--13-120-75-75-c-80-iso8859-1"

let get_fixed_font2 () =
  Gdk.Font.load_fontset "-misc-fixed-medium-r-normal--13-120-75-75-c-80-iso8859-1"

(* ============================================================================= *)
(* = Modification de la fonte d'un widget                                      = *)
(* =                                                                           = *)
(* = w    = le widget                                                          = *)
(* = font = la fonte a appliquer                                               = *)
(* ============================================================================= *)
let set_widget_font w font =
  let style = w#misc#style#copy in
  style#set_font font ;
  ignore(w#misc#set_style style)

(* ============================================================================= *)
(* = Modification de la couleur d'un widget                                    = *)
(* =                                                                           = *)
(* = w     = le widget                                                         = *)
(* = color = la nouvelle couleur                                               = *)
(* ============================================================================= *)
let set_widget_back_color w color =
  let style = w#misc#style#copy in
  style#set_bg [`NORMAL, color];
  ignore(w#misc#set_style style)
let set_widget_front_color w color =
  let style = w#misc#style#copy in
  style#set_fg [`NORMAL, color];
  ignore(w#misc#set_style style)

let set_entry_back_color w color =
  let style = w#misc#style#copy in
  style#set_base [`NORMAL, color] ;
  ignore(w#misc#set_style style)
let set_entry_front_color w color =
  let style = w#misc#style#copy in
  style#set_text [`NORMAL, color] ;
  ignore(w#misc#set_style style)
let set_entry_outline_color w color =
  set_widget_back_color w color
  
let set_button_back_color w color =
  let style = w#misc#style#copy in
  style#set_bg [`NORMAL, color; `PRELIGHT, color] ;
  ignore(w#misc#set_style style)
let set_button_front_color w color =
  set_widget_front_color (List.hd w#children) color

(* Pour le fond d'un label il faut une event box au dessus *)

(* ============================================================================= *)
(* = Modification de l'alignement d'un widget de type label                    = *)
(* =                                                                           = *)
(* = label = le widget                                                         = *)
(* = pos   = la valeur de l'alignement (entre 0. et 1.)                        = *)
(* ============================================================================= *)
let set_label_align label pos = label#set_xalign pos

let set_label_align_left   label = set_label_align label 0.
let set_label_align_right  label = set_label_align label 1.
let set_label_align_center label = set_label_align label 0.5

let set_label_padding label xpad = label#set_xpad xpad

(* ============================================================================= *)
(* = Taille en pixel de la chaine de caractere dans la fonte indiquee          = *)
(* =                                                                           = *)
(* = font   = la fonte                                                         = *)
(* = string = la chaine de caracteres                                          = *)
(* ============================================================================= *)
let string_width_height font string =
  (Gdk.Font.string_width font string, Gdk.Font.string_height font string)

(* ============================================================================= *)
(* = Creation d'une boite a boutons                                            = *)
(* =                                                                           = *)
(* = pack_method = ou mettre la boite                                          = *)
(* ============================================================================= *)
let create_bbox pack_method =
  GPack.button_box `HORIZONTAL ~border_width:5 ~packing:pack_method
    ~layout:`SPREAD ~spacing:5 ()

(* ============================================================================= *)
(* = Creation d'une boite horizontale                                          = *)
(* =                                                                           = *)
(* = pack_method = ou mettre la boite                                          = *)
(* ============================================================================= *)
let create_hbox pack_method = GPack.hbox ~packing:pack_method ()
let create_hom_hbox pack_method =
  GPack.hbox  ~homogeneous:true ~packing:pack_method ()
let create_spaced_hbox pack_method =
  GPack.hbox  ~spacing:5 ~border_width:5~packing:pack_method ()
let create_hom_spaced_hbox pack_method =
  GPack.hbox  ~homogeneous:true ~spacing:5 ~border_width:5~packing:pack_method ()

(* ============================================================================= *)
(* = Creation d'une boite verticale                                            = *)
(* =                                                                           = *)
(* = pack_method = ou mettre la boite                                          = *)
(* ============================================================================= *)
let create_vbox pack_method = GPack.vbox ~packing:pack_method ()
let create_hom_vbox pack_method =
  GPack.vbox  ~homogeneous:true ~packing:pack_method ()
let create_spaced_vbox pack_method =
  GPack.vbox  ~spacing:5 ~border_width:5~packing:pack_method ()
let create_hom_spaced_vbox pack_method =
  GPack.vbox  ~homogeneous:true ~spacing:5 ~border_width:5~packing:pack_method ()

(* ============================================================================= *)
(* = Creation d'une frame contenant une vbox                                   = *)
(* =                                                                           = *)
(* = title       = titre de la frame                                           = *)
(* = pack_method = ou mettre la frame                                          = *)
(* = Renvoie la frame et la vbox                                               = *)
(* ============================================================================= *)
let create_vframe title pack_method =
  let fr = GBin.frame ~label:title ~packing:pack_method () in
  (fr, create_vbox fr#add)

let create_spaced_vframe title pack_method =
  let fr = GBin.frame ~label:title ~packing:pack_method () in
  (fr, create_spaced_vbox fr#add)

(* ============================================================================= *)
(* = Creation d'une frame contenant une hbox                                   = *)
(* =                                                                           = *)
(* = title       = titre de la frame                                           = *)
(* = pack_method = ou mettre la frame                                          = *)
(* = Renvoie la frame et la vbox                                               = *)
(* ============================================================================= *)
let create_hframe title pack_method =
  let fr = GBin.frame ~label:title ~packing:pack_method () in
  (fr, create_hbox fr#add)

let create_spaced_hframe title pack_method =
  let fr = GBin.frame ~label:title ~packing:pack_method () in
  (fr, create_spaced_hbox fr#add)

(* ============================================================================= *)
(* = Creation d'une zone scrollable contenant une boite a widgets              = *)
(* =                                                                           = *)
(* = pack_method = ou mettre la zone scrollable                                = *)
(* ============================================================================= *)
let create_scrolled_box pack_method =
  let scrolled_window = GBin.scrolled_window ~border_width: 10
	  ~hpolicy: `AUTOMATIC ~packing:pack_method () in
  (scrolled_window, create_vbox scrolled_window#add_with_viewport)

(* ============================================================================= *)
(* = Destruction et recreation d'une boite verticale dans une zone scrollable  = *)
(* =                                                                           = *)
(* = scrolled_window = zone scrollable a modifier                              = *)
(* = old_box         = ancienne boite contenue dans cette zone                 = *)
(* ============================================================================= *)
let change_scrolled_box scrolled_window old_box =
  old_box#destroy () ;
  create_vbox scrolled_window#add_with_viewport

(* ============================================================================= *)
(* = Creation d'un widget de type notebook                                     = *)
(* =                                                                           = *)
(* = pack_method       = ou mettre le widget                                   = *)
(* ============================================================================= *)
let create_notebook pack_method =
  GPack.notebook ~scrollable:true ~packing:pack_method ()


(* ============================================================================= *)
(* = Creation d'un paned (division mobile entre deux zones d'une fenetre)      = *)
(* =                                                                           = *)
(* = pack_method = ou mettre le widget                                         = *)
(* ============================================================================= *)
let create_hpaned pack_method =
  GPack.paned ~packing:pack_method `HORIZONTAL () 

let create_vpaned pack_method =
  GPack.paned ~packing:pack_method `VERTICAL () 

(* ============================================================================= *)
(* = Creation d'un bouton                                                      = *)
(* =                                                                           = *)
(* = label       = texte du bouton                                             = *)
(* = pack_method = ou mettre le bouton                                         = *)
(* ============================================================================= *)
let create_button label pack_method =
  GButton.button ~label:label ~packing:pack_method ()

(* ============================================================================= *)
(* = Creation d'un bouton avec une taille fixee                                = *)
(* =                                                                           = *)
(* = label       = texte du bouton                                             = *)
(* = width       = taille du bouton                                            = *)
(* = pack_method = ou mettre le bouton                                         = *)
(* ============================================================================= *)
let create_sized_button label width pack_method =
  let but = GButton.button ~label:label ~packing:pack_method () in
  but#misc#set_size_request ~width:width () ;
  but
(* GTK2 AAA  GButton.button ~label:label ~width:width ~packing:pack_method () *)

(* ============================================================================= *)
(* = Modification du texte dans un bouton                                      = *)
(* =                                                                           = *)
(* = button      = le bouton                                                   = *)
(* = label       = texte du bouton                                             = *)
(* ============================================================================= *)
let but_set_label button label =
  (GMisc.label_cast (List.hd button#children))#set_text label

let set_button_align button pos =
  set_label_align (GMisc.label_cast (List.hd button#children)) pos
let set_button_align_left button  =set_button_align button 0.
let set_button_align_right button =set_button_align button 1.
let set_button_align_center button=set_button_align button 0.5

let set_button_padding button xpad =
  set_label_padding (GMisc.label_cast (List.hd button#children)) xpad

let but_set_width button width =
  (button:GButton.button)#misc#set_size_request ~width ()
(* GTK2 AAA GtkBase.Container.set (GtkButton.Button.cast button#as_widget) ~width:width*)
let but_set_height button height =
  (button:GButton.button)#misc#set_size_request ~height ()
(* GTK2 AAA  GtkBase.Container.set (GtkButton.Button.cast button#as_widget) ~height:height*)

(* ============================================================================= *)
(* = Connexion d'une fonction a un bouton                                      = *)
(* =                                                                           = *)
(* = but  = le bouton                                                          = *)
(* = func = la fonction                                                        = *)
(* ============================================================================= *)
let but_connect but func = ignore(but#connect#clicked ~callback:func)

(* ============================================================================= *)
(* = Creation d'une rangee de boutons                                          = *)
(* =                                                                           = *)
(* = lst_buts    = liste des (noms, tooltips)                                  = *)
(* = tooltips    = aide contextuelle                                           = *)
(* = pack_method = ou mettre les boutons                                       = *)
(* ============================================================================= *)
let create_buttons lst_buts tooltips pack_method =
  let bbox = create_bbox pack_method in
  List.map (fun (text, tip) ->
	let b = create_button text bbox#add in
	add_tooltips tooltips b tip;
	b) lst_buts

(* ============================================================================= *)
(* = Association de callbacks a une liste de boutons                           = *)
(* =                                                                           = *)
(* = lst_buts      = liste des boutons creee avec la fonction precedente       = *)
(* = lst_callbacks = callbacks associes                                        = *)
(* ============================================================================= *)
let create_buttons_connect lst_buts lst_callbacks =
  List.iter2 (fun b c -> but_connect b c) lst_buts lst_callbacks

(* ============================================================================= *)
(* = Creation d'un check button sans callback                                  = *)
(* =                                                                           = *)
(* = active      = valeur active/inactif au depart                             = *)
(* = label       = texte du label                                              = *)
(* = pack_method = ou mettre les widgets                                       = *)
(* = tip         = texte de l'aide contextuelle                                = *)
(* = tooltips    = aide contextuelle                                           = *)
(* ============================================================================= *)
let create_checkbutton_simple active label pack_method tip tooltips =
  let check = GButton.check_button ~label:label ~active:active
	  ~packing:pack_method () in
  if tip <> "" then	add_tooltips tooltips check tip ;
  check

(* ============================================================================= *)
(* = Creation d'un check button avec callback                                  = *)
(* =                                                                           = *)
(* = active      = valeur active/inactif au depart                             = *)
(* = label       = texte du label                                              = *)
(* = pack_method = ou mettre les widgets                                       = *)
(* = tip         = texte de l'aide contextuelle                                = *)
(* = tooltips    = aide contextuelle                                           = *)
(* = callback    = callback appele lors de la modification du bouton           = *)
(* ============================================================================= *)
let create_checkbutton active label pack_method tip tooltips callback =
  let check = create_checkbutton_simple active label pack_method
	  tip tooltips in
  but_connect check (fun () -> callback check#active) ;
  check

(* ============================================================================= *)
(* = Creation d'une suite de radio buttons                                     = *)
(* =                                                                           = *)
(* = lst_names   = liste des noms des boutons avec leurs types (nom, type)     = *)
(* = func_active = fonction indiquant quel est le bouton actif par defaut      = *)
(* = pack_method = ou mettre les widgets                                       = *)
(* =                                                                           = *)
(* = Renvoie la liste des boutons crees (pour ajout de tooltips par exemple)   = *)
(* ============================================================================= *)
let create_radiobuttons_simple lst_names func_active pack_method =
  let lst_but = List.fold_left (fun lst (name, typ) ->
	let but =
	  if lst = [] then GButton.radio_button ~label:name ~packing:pack_method ()
	  else GButton.radio_button ~label:name ~packing:pack_method
		  ~group:(List.hd lst)#group ()
	in
	but#set_active (func_active typ) ;
	lst @ [but]) [] lst_names in

  lst_but

(* ============================================================================= *)
(* = Connexion de callbacks a des radiobuttons                                 = *)
(* =                                                                           = *)
(* = lst_names   = liste des noms des boutons avec leurs types (nom, type)     = *)
(* = lst_but     = liste des radiobuttons                                      = *)
(* = func_select = fonction appelee lors de la modification du bouton actif    = *)
(* ============================================================================= *)
let radiobuttons_connect lst_names lst_but func_select =
  List.iter2 (fun but (_, typ) ->
	ignore((but:GButton.radio_button)#connect#clicked ~callback:(fun () ->
	  func_select typ))) lst_but lst_names

(* ============================================================================= *)
(* = Creation d'une suite de radio buttons                                     = *)
(* =                                                                           = *)
(* = lst_names   = liste des noms des boutons avec leurs types (nom, type)     = *)
(* = func_active = fonction indiquant quel est le bouton actif par defaut      = *)
(* = func_select = fonction appelee lors de la modification du bouton actif    = *)
(* = pack_method = ou mettre les widgets                                       = *)
(* =                                                                           = *)
(* = Renvoie la liste des boutons crees (pour ajout de tooltips par exemple)   = *)
(* ============================================================================= *)
let create_radiobuttons lst_names func_active func_select pack_method =
  let lst_but = create_radiobuttons_simple
	  lst_names func_active pack_method in

  (* Connexion de la fonction de selection *)
  radiobuttons_connect lst_names lst_but func_select ;
  lst_but

(* ============================================================================= *)
(* = Creation d'un toggle button                                               = *)
(* =                                                                           = *)
(* = label       = texte du bouton                                             = *)
(* = active      = indique l'etat initial du bouton                            = *)
(* = pack_method = ou mettre le widget                                         = *)
(* ============================================================================= *)
let create_togglebutton label active pack_method =
  GButton.toggle_button ~label:label ~active:active ~packing:pack_method ()

(* ============================================================================= *)
(* = Creation d'un bouton contenant une pixmap                                 = *)
(* =                                                                           = *)
(* = pm          = la pixmap a mettre dans le bouton                           = *)
(* = pack_method = ou mettre le bouton                                         = *)
(* ============================================================================= *)
let create_pixbutton pm pack_method =
  let but = GButton.button ~packing:pack_method () in
  ignore(GMisc.pixmap (pm:GDraw.pixmap) ~packing:but#add ()) ;
  but

(* ============================================================================= *)
(* = Creation d'un label                                                       = *)
(* =                                                                           = *)
(* = label       = texte du label                                              = *)
(* = pack_method = ou mettre le label                                          = *)
(* ============================================================================= *)
let create_label label pack_method =
  GMisc.label ~text:label ~packing:pack_method ()

(* ============================================================================= *)
(* = Creation d'un label ayant une taille fixee                                = *)
(* =                                                                           = *)
(* = label       = texte du label                                              = *)
(* = width       = largeur du label en pixels                                  = *)
(* = pack_method = ou mettre le label                                          = *)
(* ============================================================================= *)
let create_sized_label label width pack_method =
  GMisc.label ~text:label ~width:width ~packing:pack_method ()

let create_sized_label_align label width pos pack_method =
  GMisc.label ~text:label ~width:width ~packing:pack_method ~xalign:pos ()
let create_sized_label_align_left label width pack_method =
  GMisc.label ~text:label ~width:width ~packing:pack_method ~xalign:0. ()
let create_sized_label_align_right label width pack_method =
  GMisc.label ~text:label ~width:width ~packing:pack_method ~xalign:1. ()

(* ============================================================================= *)
(* = Boite de question                                                         = *)
(* =                                                                           = *)
(* = title             = titre de la fenetre                                   = *)
(* = question_msg      = message a afficher                                    = *)
(* = default_is_cancel = indique si c'est le bouton annuler qui est le defaut  = *)
(* ============================================================================= *)
let question_box window title question_msg default_is_cancel =
  let pm = GDraw.pixmap_from_xpm_d ~data:question_icon ~window:window () in
  let icon = (GMisc.pixmap pm ())#coerce in
  (* Fonction de remplacement d'une extension *)
  let reponse = question_box ~title:title ~buttons:["Oui"; "Annuler"]
	  ~default:(if default_is_cancel then 2 else 1)
	  ~icon:icon question_msg in
  (* Renvoie vrai si "Oui" est selectionne *)
  reponse=1

(* ============================================================================= *)
(* = Boite de message d'erreur                                                 = *)
(* =                                                                           = *)
(* = title     = titre de la fenetre                                           = *)
(* = error_msg = message d'erreur a afficher                                   = *)
(* ============================================================================= *)
let error_box window title error_msg =
  let pm = GDraw.pixmap_from_xpm_d ~data:warning_icon ~window:window () in
  let icon = (GMisc.pixmap pm ())#coerce in
  message_box ~title:title ~icon:icon error_msg

(* ============================================================================= *)
(* = Boite de message avec icone anime                                         = *)
(* =                                                                           = *)
(* = title     = titre de la fenetre                                           = *)
(* = error_msg = message d'erreur a afficher                                   = *)
(* ============================================================================= *)
let animated_msg_box title msg lst_pixmaps =
  let window = GWindow.dialog ~modal:true ~title:title () in

  let max_pixmaps = List.length lst_pixmaps in
  let l = List.map (fun f ->
	GDraw.pixmap_from_xpm ~file:f ~window:window ()) lst_pixmaps in
  let t = Array.of_list l in

  let hb = GPack.hbox ~border_width:10 ~packing:window#vbox#add () in
  ignore (create_label msg (hb#pack ~from:`END));
  let hbox = ref (create_hbox hb#pack) in

  (* Fonction de mise a jour du pixmap dans la boite de dialogue *)
  let create pm =
	(!hbox)#destroy () ;
	hbox := create_hbox hb#pack ;
	(!hbox)#pack (GMisc.pixmap pm ())#coerce ~padding:4 ;
  in
  create t.(0) ;

  let b = create_button "OK"
	  (window#action_area#pack ~expand:true ~padding:4) in
  but_connect b window#destroy ;
  b#grab_default () ;
  window#set_position `CENTER;
  window#show ();

  (* Timeout de l'animation *)
  let current_idx = ref 0 in
  let timeout _ =
	incr current_idx ;
	if !current_idx>=max_pixmaps then current_idx:=0 ;
	create t.(!current_idx) ;
	true
  in
  let timeoutid = Timeout.add ~ms:100 ~callback:timeout in

  (* On stoppe le timeout quand la boite de dialogue est fermee *)
  ignore (window#connect#destroy ~callback: (fun () ->
    Timeout.remove timeoutid ; GMain.Main.quit ())) ;

  GMain.Main.main ()

(* ============================================================================= *)
(* = Ouverture d'une boite de dialogue de selection de fichier. On teste       = *)
(* = l'existence du fichier si c'est demande                                   = *)
(* =                                                                           = *)
(* = title            : titre de la fenetre de selection                       = *)
(* = read_func        : fonction de lecture du fichier apres selection         = *)
(* = update_func      : None ou Some f, fonction appelee apres read_func       = *)
(* = default_filename : nom selectionne par defaut si <> ""                    = *)
(* = check_overwrite  : indique si on teste l'existence du fichier             = *)
(* ============================================================================= *)
let open_file_dlg title read_func update_func default_filename
	check_overwrite =
  let do_select_file filename =
	(* Appel a la fonction de lecture du fichier *)
	read_func filename;
	(* Appel a la fonction de mise a jour si besoin *)
	match update_func with None -> () | Some f -> f ()
  in

  (* Creation de la boite de dialogue *)
  let fs = GWindow.file_selection ~modal:true ~title:title () in

  let check_file_overwrite_ok filename =
	try
      let stat = Unix.stat filename in
      if stat.Unix.st_kind = Unix.S_REG then begin
		question_box fs
		  "Le fichier existe" "Le fichier existe, continuer ?" false
      end else true
    with
      Unix.Unix_error (Unix.ENOENT, _, _) -> true
	| _ -> false
  in

  ignore(fs#ok_button#connect#clicked ~callback:(fun () ->
	(* Recuperation du nom du fichier *)
	let filename = fs#filename in
	if not check_overwrite or (check_file_overwrite_ok filename) then begin
	  fs#destroy () ;
	  do_select_file filename
	end)) ;

  ignore(fs#cancel_button#connect#clicked ~callback:fs#destroy) ;
  (* Mise a jour du nom de fichier par defaut *)
  if default_filename <> "" then fs#set_filename default_filename ;
  fs#show ()

type timer_type =
	TIMER_TIME          (* Heure uniquement *)
  | TIMER_DATE          (* Date uniquement  *)
  | TIMER_TIME_AND_DATE (* Affichage de l'heure et de la date *)

(* ============================================================================= *)
(* = Insertion d'un timer dans un label                                        = *)
(* =                                                                           = *)
(* = label           = le widget label ou afficher le timer                    = *)
(* = timer_type      = TIMER_TIME, TIMER_DATE ou TIMER_TIME_AND_DATE           = *)
(* = force_beginning = true si active lance des le depart                      = *)
(* ============================================================================= *)
let insert_timer label timer_type force_beginning =
  (* Fonction appelee *)
  let timeout () =
	let tm = timer_get_time () in
	let val_time =
	  match timer_type with
		TIMER_TIME -> timer_string_of_time tm
	  |	TIMER_DATE -> timer_string_of_date tm
	  |	TIMER_TIME_AND_DATE -> (timer_string_of_time tm) ^ " - " ^
		  (timer_string_of_date tm)
	in

	(label:GMisc.label)#set_text val_time;
	true in

  (* Mise en place du timer toutes les 1000 millisecondes *)
  ignore(Timeout.add ~ms:1000 ~callback:timeout) ;

  (* Si c'est demande, on force l'affichage des le debut *)
  if force_beginning then ignore(timeout ())

(* ============================================================================= *)
(* = Encapsulation de GToolbox.popup                                           = *)
(* ============================================================================= *)
let popup menu_entries =
  GToolbox.popup_menu ~button:0 ~time:Int32.zero ~entries:menu_entries

(* ============================================================================= *)
(* = Connexion d'un popup menu a un bouton                                     = *)
(* =                                                                           = *)
(* = wid            = le wiget concerne                                        = *)
(* = button         = bouton a presser pour afficher le menu                   = *)
(* = test_cond_func = fonction testant si le menu doit etre affiche            = *)
(* = menu_entries   = entrees a afficher dans le menu                          = *)
(* ============================================================================= *)
let connect_popup_menu wid button test_cond_func menu_entries =
  ignore(wid#event#connect#button_press ~callback:(fun ev ->
	if test_cond_func () &&
	  (test_mouse_but ev) = button &&
	  GdkEvent.get_type ev = `BUTTON_PRESS then begin
		popup menu_entries ; true
	  end else false))

(* ============================================================================= *)
(* = Connexion d'un popup menu a un bouton. Le menu est construit de maniere   = *)
(* = dynamique, il provient de l'appel a une fonction                          = *)
(* =                                                                           = *)
(* = wid              = le wiget concerne                                      = *)
(* = button           = bouton a presser pour afficher le menu                 = *)
(* = test_cond_func   = fonction testant si le menu doit etre affiche          = *)
(* = get_menu_entries = fonction renvoyant les entrees a afficher dans le menu = *)
(* ============================================================================= *)
let connect_func_popup_menu wid button test_cond_func get_menu_entries =
  ignore(wid#event#connect#button_press ~callback:(fun ev ->
	if test_cond_func () &&
	  (test_mouse_but ev) = button &&
	  GdkEvent.get_type ev = `BUTTON_PRESS then begin
		popup (get_menu_entries ()) ;	true
	  end else false))

(* ============================================================================= *)
(* = Creation d'un menu la ou se trouve la souris                              = *)
(* =                                                                           = *)
(* = title = titre du menu ("" si pas de titre)                                = *)
(* = event = evenement de type click souris                                    = *)
(* = data  = liste de parametres du type (texte, Some fonction, param) ou      = *)
(* = texte designe le texte correspondant au sous-menu et fonction la fonction = *)
(* = a appeler apres selection avec le parametre param                         = *)
(* ============================================================================= *)
let create_popup_menu title event data =
  let button = GdkEvent.Button.button event and
	  time = GdkEvent.Button.time event and
	  menu = GMenu.menu ~show:true () in

  (* Fonction d'ajout de sous-menus *)
  let attache_menu (texte, fonction, param, sous_menu) =
	(* Ajout d'un separateur ou d'un sous-menu ? *)
	if texte = "" then ignore(GMenu.menu_item ~packing:menu#append ())
	else begin
	  let ssmenu = GMenu.menu_item ~label:texte ~packing:menu#append () in
	  begin
		match fonction with
		  None -> ()
		| Some f -> ignore(ssmenu#connect#activate ~callback:(fun () -> f param))
	  end ;

	  (* Ajout d'un sous-menu *)
	  if sous_menu <> [] then begin
		let m = GMenu.menu () in
		List.iter (fun (t, func, param) ->
		  let ss_m = GMenu.menu_item ~label:t ~packing:m#append () in
		  match func with
			None -> ()
		  |	Some f -> ignore(ss_m#connect#activate ~callback:(fun () -> f param))
				  ) sous_menu ;
		ssmenu#set_submenu m
	  end
	end
  in

  if title <> "" then begin
	(* Ajout du titre *)
	attache_menu (title, None, "", []) ;
	(* Ajout d'un separateur apres le titre *)
	attache_menu ("", None, "", []) ;
  end ;
  List.iter attache_menu data ;

  (* Affichage du popup-menu *)
  menu#popup ~button:button ~time:time

(* ============================================================================= *)
(* = Creation d'un menu a options                                              = *)
(* =                                                                           = *)
(* = lst_names   = liste des noms des options avec leurs types (nom, type)     = *)
(* = func_active = fonction indiquant quelle est l'option active par defaut    = *)
(* = func_select = fonction appelee lors de la modification de l'option        = *)
(* = pack_method = ou mettre le menu                                           = *)
(* =                                                                           = *)
(* = Renvoie le menu et la fonction de selection d'une option                  = *)
(* ============================================================================= *)
let create_optionmenu lst_names func_active func_select pack_method =
  (* Creation des options *)
  let menu = GMenu.menu () in
  let lst_menus =
	List.map (fun (nom, typ) ->
	  let m = GMenu.menu_item ~label:nom ~packing: menu#append () in
	  ignore (m#connect#activate ~callback:(fun () -> func_select typ)) ;
	  m) lst_names in

  (* Creation de l'option menu *)
  let optionmenu = GMenu.option_menu ~packing:pack_method () in
  optionmenu#set_menu menu ;

  (* Recherche du numero du menu de l'option par defaut *)
  let n = ref 0 and idx = ref 0 in
  List.iter (fun (_, typ) -> if func_active typ then n:=!idx; incr idx) lst_names ;
  optionmenu#set_history !n ;

  let set_option typ =
	let n = ref 0 and idx = ref 0 in
	List.iter (fun (_, typ0) -> if typ0=typ then n:=!idx; incr idx) lst_names ;
	optionmenu#set_history !n ;
  in

  let set_option_and_activate typ =
	let n = ref 0 and idx = ref 0 in
	List.iter2 (fun (_, typ0) m ->
	  if typ0=typ then begin n:=!idx; m#activate () end ;
	  incr idx) lst_names lst_menus ;
	optionmenu#set_history !n ;
  in

  (optionmenu, set_option, set_option_and_activate)

(* ============================================================================= *)
(* = Pixmap d'ouverture d'un fichier                                           = *)
(* ============================================================================= *)
let open_file_pixmap =
  [|(* width height num_colors chars_per_pixel *)
	  "    20    19       5            1";
	(* colors *)
	". c None";"# c #000000";"i c #ffffff";"s c #7f7f00";"y c #ffff00";
	(* pixels *)
	"....................";	"....................";	"....................";
	"...........###......";	"..........#...#.#...";	"...............##...";
	"...###........###...";	"..#yiy#######.......";	"..#iyiyiyiyi#.......";
	"..#yiyiyiyiy#.......";	"..#iyiy###########..";	"..#yiy#sssssssss#...";
	"..#iy#sssssssss#....";	"..#y#sssssssss#.....";	"..##sssssssss#......";
	"..###########.......";	"....................";	"....................";
	"...................." |]

(* ============================================================================= *)
(* = Creation d'une GDraw.pixmap a partir d'un fichier xpm                     = *)
(* =                                                                           = *)
(* = filename = nom du fichier xpm                                             = *)
(* = window   = fenetre                                                        = *)
(* ============================================================================= *)
let pixmap_from_file filename window =
  GDraw.pixmap_from_xpm ~file:filename ~window:(window:GWindow.window) ()

(* ============================================================================= *)
(* = Creation d'une pixmap                                                     = *)
(* =                                                                           = *)
(* = window = fenetre mere                                                     = *)
(* = width  = largeur en pixels de la pixmap                                   = *)
(* = height = hauteur en pixels de la pixmap                                   = *)
(* =                                                                           = *)
(* = En retour, pix sert a mettre la pixmap dans une zone de dessin par ex.    = *)
(* = et pixmap sert au dessin.                                                 = *)
(* ============================================================================= *)
let create_pixmap window width height =
  let depth = (window:GWindow.window)#misc#visual_depth and w = window#misc#window in
  let pix = Pixmap.create ~window:w ~width:width ~height:height ~depth:depth () in
  let pixmap = new GDraw.pixmap pix in
  (pix, pixmap)

let create_d_pixmap window width height =
  let depth = (window:GWindow.window)#misc#visual_depth and w = window#misc#window in
  let pix = Pixmap.create ~window:w ~width:width ~height:height ~depth:depth () in
  let pixmap = new GDraw.drawable pix in
  (pix, pixmap)

(* ============================================================================= *)
(* = Fonction de creation d'une fenetre avec une boite verticale               = *)
(* =                                                                           = *)
(* = title  = titre de la fenetre                                              = *)
(* = width  = largeur de la fenetre                                            = *)
(* = height = hauteur de la fenetre                                            = *)
(* ============================================================================= *)
let create_window title width height =
  let window =
	if width = 0 && height = 0 then
	  GWindow.window ~border_width: 1 ~title ()
	else
	  GWindow.window ~width ~height ~border_width: 1 ~title ()
  in
  (window, create_vbox window#add)

(* ============================================================================= *)
(* = Fonction de creation d'une fenetre modale avec une boite verticale        = *)
(* =                                                                           = *)
(* = title  = titre de la fenetre                                              = *)
(* = width  = largeur de la fenetre                                            = *)
(* = height = hauteur de la fenetre                                            = *)
(* ============================================================================= *)
let create_modal_window title width height =
  let window =
	if width = 0 && height = 0 then
	  GWindow.window ~modal:true ~border_width: 1 ~title ()
	else
	  GWindow.window ~modal:true ~width ~height	~border_width: 1 ~title ()
  in
  (window, create_vbox window#add)

(* ============================================================================= *)
(* = Fonction de creation d'une fenetre avec une boite verticale. De plus, la  = *)
(* = fenetre contient une barre de menu                                        = *)
(* =                                                                           = *)
(* = title         = titre de la fenetre                                       = *)
(* = width         = largeur de la fenetre                                     = *)
(* = height        = hauteur de la fenetre                                     = *)
(* = menubar_items = liste de chaines contenant les titres des menus           = *)
(* ============================================================================= *)
let create_window_with_menubar title width height menubar_items =
  let (window, vbox) = create_window title width height in
  let menubar = GMenu.menu_bar ~packing:vbox#pack () in
  let factory = new GMenu.factory menubar in
  let menus = List.map (fun str -> factory#add_submenu str) menubar_items in

  (window, vbox, factory, factory#accel_group, Array.of_list menus)

(* ============================================================================= *)
(* = Fonction de creation d'une fenetre avec une boite verticale. De plus, la  = *)
(* = fenetre contient une barre de menu et un menu a gauche pour l'aide        = *)
(* =                                                                           = *)
(* = title         = titre de la fenetre                                       = *)
(* = width         = largeur de la fenetre                                     = *)
(* = height        = hauteur de la fenetre                                     = *)
(* = menubar_items = liste de chaines contenant les titres des menus           = *)
(* ============================================================================= *)
let create_window_with_menubar_help title width height menubar_items =
  let (window, vbox) = create_window title width height in
  let hbox = create_hbox vbox#pack in
  let menubar = GMenu.menu_bar ~packing:hbox#add () in
  let factory = new GMenu.factory menubar in
  let menus = List.map (fun str -> factory#add_submenu str) menubar_items in

  let menubar_hlp = GMenu.menu_bar ~packing:(hbox#pack ~from:`END) () in
  let factory_hlp = new GMenu.factory menubar_hlp in
  let menu_help = factory_hlp#add_submenu "Help" in

  (window, vbox, factory, factory#accel_group, Array.of_list menus, menu_help)

(* ============================================================================= *)
(* = Passage en avant-plan d'une fenetre                                       = *)
(* =                                                                           = *)
(* = window = la fenetre a passer en avant-plan                                = *)
(* ============================================================================= *)
let window_set_front window =
  (* On est oblige de masquer la fenetre avant de la montrer pour etre *)
  (* sur qu'elle passe en avant plan. Show tout seul ne suffit pas...  *)
  GtkBase.Widget.hide (window:GWindow.window)#as_window ;
  ignore(window#show ())

(* ============================================================================= *)
(* = Renvoie la geometrie d'une fenetre                                        = *)
(* =                                                                           = *)
(* = window = la fenetre concernee                                             = *)
(* ============================================================================= *)
let get_window_geometry window =
  let pos  = Window.get_position (window:GWindow.window)#misc#window and
	  size = Drawable.get_size window#misc#window in
  (pos, size)

(* ============================================================================= *)
(* = Fixe la position x, y d'une fenetre (la deplace si besoin)                = *)
(* =                                                                           = *)
(* = window = la fenetre concernee                                             = *)
(* = (x, y) = la nouvelle position                                             = *)
(* ============================================================================= *)
let set_window_position window (x, y) =
  GtkBase.Widget.set_uposition (window:GWindow.window)#as_window ~x:x ~y:y

(* ============================================================================= *)
(* = Connexion d'un callback lors du deplacement ou de la modification d'une   = *)
(* = fenetre                                                                   = *)
(* =                                                                           = *)
(* = window   = la fenetre concernee                                           = *)
(* = callback = le callback a appeler                                          = *)
(* ============================================================================= *)
let window_modify_connect window callback =
  (window:GWindow.window)#event#connect#configure ~callback:(fun _ev ->
	callback (get_window_geometry window) ;
	true)

(* ============================================================================= *)
(* = Connecte les callbacks lies aux evenements focus_in et focus_out          = *)
(* =                                                                           = *)
(* = window    = la fenetre concernee                                          = *)
(* = focus_in  = fonction appelee lorsque la souris entre dans le widget       = *)
(* = focus_out = fonction appelee lorsque la souris sort du widget             = *)
(* ============================================================================= *)
let connect_win_focus_change window focus_in focus_out =
  let win = (window:GWindow.window) in
  (match focus_in with
	None -> ()
  | Some f -> ignore(win#event#connect#focus_in (fun _ -> f (); false))) ;
  (match focus_out with
	None -> ()
  | Some f -> ignore(win#event#connect#focus_out (fun _ -> f (); false)))

(* ============================================================================= *)
(* = Creation d'une zone de dessin simple (i.e pas de callback associe)        = *)
(* =                                                                           = *)
(* = width       = hauteur de la zone                                          = *)
(* = height      = hauteur de la zone                                          = *)
(* = pack_method = maniere de placer la zone (ex. : hbox#pack)                 = *)
(* = pix_expose  = pixmap a utiliser pour redessiner apres un event expose     = *)
(* ============================================================================= *)
let create_draw_area_simple width height pack_method pix_expose =
  (* Creation des widgets *)
  let area = GMisc.drawing_area ~width:width ~height:height ~packing:pack_method () in
  let drawing = area#misc#realize (); new GDraw.drawable (area#misc#window) in

  (* Creation du event expose : remise en place de la pixmap dans la zone de dessin *)
  let area_expose () = drawing#put_pixmap ~x:0 ~y:0 pix_expose in
  ignore(area#event#connect#expose ~callback:(fun _ -> area_expose () ; false)) ;

  (* Renvoie la fonction de reaffichage et la zone de dessin *)
  (area, area_expose)

(* ============================================================================= *)
(* = Connexion evenements souris a une zone de dessin                          = *)
(* =                                                                           = *)
(* = area          = la zone de dessin (create_draw_area_simple)     = *)
(* = mouse_press   = fonction appelee lors d'un click souris                   = *)
(* = mouse_move    = fonction appelee lors d'un deplacement                    = *)
(* = mouse_release = fonction appelee lors du relachement d'un bouton          = *)
(* ============================================================================= *)
let area_mouse_connect area mouse_press mouse_move mouse_release =
  (area:GMisc.drawing_area)#event#add
	[`POINTER_MOTION; `BUTTON_PRESS; `BUTTON_RELEASE] ;
  area#event#set_extensions `ALL;
  ignore(area#event#connect#button_press   ~callback:mouse_press) ;
  ignore(area#event#connect#motion_notify  ~callback:mouse_move) ;
  ignore(area#event#connect#button_release ~callback:mouse_release)

(* ============================================================================= *)
(* = Connexion evenements clavier a une zone de dessin                         = *)
(* =                                                                           = *)
(* = area        = la zone de dessin (create_draw_area_simple)       = *)
(* = key_press   = fonction appelee lors de l'appui sur une touche             = *)
(* = key_release = fonction appelee lors du relachement d'une touche           = *)
(* ============================================================================= *)
let area_key_connect area key_press key_release =
  (* Par defaut l'evenement key_release n'est pas associe au widget *)
  (area:GMisc.drawing_area)#event#add [`KEY_RELEASE] ;
  ignore(area#event#connect#key_press
		   ~callback:(fun ev -> key_press   (GdkEvent.Key.keyval ev))) ;
  ignore(area#event#connect#key_release
		   ~callback:(fun ev -> key_release (GdkEvent.Key.keyval ev))) ;
  area#misc#set_can_focus true ;
  area#misc#grab_focus ()

(* ============================================================================= *)
(* = Creation d'une zone de dessin avec callbacks                              = *)
(* =                                                                           = *)
(* = width       : largeur de la zone                                          = *)
(* = height      : hauteur de la zone                                          = *)
(* = pack_method : ou mettre la zone (ex : hbox#pack)                          = *)
(* = pix_expose  : pix servant au dessin dans la zone                          = *)
(* = mouse_press   : callback click souris (fun event -> ... ; true)           = *)
(* = mouse_move    : callback deplacement                                      = *)
(* = mouse_release : callback bouton relache                                   = *)
(* =                                                                           = *)
(* = En retour, la fonction area_expose permet de forcer l'affichage dans      = *)
(* = la zone de dessin                                                         = *)
(* ============================================================================= *)
let create_draw_area width height pack_method pix_expose
	mouse_press mouse_move mouse_release =
  let (area, area_expose) = create_draw_area_simple width height
	  pack_method pix_expose in
  (* Connexion des eventuels signaux generes par la souris *)
  area_mouse_connect area mouse_press mouse_move mouse_release ;

  (* Renvoie la fonction de reaffichage *)
  area_expose

(* ============================================================================= *)
(* = Boite de selection d'une couleur                                          = *)
(* =                                                                           = *)
(* = update_func = la fonction appelee apres selection. La couleur selectionnee= *)
(* =               lui est alors passee en parametre                           = *)
(* ============================================================================= *)
let select_color update_func =
  let csd = GWindow.color_selection_dialog ~modal:true ~title:"Selection couleur" () in
  ignore(csd#cancel_button#connect#clicked ~callback:csd#destroy);
  ignore(csd#ok_button#connect#clicked ~callback:(fun () ->
	let color = csd#colorsel#color in
	(* Destruction dela fenetre apres selection *)
	csd#destroy () ;
	update_func (`RGB (Color.red color, Color.green color, Color.blue color)))) ;
  csd#show ()

(* ============================================================================= *)
(* = Bouton de couleur permettant la selection d'une autre couleur             = *)
(* =                                                                           = *)
(* = window      = fenetre mere                                                = *)
(* = taille_x    = largeur du rectangle indiquant la couleur                   = *)
(* = taille_y    = hauteur                                                     = *)
(* = color       = couleur initiale                                            = *)
(* = pack_method = ou mettre le bouton                                         = *)
(* = callback    = fonction appelee apres modification de la couleur           = *)
(* ============================================================================= *)
let create_color_selection_button window taille_x taille_y color
	pack_method callback =
  (* Le pixmap contenant la couleur courante *)
  let pm = GDraw.pixmap ~window:window ~width:taille_x ~height:taille_y () in
  (* Mise a jour de la couleur initiale *)
  (pm:GDraw.pixmap)#set_foreground color ;
  (* Dessin avec cette couleur dans le pixmap *)
  pm#rectangle ~filled:true ~x:0 ~y:0 ~width:taille_x ~height:taille_y () ;
  (* Creation d'un bouton contenant ce pixmap *)
  let but = create_pixbutton pm pack_method in
  (* Connexion du callback *)
  but_connect but (fun () -> select_color callback) ;

  but

let create_color_selection_button2 window taille_x taille_y color
	pack_method callback =
  (* Le pixmap contenant la couleur courante *)
  let pm = GDraw.pixmap ~window:window ~width:taille_x ~height:taille_y () in
  (* Mise a jour de la couleur initiale *)
  (pm:GDraw.pixmap)#set_foreground color ;
  (* Dessin avec cette couleur dans le pixmap *)
  pm#rectangle ~filled:true ~x:0 ~y:0 ~width:taille_x ~height:taille_y () ;
  (* Creation d'un bouton contenant ce pixmap *)
  let but = create_pixbutton pm pack_method in
  (* Connexion du callback *)
  but_connect but (fun () -> select_color
	  (fun c -> pm#set_foreground c ;
		pm#rectangle ~filled:true ~x:0 ~y:0 ~width:taille_x ~height:taille_y () ;
		callback c)) ;

  but

(* ============================================================================= *)
(* = Routine interne de creation d'une boite de selection de couleurs          = *)
(* ============================================================================= *)
let scw window colors tooltips update_func vbox destroy_func =
  (* Sauvegarde des valeurs initiales des couleurs en cas d'annulation *)
  let save_colors =
	let f c (_, lst) = c @ (List.map (fun (_, couleur, _) -> !couleur) lst) in
	Array.of_list(List.fold_left f [] colors)
  in

  let taille_x = 40 and taille_y = 10
  and clicked_apply = ref false and application_auto = ref true in

  (vbox:GPack.box)#set_spacing 10 ;
  let (scrolled_window, v) = create_scrolled_box vbox#add in
  let vb = ref v in

  (* Fonction de creation d'une boite contenant un label de titre   *)
  (* et un bouton de selection de couleur dont la couleur est color *)
  let create_boite v title color callback =
	let hbox = create_hbox (v:GPack.box)#pack in
	let _lab = create_label title hbox#pack and
	    but = create_color_selection_button (window:GWindow.window)
	    taille_x taille_y color	(hbox#pack ~from:`END) callback in
	but
  in

  (* Creation des boites de selection de couleurs, pour toutes les frames *)
  let rec creation_liste () =
	(* Destruction de la liste precedente et recreation dans la scrolled_window *)
	vb := change_scrolled_box scrolled_window !vb ;

	List.iter (fun (nom, lst) ->
	  let v = snd (create_vframe nom !vb#pack) in
	  let do_boites (title, couleur, _) =
	    let _b = create_boite v title !couleur
		(fun color -> couleur := color;
		  (* Recreation de toute la liste pour mise a jour de la couleur de la boite *)
		  creation_liste () ;
			  (* Application automatique ? *)
			  if !application_auto then begin
				clicked_apply := true ;	update_func ()
			  end) in
		()
	  in
	  List.iter do_boites lst) colors
  in

  (* Application (i.e redessin par update_func) apres chaque modif ? *)
  let check = GButton.check_button ~label:"Appliquer automatiquement"
	  ~active: !application_auto ~packing:vbox#pack () in
  but_connect check (fun () -> application_auto := check#active) ;

  (* Boutons OK/Annulation, ... *)
  let hbox = create_hom_hbox vbox#pack in
  let but_ok      = create_button "Ok"        hbox#pack and
	  but_apply   = create_button "Appliquer" hbox#pack and
	  but_default = create_button "Defaut"    hbox#pack and
	  but_cancel  = create_button "Annuler"   hbox#pack in

  (* Association de l'aide contextuelle si possible *)
  begin
	match tooltips with
	  None -> ()
	| Some t ->
		add_tooltips t but_ok      "Valide les changements" ;
		add_tooltips t but_apply   "Applique les changements" ;
		add_tooltips t but_default "Couleurs par defaut" ;
		add_tooltips t but_cancel  "Annulation des changements" ;
		add_tooltips t check "Application des qu'une modification est effectuee"
  end ;

  (* Connexion des callbacks des boutons *)
  but_connect but_ok      (fun () ->
	(match destroy_func with None -> () | Some f -> f ());
	if not !application_auto then update_func ()) ;
  but_connect but_apply   (fun () -> clicked_apply := true; update_func()) ;
  but_connect but_default (fun () ->
	(* Valeurs par defaut *)
	let f (_, lst) = List.iter
		(fun (_, couleur, couleur_def) -> couleur := couleur_def) lst in
	List.iter f colors ;
	clicked_apply := true ;
	(* Mise a jour de la liste *)
	creation_liste ();
	(* Si application automatique on redessine *)
	if !application_auto then update_func ()) ;

  but_connect but_cancel  (fun () ->
	(* Restauration des couleurs initiales *)
	let idx = ref 0 in
	let f (_, lst) = List.iter (fun (_, couleur, _) ->
	  couleur := save_colors.(!idx); incr idx) lst in
	List.iter f colors ;
	(match destroy_func with None -> () | Some f -> f ());
	(* Application des parametres anterieurs si necessaire *)
	if !clicked_apply then update_func ()) ;

  (* Premier affichage de la liste des couleurs *)
  creation_liste ()

(* ============================================================================= *)
(* = Widget de selection de plusieurs couleurs                                 = *)
(* =                                                                           = *)
(* = colors      = liste de (nom_frame, [(label, couleur ref, couleur_defaut)])= *)
(* = tooltips    = None ou (Some tooltips)                                     = *)
(* = update_func = la fonction d'application des nouvelles couleurs            = *)
(* = vbox        = ou mettre le widget                                         = *)
(* ============================================================================= *)
let select_colors_widget window colors tooltips update_func vbox =
  scw window colors tooltips update_func vbox None

(* ============================================================================= *)
(* = Boite de selection de plusieurs couleurs                                  = *)
(* =                                                                           = *)
(* = title       = titre de la fenetre                                         = *)
(* = colors      = liste de (nom_frame, [(label, couleur ref, couleur_defaut)])= *)
(* = tooltips    = None ou (Some tooltips)                                     = *)
(* = update_func = la fonction d'application des nouvelles couleurs            = *)
(* ============================================================================= *)
let select_colors title width height colors tooltips update_func =
  (* Creation de la fenetre *)
  let (window, vbox) = create_window title width height in
  scw window colors tooltips update_func vbox
	(Some (fun () -> window#destroy ()));
  window#show ()



(* ============================================================================= *)
(* = Creation d'une zone de selection de valeur simple (sans callback)         = *)
(* =                                                                           = *)
(* = label       = texte du label                                              = *)
(* = lab_width   = taille du label                                             = *)
(* = init_value  = valeur initiale du texte dans la zone d'entree (chaine)     = *)
(* = min_value   = valeur min admissible                                       = *)
(* = max_value   = valeur max admissible                                       = *)
(* = value_width = taille de la zone d'entree                                  = *)
(* = step_incr   = increment lie aux fleches haut/bas                          = *)
(* = page_incr   = increment lie a page up/page down                           = *)
(* = tip         = texte de l'aide contextuelle                                = *)
(* = tooltips    = aide contextuelle                                           = *)
(* = pack_method = ou mettre les widgets                                       = *)
(* ============================================================================= *)
let create_int_spinner_simple label lab_width init_value min_value
	max_value value_width step_incr page_incr tip tooltips pack_method =
  let hbox = create_hbox pack_method in
  let _l = create_sized_label label lab_width hbox#pack 
  and spinner = GEdit.spin_button
      ~adjustment:(GData.adjustment ~value:(float_of_int init_value)
		     ~lower:(float_of_int min_value) ~upper:(float_of_int max_value)
		     ~step_incr:(float_of_int step_incr)
		     ~page_incr:(float_of_int page_incr) ~page_size:0.0 ())
	  ~rate:0. ~digits:0 ~width:value_width () in
  hbox#pack spinner#coerce ;
  if tip <> "" then	add_tooltips tooltips spinner tip ;
  spinner

(* ============================================================================= *)
(* = Connexion d'un callback a un spinner                                      = *)
(* =                                                                           = *)
(* = sp       = le spinner concerne                                            = *)
(* = callback = le callback a associer                                         = *)
(* ============================================================================= *)
let int_spinner_connect sp callback =
  ignore((sp:GEdit.spin_button)#connect#value_changed ~callback:(fun () ->
	try let new_value = sp#value_as_int in callback new_value
	with Failure("int_of_string") -> ()))
  (* Necessaire sous GTK2 pour mettre a jour la valeur quand on la *)
  (* modifie en changeant directement le texte de l'entry...       *)
(*  ignore(sp#connect#changed ~callback:(fun () -> sp#update))*)

(* ============================================================================= *)
(* = Creation d'une zone de selection de valeur avec callback                  = *)
(* =                                                                           = *)
(* = label       = texte du label                                              = *)
(* = lab_width   = taille du label                                             = *)
(* = init_value  = valeur initiale du texte dans la zone d'entree (chaine)     = *)
(* = min_value   = valeur min admissible                                       = *)
(* = max_value   = valeur max admissible                                       = *)
(* = value_width = taille de la zone d'entree                                  = *)
(* = step_incr   = increment lie aux fleches haut/bas                          = *)
(* = page_incr   = increment lie a page up/page down                           = *)
(* = tip         = texte de l'aide contextuelle                                = *)
(* = tooltips    = aide contextuelle                                           = *)
(* = pack_method = ou mettre les widgets                                       = *)
(* = callback    = callback appele lors de la modification de la zone          = *)
(* ============================================================================= *)
let create_int_spinner label lab_width init_value min_value max_value
	value_width step_incr page_incr tip tooltips pack_method callback =
  let spinner = create_int_spinner_simple label lab_width init_value
	  min_value max_value value_width step_incr page_incr tip tooltips
	  pack_method in
  int_spinner_connect spinner callback ;
  spinner

(* ============================================================================= *)
(* = Creation d'un slider de selection d'une valeur entiere                    = *)
(* =                                                                           = *)
(* = init_val    = valeur initiale                                             = *)
(* = min_val     = valeur mini acceptee                                        = *)
(* = max_val     = valeur maxi acceptee                                        = *)
(* = step        = valeur du pas d'incrementation                              = *)
(* = page        = valeur du pas d'incrementation d'une page                   = *)
(* = draw_val    = affichage ou pas de la valeur sur le slider                 = *)
(* = pack_method = ou mettre le widget                                         = *)
(* ============================================================================= *)
let create_vslider_simple init_val min_val max_val step page draw_val
	pack_method =
  let adj = GData.adjustment ~lower:(float_of_int min_val)
	  ~upper:(float_of_int (max_val+10))
	  ~step_incr:(float_of_int step) ~page_incr:(float_of_int page) () in
  let sc = GRange.scale `VERTICAL ~adjustment:adj ~draw_value:draw_val
	  ~digits:0 ~packing:pack_method () in
  adj#set_value (float_of_int init_val) ;
  (adj, sc)

let create_hslider_simple init_val min_val max_val step page draw_val
	pack_method =
  let adj = GData.adjustment ~lower:(float_of_int min_val)
	  ~upper:(float_of_int (max_val+10))
	  ~step_incr:(float_of_int step) ~page_incr:(float_of_int page) () in
  let sc = GRange.scale `HORIZONTAL ~adjustment:adj ~draw_value:draw_val
	  ~digits:0 ~packing:pack_method () in
  adj#set_value (float_of_int init_val) ;
  (adj, sc)

(* ============================================================================= *)
(* = Connexion d'un callback a un slider                                       = *)
(* =                                                                           = *)
(* = slider   = le widget concerne                                             = *)
(* = callback = le callback a associer                                         = *)
(* ============================================================================= *)
let slider_connect slider callback =
  (slider:GData.adjustment)#connect#value_changed ~callback:(fun () ->
	callback (int_of_float slider#value))

(* ============================================================================= *)
(* = Creation d'un slider de selection d'une valeur entiere avec callback      = *)
(* =                                                                           = *)
(* = init_val    = valeur initiale                                             = *)
(* = min_val     = valeur mini acceptee                                        = *)
(* = max_val     = valeur maxi acceptee                                        = *)
(* = step        = valeur du pas d'incrementation                              = *)
(* = page        = valeur du pas d'incrementation d'une page                   = *)
(* = draw_val    = affichage ou pas de la valeur sur le slider                 = *)
(* = pack_method = ou mettre le widget                                         = *)
(* = callback    = le callback a associer                                      = *)
(* ============================================================================= *)
let create_vslider init_val min_val max_val step page draw_val
	pack_method callback =
  let slider = create_vslider_simple init_val min_val max_val
	  step page draw_val pack_method in
  ignore(slider_connect (fst slider) callback) ;
  slider

let create_hslider init_val min_val max_val step page draw_val
	pack_method callback =
  let slider = create_hslider_simple init_val min_val max_val
	  step page draw_val pack_method in
  ignore(slider_connect (fst slider) callback) ;
  slider

let create_float_spinner_simple label lab_width init_value min_value
	max_value value_width nb_digits step_incr page_incr tip tooltips pack_method =
  let hbox = create_hbox pack_method in
  let _l = create_sized_label label lab_width hbox#pack and
	  spinner = GEdit.spin_button
	  ~adjustment:(GData.adjustment ~value:init_value
					 ~lower:min_value ~upper:max_value
					 ~step_incr:step_incr ~page_incr:page_incr
					 ~page_size:0.0 ())
	  ~rate:0. ~digits:nb_digits ~width:value_width () in
  hbox#pack spinner#coerce ;
  if tip <> "" then	add_tooltips tooltips spinner tip ;
  spinner

let float_spinner_connect sp callback =
  ignore((sp:GEdit.spin_button)#connect#value_changed ~callback:(fun () ->
	try let new_value = sp#value in callback new_value ;
	with Failure("float_of_string") -> ()))
  (* Necessaire sous GTK2 pour mettre a jour la valeur quand on la *)
  (* modifie en changeant directement le texte de l'entry...       *)
(*  ignore(sp#connect#changed ~callback:(fun () -> sp#update))*)

let create_float_spinner label lab_width init_value min_value max_value
	value_width nb_digits step_incr page_incr tip tooltips pack_method callback =
  let spinner = create_float_spinner_simple label lab_width init_value
	  min_value max_value value_width nb_digits step_incr page_incr tip tooltips
	  pack_method in
  float_spinner_connect spinner callback ;
  spinner

(* ============================================================================= *)
(* = Creation d'une zone de selection de texte                                 = *)
(* =                                                                           = *)
(* = label       = texte du label                                              = *)
(* = lab_width   = taille du label                                             = *)
(* = init_value  = valeur initiale du texte dans la zone d'entree (chaine)     = *)
(* = value_width = taille de la zone d'entree                                  = *)
(* = tip         = texte de l'aide contextuelle                                = *)
(* = tooltips    = aide contextuelle                                           = *)
(* = pack_method = ou mettre les widgets                                       = *)
(* ============================================================================= *)
let create_text_entry_simple label lab_width init_value value_width
	tip tooltips pack_method =
  let hbox = create_hbox pack_method in
  let lab = create_sized_label label lab_width hbox#pack and
	  entry = GEdit.entry ~text:init_value ~width:value_width ~packing:hbox#pack () in
  if tip <> "" then add_tooltips tooltips entry tip ;
 (lab,  entry)

(* ============================================================================= *)
(* = Attachement d'un callback lorsqu'on appuie sur entree dans l'entry        = *)
(* =                                                                           = *)
(* = entry    = le widget concerne                                             = *)
(* = callback = le callback a appeler                                          = *)
(* ============================================================================= *)
let text_entry_connect entry callback =
  (entry:GEdit.entry)#connect#activate
	~callback:(fun () -> callback (String.uppercase entry#text))

(* ============================================================================= *)
(* = Attachement d'un callback lorsqu'on modifie le texte dans l'entry         = *)
(* =                                                                           = *)
(* = entry    = le widget concerne                                             = *)
(* = callback = le callback a appeler                                          = *)
(* ============================================================================= *)
let text_entry_connect_modify entry callback =
  (entry:GEdit.entry)#connect#changed
	~callback:(fun () -> callback (String.uppercase entry#text))

(* ============================================================================= *)
(* = Creation d'une zone de selection de texte                                 = *)
(* =                                                                           = *)
(* = label       = texte du label                                              = *)
(* = lab_width   = taille du label                                             = *)
(* = init_value  = valeur initiale du texte dans la zone d'entree (chaine)     = *)
(* = value_width = taille de la zone d'entree                                  = *)
(* = tip         = texte de l'aide contextuelle                                = *)
(* = tooltips    = aide contextuelle                                           = *)
(* = pack_method = ou mettre les widgets                                       = *)
(* = callback    = callback appele lors de la modification de la zone          = *)
(* ============================================================================= *)
let create_text_entry label lab_width init_value value_width
	tip tooltips pack_method callback =
  let (lab,  entry) = create_text_entry_simple label lab_width
	  init_value value_width tip tooltips pack_method in
  ignore(text_entry_connect_modify entry callback) ;
  (lab, entry)

(* Type pour les fenetres enregistrees *)
type registered_win = {reg_win_id : int ;
								 mutable reg_win_handle : GWindow.window option ;
								 reg_win_build : unit -> GWindow.window}

(* Compteur pour les fenetres enregistrees *)
let register_count = ref 0
(* Tableau contenant les fenetres enregistrees *)
let registered_windows = ref ([||] : registered_win array)

(* Exception levee lors de l'appel a un numero de fenetre non enregistree *)
exception GTK_TOOLS_UNREGISTERED_WINDOW of int

(* ============================================================================= *)
(* = Enregistrement d'une fenetre                                              = *)
(* =                                                                           = *)
(* = build_window_func = la fonction de creation de la fenetre. Cette fonction = *)
(* =                     doit renvoyer un objet du type GWindow.window         = *)
(* =                                                                           = *)
(* = Renvoie un identifiant (entier) pour cette fenetre                        = *)
(* ============================================================================= *)
let register_window build_window_func =
  let this_win_num = !register_count in
  incr register_count ;

  registered_windows := Array.append !registered_windows
	  [|{reg_win_id = this_win_num; reg_win_handle = None;
		 reg_win_build = build_window_func}|] ;

  this_win_num

(* ============================================================================= *)
(* = Appel d'une fenetre enregistree                                           = *)
(* =                                                                           = *)
(* = id = identifiant de la fenetre (obtenu par register_window)     = *)
(* ============================================================================= *)
let show_registered_window id =
  if id < 0 or id >= !register_count then
	raise (GTK_TOOLS_UNREGISTERED_WINDOW id) ;

  let w = !registered_windows.(id) in
  match w.reg_win_handle with
	None -> (* La fenetre n'avait pas encore ete creee, on le fait *)
	  (* Appel de la fonction de creation de la fenetre *)
	  let win = w.reg_win_build () in
	  ignore(win#connect#destroy ~callback:(fun _ -> w.reg_win_handle <- None)) ;
	  w.reg_win_handle <- Some win ;
	  win#show ()

  |	Some w -> (* La fenetre etait deja creee, on la met en avant-plan *)
	  window_set_front w

(* ============================================================================= *)
(* = Cache (detruit) une fenetre enregistree                                   = *)
(* =                                                                           = *)
(* = id = identifiant de la fenetre (obtenu par register_window)     = *)
(* ============================================================================= *)
let hide_registered_window id =
  if id < 0 or id >= !register_count then
	raise (GTK_TOOLS_UNREGISTERED_WINDOW id) ;

  let w = !registered_windows.(id) in
  match w.reg_win_handle with
	None -> ()
  |	Some win -> win#destroy () ; w.reg_win_handle <- None

(* ============================================================================= *)
(* = Renvoie une fenetre enregistree                                           = *)
(* =                                                                           = *)
(* = id = identifiant de la fenetre (obtenu par register_window)     = *)
(* ============================================================================= *)
let get_registered_window id =
  if id < 0 or id >= !register_count then
	raise (GTK_TOOLS_UNREGISTERED_WINDOW id) ;

  (!registered_windows.(id)).reg_win_handle

(* ============================================================================= *)
(* = Fonction effectuant le scrolling d'un adjustment (scrollbar)              = *)
(* ============================================================================= *)
let scroll_adjustment adj scroll_up delta =
  (* Valeur extremes que peut prendre l'adjustment *)
  let val_min = adj#lower and val_max = adj#upper -. adj#page_size in
  let current_value = adj#value in
  let new_value =
	if scroll_up then max val_min (current_value-.delta)
	else min val_max (current_value+.delta)
  in
  if new_value<>current_value then adj#set_value new_value

(* ============================================================================= *)
(* = Connexion d'un scroll aux mouvements de la molette de la souris           = *)
(* ============================================================================= *)
let connect_mouse_wheel_scroll widget sb delta =
  let check_wheel ev =
	match GdkEvent.Scroll.direction ev with
	  `UP   -> scroll_adjustment sb#adjustment true delta; true
	| `DOWN -> scroll_adjustment sb#adjustment false delta; true
	| _ -> false
  in

  (* Connexion a present identique pour Unix et Windows... *)
  ignore(widget#event#connect#scroll ~callback:check_wheel)

(* ============================================================================= *)
(* = Creation d'un objet clist                                                 = *)
(* =                                                                           = *)
(* = lst_titles      = liste des titres de chaque colonne                      = *)
(* = sortable_titles = tri possible ou pas en cliquant sur les titres ?        = *)
(* = first_sort      = indique quelle est la colonne triee par defaut          = *)
(* = pack_method     = ou mettre la liste                                      = *)
(* ============================================================================= *)
let create_list lst_titles (sortable_titles, first_sort) pack_method =
  let hb = create_hbox pack_method in
  let sb = GRange.scrollbar `VERTICAL ~packing:(hb#pack ~from:`END) () in
  let clist = GList.clist ~titles:lst_titles ~shadow_type:`OUT
	  ~packing:hb#add ~vadjustment:sb#adjustment () in

  if sortable_titles then begin
	(* Sens de tri de chaque colonne *)
	let sens_tri = Array.mapi (fun i _ -> i <> first_sort)
		(Array.of_list lst_titles) in
	let select_column column =
	  let dir = if sens_tri.(column) then `ASCENDING else `DESCENDING	in
	  clist#set_sort ~column:column ~dir:dir () ;
	  clist#sort () ;
	  (* Inversion du sens de tri pour la prochaine fois *)
	  sens_tri.(column) <- not sens_tri.(column)
	in
	ignore(clist#connect#click_column ~callback:(fun col -> select_column col)) ;
  end ;

  (* Autorisation du defilement avec la molette de la souris *)
  prerr_endline "TODO (Gtk_tools): connect_mouse_wheel_scroll clist sb 15. ;";
  clist

(* ============================================================================= *)
(* = Creation d'un objet clist avec scroll horizontal                          = *)
(* =                                                                           = *)
(* = lst_titles      = liste des titres de chaque colonne                      = *)
(* = sortable_titles = tri possible ou pas en cliquant sur les titres ?        = *)
(* = first_sort      = indique quelle est la colonne triee par defaut          = *)
(* = pack_method     = ou mettre la liste                                      = *)
(* ============================================================================= *)
let create_list_with_hor_scroll lst_titles
	(sortable_titles, first_sort) pack_method =
  let vb = create_vbox pack_method in
  let hb = create_hbox vb#add in
  let sb = GRange.scrollbar `VERTICAL ~packing:(hb#pack ~from:`END) () in
  let sb2 = GRange.scrollbar `HORIZONTAL ~packing:(vb#pack ~from:`END) () in
  let clist = GList.clist ~titles:lst_titles ~shadow_type:`OUT
	  ~packing:hb#add ~vadjustment:sb#adjustment ~hadjustment:sb2#adjustment () in
  if sortable_titles then begin
	(* Sens de tri de chaque colonne *)
	let sens_tri = Array.mapi (fun i _ -> i <> first_sort)
		(Array.of_list lst_titles) in
	let select_column column =
	  let dir = if sens_tri.(column) then `ASCENDING else `DESCENDING	in
	  clist#set_sort ~column:column ~dir:dir () ;
	  clist#sort () ;
	  (* Inversion du sens de tri pour la prochaine fois *)
	  sens_tri.(column) <- not sens_tri.(column)
	in
	ignore(clist#connect#click_column ~callback:(fun col -> select_column col)) ;
  end ;

  (* Autorisation du defilement avec la molette de la souris *)
  prerr_endline "TODO (Gtk_tools): connect_mouse_wheel_scroll clist sb 15.;";
  clist

(* ============================================================================= *)
(* = Ajout de callbacks a un objet clist                                       = *)
(* =                                                                           = *)
(* = clist                  = la liste                                         = *)
(* = callback_select        = callback de selection d'une ligne                = *)
(* = callback_deselect      = callback de deselection d'une ligne              = *)
(* = callback_select_column = callback de selection d'un titre de colonne      = *)
(* ============================================================================= *)
let list_connect clist
	callback_select callback_deselect callback_select_column =
  (* Selection d'un element *)
  (match callback_select with
	None -> ()
  | Some callback ->
	  ignore((clist:string GList.clist)#connect#select_row ~callback:
			   (fun ~row ~column ~event -> callback row column))) ;

  (* Deselection d'un element *)
  (match callback_deselect with
	None -> ()
  | Some callback ->
	  ignore(clist#connect#unselect_row ~callback:
			   (fun ~row ~column ~event -> callback row column))) ;

  (* Click sur un titre *)
  (match callback_select_column with
	None -> ()
  | Some callback ->
	  ignore(clist#connect#click_column ~callback:(fun col -> callback col)))

(* ============================================================================= *)
(* = Ajout de callbacks a un objet clist avec test du double click             = *)
(* =                                                                           = *)
(* = clist                  = la liste                                         = *)
(* = callback_select        = callback de selection d'une ligne                = *)
(* = callback_deselect      = callback de deselection d'une ligne              = *)
(* = callback_select_column = callback de selection d'un titre de colonne      = *)
(* ============================================================================= *)
let list_connect_check_dbl_click clist
	callback_select callback_deselect callback_select_column =
  let test_dbl_click event =
	 match event with None -> false | Some ev -> check_dbl_click ev
  in

  (* Selection d'un element *)
  (match callback_select with
	None -> ()
  | Some callback ->
	  ignore((clist:string GList.clist)#connect#select_row ~callback:
			   (fun ~row ~column ~event -> callback row column
				   (test_dbl_click event)))) ;

  (* Deselection d'un element *)
  (match callback_deselect with
	None -> ()
  | Some callback ->
	  ignore(clist#connect#unselect_row ~callback:
			   (fun ~row ~column ~event -> callback row column
				   (test_dbl_click event)))) ;

  (* Click sur un titre *)
  (match callback_select_column with
	None -> ()
  | Some callback ->
	  ignore(clist#connect#click_column ~callback:(fun col -> callback col)))

(* ============================================================================= *)
(* = Ajout de callbacks d'appui sur une touche dans une clist                  = *)
(* =                                                                           = *)
(* = clist         = la liste                                                  = *)
(* = callback_up   = callback appui fleche haut                                = *)
(* = callback_down = callback appui fleche bas                                 = *)
(* ============================================================================= *)
let list_connect_up_down_keys clist callback_up callback_down =
  let key_press key =
	if key = _Down then (callback_down (); true)
	else if key = _Up then (callback_up (); true)
	else false
  in
  ignore((clist:string GList.clist)#event#connect#key_press
		   ~callback:(fun ev -> key_press (GdkEvent.Key.keyval ev)))

(* ============================================================================= *)
(* = Creation d'une liste avec titres+largeurs colonnes+label pour nb d'elts   = *)
(* =                                                                           = *)
(* = title_sizes = liste des (nom, taille) des colonnes                        = *)
(* = pack_method = ou mettre le widget                                         = *)
(* ============================================================================= *)
let create_managed_list titles_sizes pack_method =
  let vb = create_vbox pack_method in
  let lab = create_label "" vb#pack in

  (* Creation de la liste *)
  let titles = List.map fst titles_sizes in
  let lst = create_list titles (true, 0) vb#add in

  let column_sizes = Array.of_list (List.map snd titles_sizes) in
  (* Mise a jour de la taille des colonnes *)
  Array.iteri (fun i size -> lst#set_column i ~width:size) column_sizes ;

  (lst, lab)

(* ============================================================================= *)
(* = Mise a jour des tailles des colonnes d'une liste                          = *)
(* ============================================================================= *)
let set_columns_sizes (lst: string GList.clist) (sizes: int list) =
  let i = ref 0 in
  List.iter (fun width -> lst#set_column !i ~width ; incr i) sizes

(* ============================================================================= *)
(* = Connection de callback a la liste precedente                              = *)
(* =                                                                           = *)
(* = (lst, lab)          = la liste et le label cree par la fct precedente     = *)
(* = item_column         = colonne ou se trouve l'item de reference            = *)
(* = selection_callback  = callback appele lors de la selection d'un element   = *)
(* = (name, female, cap) = regle d'affichage du label du nombre d'elements     = *)
(* ============================================================================= *)
let connect_managed_list (lst, lab) item_column selection_callback
	(name, female, cap) =
  let eval_string base_string n female first_char_upper =
	let get_upper_and_female str =
	  let str = if female then str^"e" else str in
	  if first_char_upper then String.capitalize str
	  else str
	in
	match n with
	  0 -> let s = get_upper_and_female "aucun" in s ^ " " ^ base_string
	| 1 -> let s = get_upper_and_female "un" in s ^ " " ^ base_string
	| _ -> if String.get base_string (String.length base_string -1) = 'x' then
		Printf.sprintf "%d %s" n base_string
	else Printf.sprintf "%d %ss" n base_string
  in

  let last_row = ref (-1) and id_selected = ref "" in
  let reset_selection () = last_row := (-1) ; id_selected := "" in

  (* Selections dans la liste *)
  let select_func selection_type row column dbl_click =
	if selection_type then begin
	  id_selected := lst#cell_text row item_column ; last_row := row
	end else reset_selection () ;
	selection_callback !id_selected (row, column, dbl_click) selection_type
  in

  (* Connexion des callbacks de la liste *)
  list_connect_check_dbl_click lst
	(Some (select_func true)) (Some (select_func false)) None ;

  (* Ajout de cette fonction pour GTK2. En GTK1, ca scrolle tout seul *)
  (* quand necessaire, pas en GTK2... *)
  let scroll_if_needed () =
	if !last_row<>(-1) then
	  match lst#row_is_visible !last_row with
		`NONE | `PARTIAL -> lst#moveto !last_row item_column
	  | `FULL -> ()
  in

  let nb_elts = ref 0 in
  (* Callback permettant de naviguer dans la liste avec les fleches haut/bas *)
  let key_up () =
	if !last_row<> -1 && !last_row <> 0 then begin
	  lst#select (!last_row-1) item_column ;
	  scroll_if_needed ()
	end
  in
  let key_down () =
	if !last_row<> -1 && !last_row <> (!nb_elts-1) then begin
	  lst#select (!last_row+1) item_column ;
	  scroll_if_needed ()
	end
  in
  list_connect_up_down_keys lst key_up key_down ;

  let fill_list select_id lst_items =
	lst#clear () ; lst#freeze () ;
	reset_selection () ;
	let row = ref 0 and selected_row = ref (-1) in
	List.iter (fun l ->
	  ignore(lst#append l) ;
	  (* Est-ce la ligne de l'element a selectionner ? *)
	  if List.nth l item_column = select_id then selected_row := !row ;
	  incr row) lst_items ;

	nb_elts := !row ;
	(lab:GMisc.label)#set_text (eval_string name !nb_elts female cap) ;
	lst#thaw () ;

	(* Selection de l'element selectionne si c'est demande *)
	if !selected_row <> -1 then lst#select !selected_row item_column
  in

  fill_list

(* Type Lat/Lon en degres ou degres/minutes/secondes *)
type t_val =
	G_LAT    (* Latitude en degres flottants      *)
  | G_LON    (* Longitude en degres flottants     *)
  | G_LAT_D  (* Latitude : degres                 *)
  | G_LAT_M  (* Latitude : minutes                *)
  | G_LAT_S  (* Latitude : secondes               *)
  | G_LAT_NS (* Latitude : orientation Nord/Sud   *)
  | G_LON_D  (* Longitude : degres                *)
  | G_LON_M  (* Longitude : minutes               *)
  | G_LON_S  (* Longitude : secondes              *)
  | G_LON_EW (* Longitude : orientation Est/Ouest *)

(* Type contenant un widget de selection de coordonnees lat/lon *)
type latlon = {latlon_lat_val         : float ref ;
						 latlon_lon_val         : float ref ;
						 latlon_update          : float -> float -> unit ;
					     latlon_change_callback : (unit -> unit) option ref}

(* ============================================================================= *)
(* = Creation d'un widget de selection de coordonnees lat/lon                  = *)
(* =                                                                           = *)
(* = (lat_in, lon_in) = position initiale en degres                            = *)
(* = pack_method = ou mettre le widget                                         = *)
(* = tooltips    = aide contextuelle                                           = *)
(* =                                                                           = *)
(* = Renvoie un widget de type latlon                                = *)
(* ============================================================================= *)
let create_latlon_selection (lat_in, lon_in) pack_method tooltips =
  let degminsec_of_pos d =
	let (d, dd, signe) =
	  if d < 0.0 then (-.d, int_of_float (-.d), (-1.0)) else (d, int_of_float d, 1.0)
	in
	let reste = d-.(float_of_int dd) in
	let mm = int_of_float (reste*.60.0) in
	let ss = (reste-.(float_of_int mm)/.60.0)*.3600.0 in
	(* Evite d'avoir 49.3 -> 49 17 60.00 *)
	let (mm, ss) = if ss > 59.9999 then (mm+1, 0.0) else (mm, ss) in
	(dd, mm, ss, signe) in

  let pos_of_degminsec (d, m, s, signe) =
	((float_of_int d)+.(float_of_int m)/.60.0+.s/.3600.0) *. signe in

  (* Fonction de creation des menus de choix N/S et E/W *)
  let menu_NS_EW options signe pack_method =
	let menu = GMenu.menu () in
	let l = List.map (fun op ->
	  (GMenu.menu_item ~label:op ~packing:menu#append (), op)) options in
	let optionmenu = GMenu.option_menu ~packing:pack_method () in
	optionmenu#set_menu menu ;

	(* Si signe est negatif alors S ou W. On selectionne par defaut le menu *)
	(* correspondant *)
	if signe<0.0 then optionmenu#set_history 1 ;

	(optionmenu, l)
  in

  (* Initialisation des variables *)
  let lat = ref lat_in and lon = ref lon_in in
  let (a, b, c, s) = degminsec_of_pos lat_in in
  let lat_d = ref a and lat_m = ref b and lat_s = ref c and lat_signe = ref s in
  let (a, b, c, s) = degminsec_of_pos lon_in in
  let lon_d = ref a and lon_m = ref b and lon_s = ref c and lon_signe = ref s in

  (* Creation des widgets *)
  let box = create_spaced_hbox pack_method in

  (* Latitudes *)
  let b = snd (create_vframe "Latitude" box#pack) in
  let e_lat = create_float_spinner_simple "" 0
	  !lat (-90.0) 90.0 210 4 0.1 1.0
	  "Latitude en degres (flottant)" tooltips b#pack and
	  hb = GPack.hbox ~packing:b#pack ~spacing:5 ~border_width:5 () in
  let e_lat_d = create_int_spinner_simple "" 0 !lat_d 0 90 40 1 5
	  "Latitude : degres" tooltips hb#pack and
	  e_lat_m = create_int_spinner_simple "" 0 !lat_m 0 59 40 1 5
	  "Latitude : minutes" tooltips hb#pack and
	  e_lat_s = create_float_spinner_simple "" 0
	  !lat_s 0.0 59.99 55 2 0.1 1.0
	  "Latitude : secondes" tooltips hb#pack and
	  (o_lat, m_lat) = menu_NS_EW ["N"; "S"] !lat_signe hb#pack in

  (* Longitudes *)
  let b = snd (create_vframe "Longitude" box#pack) in
  let e_lon = create_float_spinner_simple "" 0
	  !lon (-180.0) 180.0 210 4 0.1 1.0
	  "Longitude en degres (flottant)" tooltips b#pack and
	  hb = GPack.hbox ~packing:b#pack ~spacing:5 ~border_width:5 () in
  let e_lon_d = create_int_spinner_simple "" 0 !lon_d 0 180 40 1 5
	  "Longitude : degres" tooltips hb#pack and
	  e_lon_m = create_int_spinner_simple "" 0 !lon_m 0 59 40 1 5
	  "Longitude : minutes" tooltips hb#pack and
	  e_lon_s = create_float_spinner_simple "" 0
	  !lon_s 0.0 59.99 55 2 0.1 1.0
	  "Longitude : secondes" tooltips hb#pack and
	  (o_lon, m_lon) = menu_NS_EW ["E"; "W"] !lon_signe hb#pack in

  (* Callbacks de modification *)
  let updating = ref false in

  (* Fonctions de coherence des differents affichages *)
  let update_from_latlon () =
	let (a, b, c, signe) = degminsec_of_pos !lat in
	lat_d := a; lat_m := b; lat_s := c ; lat_signe := signe ;

	e_lat_d#set_value (float_of_int !lat_d) ;
	e_lat_m#set_value (float_of_int !lat_m) ;
	e_lat_s#set_value !lat_s ;

	let m = if signe > 0.0 then 0 else 1 in
	o_lat#set_history m ;
	let (a, b, c, signe) = degminsec_of_pos !lon in
	lon_d := a; lon_m := b; lon_s := c ; lon_signe := signe ;

	e_lon_d#set_value (float_of_int !lon_d) ;
	e_lon_m#set_value (float_of_int !lon_m) ;
	e_lon_s#set_value !lon_s ;

	let m = if signe > 0.0 then 0 else 1 in
	o_lon#set_history m
  in
  let update_from_latlon_dms () =
	lat := pos_of_degminsec (!lat_d, !lat_m, !lat_s, !lat_signe) ;
	lon := pos_of_degminsec (!lon_d, !lon_m, !lon_s, !lon_signe) ;
	e_lat#set_value !lat ; e_lon#set_value ! lon ;
  in

  let change_callback = ref None in

  let update type_valeur t =
	if not !updating then begin
	  updating := true ;
	  begin
		match type_valeur with
		  G_LAT    -> lat := t ; update_from_latlon ()
		| G_LON    -> lon := t ; update_from_latlon ()
		| G_LAT_D  -> lat_d := int_of_float t ; update_from_latlon_dms ()
		| G_LAT_M  -> lat_m := int_of_float t ; update_from_latlon_dms ()
		| G_LAT_S  -> lat_s := t ; update_from_latlon_dms ()
		| G_LON_D  -> lon_d := int_of_float t ; update_from_latlon_dms ()
		| G_LON_M  -> lon_m := int_of_float t ; update_from_latlon_dms ()
		| G_LON_S  -> lon_s := t ; update_from_latlon_dms ()
		| G_LAT_NS -> lat_signe := t ; update_from_latlon_dms ()
		| G_LON_EW -> lon_signe := t ; update_from_latlon_dms ()
	  end ;
	  updating := false ;
	  match !change_callback with None -> () | Some callback -> callback ()
	end
  in

  let connect_sp spinner t =
    spinner#connect#value_changed ~callback:(fun () ->
	  try let new_value = spinner#value in update t new_value
	  with Failure("float_of_string") -> ())
  in

  ignore(connect_sp e_lat G_LAT)     ; ignore(connect_sp e_lon G_LON) ;
  ignore(connect_sp e_lat_d G_LAT_D) ; ignore(connect_sp e_lat_m G_LAT_M) ;
  ignore(connect_sp e_lat_s G_LAT_S) ; ignore(connect_sp e_lon_d G_LON_D) ;
  ignore(connect_sp e_lon_m G_LON_M) ; ignore(connect_sp e_lon_s G_LON_S) ;

  let connect_menu_NS_EW l =
	List.iter (fun (menuitem, op) ->
	  ignore(menuitem#connect#activate ~callback:(fun () ->
		if op = "N" then update G_LAT_NS 1.0 ;
		if op = "S" then update G_LAT_NS (-1.0) ;
		if op = "E" then update G_LON_EW 1.0 ;
		if op = "W" then update G_LON_EW (-1.0)))) l
  in

  connect_menu_NS_EW m_lat ;
  connect_menu_NS_EW m_lon ;

  {latlon_lat_val = lat; latlon_lon_val = lon;
   latlon_update  = (fun lat lon -> update G_LAT lat; update G_LON lon;
	 update G_LAT_D (float_of_int !lat_d); update G_LON_D (float_of_int !lon_d)) ;
   latlon_change_callback = change_callback}

(* ============================================================================= *)
(* = Mise a jour des valeurs dans un widget de selection de coordonnees lat/lon= *)
(* =                                                                           = *)
(* = latlon_widget = le widget de selection de coordonnees lat/lon             = *)
(* = new_lat       = nouvelle latitude (en degres)                             = *)
(* = new_lon       = nouvelle longitude (en degres)                            = *)
(* ============================================================================= *)
let update_latlon_selection latlon_widget new_lat new_lon =
  latlon_widget.latlon_update new_lat new_lon

(* ============================================================================= *)
(* = Renvoie les coordonnees choisies dans un widget lat/lon                   = *)
(* =                                                                           = *)
(* = latlon_widget = le widget de selection de coordonnees lat/lon             = *)
(* =                                                                           = *)
(* = Renvoie une paire contenant la latitude et la longitude en degres         = *)
(* ============================================================================= *)
let latlon_selection_get latlon_widget =
  (!(latlon_widget.latlon_lat_val), !(latlon_widget.latlon_lon_val))

(* ============================================================================= *)
(* = Connecte un callback appele lors d'une modif dans le widget lat/lon       = *)
(* =                                                                           = *)
(* = latlon_widget = le widget de selection de coordonnees lat/lon             = *)
(* = callback      = le callback                                               = *)
(* ============================================================================= *)
let latlon_selection_change latlon_widget callback =
  latlon_widget.latlon_change_callback := Some callback

(* Numero de la fenetre de log enregistree *)
let id_log_win = ref (-1)

(* Niveau de verbose dans la fenetre de log *)
let log_verbose_level = ref 100

(* Zone de texte de la fenetre du log quand elle existe *)
let log_wid = ref None

(* Exception levee lors de l'ajout de texte dans la fenetre de log alors que
   celle-ci n'a pas encore ete creee *)
exception GTK_TOOLS_NO_LOG_WIN

(* ============================================================================= *)
(* = Creation d'une zone de texte                                              = *)
(* =                                                                           = *)
(* = editable         = zone editable par l'utilisateur                        = *)
(* = with_vert_scroll = presence d'une barre de defilement verticale           = *)
(* = with_hor_scroll  = presence d'une barre de defilement horizontale         = *)
(* = pack_method      = ou mettre le widget                                    = *)
(* ============================================================================= *)
let create_text_edit editable with_vert_scroll with_hor_scroll
	pack_method =
  let hpolicy = if with_hor_scroll then `ALWAYS else `NEVER
  and vpolicy = if with_vert_scroll then `ALWAYS else `NEVER in
  let text = GText.view ~editable () in
  let sw = GBin.scrolled_window ~packing:pack_method ~hpolicy ~vpolicy () in
  sw#add text#coerce;
  text

(* ============================================================================= *)
(* = Efface une zone editable                                                  = *)
(* ============================================================================= *)
let text_edit_clear edit =
  let (start,stop) = edit#buffer#bounds in edit#buffer#delete ~start ~stop

(* ============================================================================= *)
(* = Renvoie le texte d'une zone editable                                      = *)
(* ============================================================================= *)
let text_edit_get_text edit = (edit:GText.view)#buffer#get_text ()

(* ============================================================================= *)
(* = Renvoie le texte d'une zone editable sous la forme d'une liste de chaines = *)
(* = de caracteres correspondant aux differentes lignes                        = *)
(* ============================================================================= *)
let text_edit_get_lines edit =
  let text = text_edit_get_text edit in
  let l = split2 '\n' text in
  (* Supprime la derniere ligne vide au besoin *)
  if l<>[] && List.hd (List.rev l) = "" then List.rev (List.tl (List.rev l))
  else l

(* ============================================================================= *)
(* = Met a jour le texte d'une zone editable                                   = *)
(* ============================================================================= *)
let text_edit_set_text_list edit text_lst =
  List.iter (fun s -> (edit:GText.view)#buffer#insert (s^"\n")) text_lst

let text_edit_set_text edit text = (edit:GText.view)#buffer#insert text

(* ============================================================================= *)
(* = Fonction interne : renvoie le widget text du log                          = *)
(* ============================================================================= *)
let get_log_wid () =
  match !log_wid with None -> raise GTK_TOOLS_NO_LOG_WIN | Some w -> w

(* ============================================================================= *)
(* = Ajout d'un texte dans le log                                              = *)
(* =                                                                           = *)
(* = text  = texte du message a afficher                                       = *)
(* = level = niveau de priorite du message                                     = *)
(* ============================================================================= *)
let add_log text level =
  if level < !log_verbose_level then begin
	let zone = get_log_wid () and
		time = timer_string_of_time (timer_get_time ()) in
	(zone:GText.view)#buffer#insert (time ^ " : " ^ text)
  end

(* ============================================================================= *)
(* = Ajout d'un texte avec couleur dans le log                                 = *)
(* =                                                                           = *)
(* = text  = texte du message a afficher                                       = *)
(* = level = niveau de priorite du message                                     = *)
(* = color = la couleur a utiliser                                             = *)
(* ============================================================================= *)
let add_log_with_color text level color =
  if level < !log_verbose_level then begin
	let zone = get_log_wid () and
		time = timer_string_of_time (timer_get_time ()) in
	ignore ((zone:GText.view)#buffer#create_tag ~name:"color"
	  [`FOREGROUND_GDK (GDraw.color color)]);
	zone#buffer#insert ~tag_names:["color"] (time ^ " : " ^ text)
  end

(* ============================================================================= *)
(* = Effacement du contenu du log                                              = *)
(* ============================================================================= *)
let clear_log () = text_edit_clear (get_log_wid ())

(* ============================================================================= *)
(* = Creation de la fenetre de log                                             = *)
(* =                                                                           = *)
(* = tooltips = systeme d'aide contextuelle                                    = *)
(* ============================================================================= *)
let create_log tooltips =
  let rec in_build_log tooltips =
	let (win, box) = create_window "Log" 400 300 in

	let log = create_text_edit false true true box#add in
	log_wid := Some log ;

	let bbox = create_bbox box#pack in
	let but_ok    = create_button "OK"      bbox#add and
		but_clear = create_button "Effacer" bbox#add and
		but_save  = create_button "Sauver"  bbox#add in
	add_tooltips tooltips but_ok "Fermer la fenetre de log" ;
	add_tooltips tooltips but_clear
	  "Effacer le contenu de la fenetre de log" ;
	add_tooltips tooltips but_save
	  "Sauver le contenu de la fenetre de log" ;
	but_connect but_clear (fun () -> clear_log ()) ;
	but_connect but_ok (fun () ->
	  hide_registered_window !id_log_win) ;
	but_connect but_save (fun () ->
	  open_file_dlg "Sauvegarde du log"
		(fun filename ->
			if log#buffer#char_count>0 then begin
			  let data = text_edit_get_text log in
			  let c = open_out filename in
			  Printf.fprintf c "%s" data ;
			  close_out c
			end) None "log.txt" true) ;

	(* Pour que le bouton de destruction de la fenetre ne fasse *)
	(* que la cacher et non la detruire reellement              *)
	ignore(win#connect#destroy ~callback:(fun () ->
	  id_log_win := register_window (fun () -> in_build_log tooltips))) ;

	win
  in
  id_log_win := register_window (fun () -> in_build_log tooltips) ;

  let win = in_build_log tooltips in
  let w = !registered_windows.(!id_log_win) in
  ignore(win#connect#destroy ~callback:(fun _ -> w.reg_win_handle <- None)) ;
  w.reg_win_handle <- Some win

(* ============================================================================= *)
(* = Affichage de la fenetre de log                                            = *)
(* =                                                                           = *)
(* = tooltips = systeme d'aide contextuelle                                    = *)
(* ============================================================================= *)
let show_log () =
  if !id_log_win <> -1 then	show_registered_window !id_log_win

(* ============================================================================= *)
(* = Cache la fenetre de log                                                   = *)
(* ============================================================================= *)
let hide_log () = hide_registered_window !id_log_win

(* ============================================================================= *)
(* = Mise a jour du niveau d'affichage des messages dans le log                = *)
(* =                                                                           = *)
(* = level = le niveau de priorite                                             = *)
(* ============================================================================= *)
let set_log_verbose_level level = log_verbose_level := level

(* ============================================================================= *)
(* = Affichage du contenu d'un fichier dans une fenetre                        = *)
(* =                                                                           = *)
(* = filename = le fichier a afficher                                          = *)
(* = title    = titre de la fenetre                                            = *)
(* = tooltips = systeme d'aide contextuelle                                    = *)
(* ============================================================================= *)
let display_file filename title width height tooltips font =
  let (win, vbox) = create_window title width height in
  let text = create_text_edit false true true vbox#add in

  (* Tag pour mettre le texte en rouge *)
  ignore (text#buffer#create_tag ~name:"red_foreground" [`FOREGROUND "red"]);

  (* Mise a jour de la fonte si necessaire *)
  (match font with
	None -> ()
  | Some fontname ->
	  let font = Pango.Font.from_string fontname in
	  ignore(text#misc#modify_font font)) ;

  let bbox = create_bbox vbox#pack in
  let but_ok = create_button "OK" bbox#add in
  add_tooltips tooltips but_ok "Fermer la fenetre" ;
  but_connect but_ok (fun () -> win#destroy ()) ;

  (try
	let c = open_compress filename in
	(try
	  while true do	text#buffer#insert ((input_line c)^"\n") done ;
	  text#buffer#insert ~tag_names:["red_foreground"]
		(Printf.sprintf "Erreur de lecture de %s\n" filename)
	with End_of_file -> close_compress filename c)
  with _ ->
	text#buffer#insert  ~tag_names:["red_foreground"]
	  (Printf.sprintf "Erreur d'ouverture de %s\n" filename)) ;

  win#show ()

(* ============================================================================= *)
(* = Creation d'une barre de progression dans une fenetre externe              = *)
(* =                                                                           = *)
(* = nb_blocks         = nombre de subdivisions dans la barre                  = *)
(* = title             = titre de la fenetre                                   = *)
(* =                                                                           = *)
(* = En sortie est renvoyee la fonction de mise a jour de la barre. Cette      = *)
(* = fonction prend en parametre un flottant entre 0.0 et 1.0.                 = *)
(* = Lorsque ce flottant vaut 1.0, la fenetre est detruite                     = *)
(* ============================================================================= *)
let create_progress_bar_win title =
  let window = GWindow.window ~title:title ~border_width:10 ~width:200 () in
  let pbar = GRange.progress_bar ~packing:window#add () in
(* GTK2 AAA GRange.progress_bar ~bar_style:`DISCRETE ~discrete_blocks:nb_blocks ()
      ~packing:window#add in*)
  let update_func pct =
	pbar#set_fraction pct ;
	(* Destruction de la fenetre si pct = 1.0 *)
	if pct = 1.0 then window#destroy () ;

	(* Force la mise a jour pour voir la barre progresser *)
	force_update_interface () in
  window#show ();

  (* Renvoie la fonction de mise a jour *)
  update_func

(* ============================================================================= *)
(* = Creation d'une barre de progression                                       = *)
(* =                                                                           = *)
(* = pack_method = ou mettre la barre                                          = *)
(* =                                                                           = *)
(* = Renvoie la fonction de mise a jour (valeur entre 0.0 et 1.0)              = *)
(* ============================================================================= *)
let create_progress_bar pack_method =
  let pbar = GRange.progress_bar ~packing:pack_method () in
  let current_progress = ref "" in
  let update_progress p =
	let pp = Printf.sprintf "%.0f%%" (100.*.p) in
	(* Comme ca on n'affiche que tous les 1%. Sinon c'est tres lent avec GTK2 *)
	if !current_progress<>pp then begin
	  pbar#set_text pp ;
	  pbar#set_fraction p ; force_update_interface () ;
	  current_progress:=pp
	end
  in
  update_progress

(* Operateurs de comparaison *)
type t_ops_compare = T_EQ | T_L | T_LEQ | T_G | T_GEQ

(* ============================================================================= *)
(* = Creation d'un widget de selection d'un operateur de comparaison           = *)
(* =                                                                           = *)
(* = variable          = variable contenant l'operateur selectionne (ref)      = *)
(* = callback_modified = callback appele lorsque la valeur est modifiee (opt)  = *)
(* = pack_method       = ou mettre le widget                                   = *)
(* ============================================================================= *)
let create_ops_compare variable callback_modified pack_method =
  let string_of_t_op t =
	match t with
	  T_EQ  -> "="
	| T_L   -> "<"
	| T_LEQ -> "<="
	| T_G   -> ">"
	| T_GEQ -> ">="
  in

  let lst_ops =
	List.map (fun t -> (string_of_t_op t, t)) [T_EQ; T_L; T_LEQ; T_G; T_GEQ] in
  let func_active typ = typ = !variable in
  let func_select typ =
	variable := typ ;
	match callback_modified with
	  None -> ()
	| Some f -> f !variable
  in
  ignore (create_optionmenu lst_ops func_active func_select pack_method)

(* ============================================================================= *)
(* = Creation d'un widget de selection d'une heure                             = *)
(* =                                                                           = *)
(* = variable          = variable contenant l'heure selectionnee (ref)         = *)
(* = callback_modified = callback appele lorsque la valeur est modifiee (opt)  = *)
(* = pack_method       = ou mettre le widget                                   = *)
(* ============================================================================= *)
let create_time_select variable callback_modified pack_method =
  let split c s =
	let i = ref (String.length s - 1) in
	let j = ref !i and r = ref [] in 
	if !i >= 0 then 
      while !i >= 0 do
		while !i >= 0 & String.get s !i <> c do decr i; done; 
		if !i < !j then r := (String.sub s (!i+1) (!j - !i)) :: !r;
		while !i >= 0 & String.get s !i = c do decr i; done;
		j := !i;
      done;
	!r
  in
  let string_of_time s =
	Printf.sprintf "%02d:%02d:%02d" (s/3600) (s/60 mod 60) (s mod 60) in
  let time_of_string t =
	match split ':' t with
      [h;m;s]->(int_of_string h*60 + int_of_string m)*60 + int_of_string s
	|_-> (-1)
  in

  let b = create_hbox pack_method in
  let bmoins = create_button "-" b#pack
  and entry = GEdit.entry ~text:(string_of_time !variable)
	  ~width:65 ~packing:b#pack ()
  and bplus  = create_button "+"  b#pack in

  let set_time () =
	entry#set_text (string_of_time !variable) ;
	match callback_modified with None -> () | Some f -> f !variable
  in

  but_connect bmoins (fun () ->
	if !variable>=1 then decr variable; set_time ()) ;
  but_connect bplus (fun () ->
	if !variable<=86400-1 then incr variable; set_time ()) ;

  ignore(entry#connect#changed ~callback:(fun () ->
	variable:=time_of_string entry#text ;
	match callback_modified with None -> () | Some f -> f !variable)) ;

  match callback_modified with
	None -> ()
  | Some f ->
	  ignore(entry#connect#activate ~callback:(fun () -> f !variable))

(* ============================================================================= *)
(* = Fenetre pour affichage d'infos sous forme de labels                       = *)
(* =                                                                           = *)
(* = title       = titre de la fenetre                                         = *)
(* = width       = largeur de la fenetre                                       = *)
(* = height      = hauteur de la fenetre                                       = *)
(* ============================================================================= *)
let create_infos_win title width height =
  let (win, vb) = create_modal_window title width height in
  let update_win text =
	ignore(create_sized_label text width vb#pack) ;
	force_update_interface ()
  in
  win#show () ;
  force_update_interface () ;
  (update_win, win#destroy)

(* ============================================================================= *)
(* = Separateur dans un menu                                                   = *)
(* ============================================================================= *)
let menu_separator = ("", fun () -> ())

(* ============================================================================= *)
(* = Creation d'un menu simple dans la barre de menus d'une fenetre            = *)
(* =                                                                           = *)
(* = menu      = le menu ou acrocher le menu cree                              = *)
(* = lst_items = liste des noms+actions du menu                                = *)
(* ============================================================================= *)
let create_simple_menu menu lst_items =
  let factory = new GMenu.factory menu in
  List.iter (fun (texte, action) ->
	ignore(if texte<>"" then factory#add_item texte ~callback:action
	else factory#add_separator ())) lst_items

(* ============================================================================= *)
(* = Creation de menus dans la barre de menus d'une fenetre                    = *)
(* =                                                                           = *)
(* = menus     = liste des menus standards                                     = *)
(* = nb_menus  = variable (reference) indiquant le menu courant                = *)
(* = lst_items = liste des noms+actions du menu                                = *)
(* ============================================================================= *)
let create_menu menus nb_menus lst_items =
  create_simple_menu menus.(!nb_menus) lst_items ; incr nb_menus

(* ============================================================================= *)
(* = Creation d'un menu simple dans la barre de menus d'une fenetre            = *)
(* =                                                                           = *)
(* = menu      = le menu ou acrocher le menu cree                              = *)
(* = lst_items = liste des noms+actions du menu                                = *)
(* ============================================================================= *)
let create_simple_menu_sens menu lst_items =
  let factory = new GMenu.factory menu in
  List.fold_left (fun l data ->
	match data with
	  `I (texte, action) ->
		let m = factory#add_item texte ~callback:action in (texte, m)::l
	| `S -> ignore (factory#add_separator ()); l
	| `M (texte, entries) ->
		let sub = GMenu.menu () in let f = new GMenu.factory sub in
		List.iter (fun m ->
		  match m with
			`I (texte, action) -> ignore(f#add_item texte ~callback:action)
		  | _ -> ()) entries ;
		let m = factory#add_item ~submenu:sub texte in (texte, m)::l
	| _ -> l) [] lst_items

(* ============================================================================= *)
(* = Creation de menus dans la barre de menus d'une fenetre                    = *)
(* =                                                                           = *)
(* = menus           = liste des menus standards                               = *)
(* = store_menus     = table de stockage des menus pour recherche ensuite      = *)
(* = tab_menus_names = noms associes aux menus                                 = *)
(* = nb_menus        = variable (reference) indiquant le menu courant          = *)
(* = lst_items       = liste des noms+actions du menu                          = *)
(* ============================================================================= *)
let create_menu_sens menus tab_menus_names store_menus nb_menus lst_items =
  let l = create_simple_menu_sens menus.(!nb_menus) lst_items in
  Hashtbl.add store_menus tab_menus_names.(!nb_menus) l ;
  incr nb_menus

(* ============================================================================= *)
(* = Initialisation du stockage des menus                                      = *)
(* ============================================================================= *)
let init_menus_sens () = Hashtbl.create 13

(* ============================================================================= *)
(* = Mise a jour de l'etat active/desactive d'un sous-menu                     = *)
(* =                                                                           = *)
(* = store_menus   = table de stockage des menus                               = *)
(* = menu_name     = nom du menu                                               = *)
(* = sub_menu_name = nom du sous-menu                                          = *)
(* = sensitive     = indique l'etat a donne au sous-menu                       = *)
(* ============================================================================= *)
let set_sub_menu_sensitive store_menus menu_name sub_menu_name sensitive =
  try
	let l = Hashtbl.find store_menus menu_name in
	List.iter (fun (texte, sub_menu) ->
	  if texte = sub_menu_name then set_sensitive sub_menu sensitive) l
  with Not_found -> ()

(* ============================================================================= *)
(* = Combo box                                                                 = *)
(* ============================================================================= *)
let create_combo_simple lst_items pack_method =
  let combo = GEdit.combo ~packing:pack_method () in
  List.iter (fun item ->
	let i = GList.list_item () in
	ignore(create_label item i#add) ;
	combo#set_item_string i item ;
	combo#list#add i) lst_items ;
  (* Zone ou se trouve le texte selectionne *)
  let entry = combo#entry in
  (* On la desactive pour que l'utilisateur ne puisse pas la modifier a la main *)
  set_sensitive entry false ;

  entry

let combo_connect entry lst_items callback =
  ignore(text_entry_connect_modify entry
		   (fun s ->
			 try let (_, t) = List.find (fun (n, _) -> n=s) lst_items in callback t
			 with Not_found -> ()))

(* ============================================================================= *)
(* = Widget calendrier pour la selection d'une date. Les dates disponibles sont= *)
(* = en couleur                                                                = *)
(* ============================================================================= *)
let calendar lst_dates callback_select only_available_dates_selectable
	init_with_last_available_date tooltips win pack_method =

  (* Date de depart = la plus recente dans la liste ou date actuelle *)
  let current_month = ref 0 and current_year = ref 0 in
  if init_with_last_available_date && lst_dates<>[] then begin
	let d = List.hd (List.fast_sort (fun d1 d2 -> cmp_int d2 d1) lst_dates) in
	let (_j, m, a) = decompose_date d in current_month := m; current_year := a
  end else begin
	let tm = timer_get_time () in
	current_month := (tm.Unix.tm_mon+1); current_year:= (tm.Unix.tm_year+1900)
  end ;

  (* Transformation des dates 20030721 -> (21, 07, 2003) *)
  let lst_dates = List.map decompose_date lst_dates in
  (* Fonction indiquant si une date est dans la listes des dates autorisees *)
  let date_in_list i =
	try ignore(List.find (fun date ->
	  date = (i+1, !current_month, !current_year)) lst_dates) ; true
	with Not_found -> false
  in

  let styles =
	let default = (Obj.magic () : GObj.style) in [|default; default; default|]
  in

  (* Boite pour mettre le widget *)
  let vb = create_vbox pack_method in

  (* Boutons de changement de mois et label d'affichage mois+annee courants *)
  let hb = create_hbox vb#pack in
  let pm = pixmap_from_file "Pixmaps/left_arrow.xpm" win in
  let b_mmois = create_pixbutton pm hb#pack in
  let lab_mois = create_label "" hb#add in
  let pm = pixmap_from_file "Pixmaps/right_arrow.xpm" win in
  let b_pmois = create_pixbutton pm hb#pack in

  (* Table contenant les boutons pour les jours *)
  let calendar =
    GPack.table ~homogeneous: true ~rows:7 ~columns:7
      ~border_width:10 ~row_spacings:2 ~col_spacings:2 ~packing:vb#pack () in
  (* Barre de titre avec les jours de la semaine *)
  Array.iteri (fun i wday ->
	ignore(create_button wday
			 (calendar#attach ~top:0 ~left:i ~expand:`BOTH)))
	[|"Dim"; "Lun"; "Mar"; "Mer"; "Jeu"; "Ven"; "Sam"|] ;

  (* Creation des boutons correspondants aux jours d'un mois *)
  let buttons = Array.init 31 (fun i ->
	let b = GButton.button ~label:(string_of_int (i+1)) ~show:false () in
	but_connect b (fun () ->
	  if not only_available_dates_selectable or	(date_in_list i) then begin
		b#misc#set_style styles.(2) ;
		callback_select (compose_date (i+1, !current_month, !current_year))
	  end) ;
	b) in
  let buttons_shown = Array.create 31 false in

  (* Mise a jour des boutons dans le calendrier *)
  let update_calendar () =
	let mois = get_month_of_num !current_month in
	lab_mois#set_text (Printf.sprintf "%s %d" mois !current_year) ;

	(* Numero, dans la semaine, du premier jour du mois indique *)
	let d =
	  match get_day_of_date (1, !current_month, !current_year) with
		"Dimanche" -> 0 | "Lundi"    -> 1 | "Mardi"    -> 2
	  | "Mercredi" -> 3 | "Jeudi"    -> 4 | "Vendredi" -> 5
	  | _          -> 6
	in

	(* Suppression des boutons precedemment affiches *)
	Array.iteri (fun i button ->
	  if buttons_shown.(i) then begin
		button#misc#hide ();
		calendar#remove button#coerce ;
		buttons_shown.(i) <- false
	  end) buttons ;

	(* Affichage du bon nombre de boutons *)
	let ndays = get_nb_days_in_month !current_month !current_year in
	for i = 0 to ndays - 1 do
	  let top = (i+d) / 7 + 1 and left = (i+d) mod 7 in
      calendar#attach ~left ~top ~expand:`BOTH buttons.(i)#coerce ;
	  buttons.(i)#misc#show () ;
	  buttons_shown.(i) <- true ;
	  add_tooltips tooltips buttons.(i)
		(Printf.sprintf "%s %d %s %d"
		   (get_day_of_date (i+1, !current_month, !current_year))
		   (i+1) mois !current_year) ;

	  if date_in_list i then buttons.(i)#misc#set_style styles.(1)
	  else buttons.(i)#misc#set_style styles.(0)
	done
  in

  (* Boutons de modification du mois courant *)
  but_connect b_mmois (fun () ->
	if !current_month = 1 then begin decr current_year; current_month:=12
	end else decr current_month ;
	update_calendar ()) ;
  but_connect b_pmois (fun () ->
	if !current_month = 12 then begin incr current_year; current_month:=1
	end else incr current_month ;
	update_calendar ()) ;

  (* Valeurs des styles pour le changement de couleurs des boutons *)
  let style = win#misc#style#copy in styles.(0) <- style;
  let style = style#copy in
  style#set_bg [`NORMAL,   `NAME "light green";
				`PRELIGHT, `NAME "light green"];
  styles.(1) <- style;
  let style = style#copy in
  style#set_bg [`ACTIVE, `NAME "blue"];
				
  styles.(2) <- style;

  (* Mise a jour initiale des boutons dans le calendrier *)
  update_calendar ()

(* ============================================================================= *)
(* = Fenetre calendrier pour la selection d'une date. Les dates disponibles    = *)
(* = sont en couleur                                                           = *)
(* ============================================================================= *)
let calendar_window lst_dates callback_select
	only_available_dates_selectable	init_with_last_available_date is_modal tooltips =

  (* Creation de la fenetre modale ou pas *)
  let (win, vb) =
	if is_modal then create_modal_window "Choix de date" 0 0
	else create_window "Choix de date" 0 0
  in

  let new_callback_select date = callback_select date; win#destroy () in
  calendar lst_dates new_callback_select only_available_dates_selectable
	init_with_last_available_date tooltips win vb#add ;

  (* Bouton de fermeture de la fenetre *)
  let bbox = create_bbox vb#pack in
  let but_cancel = create_button "Annuler" bbox#add in
  but_connect but_cancel win#destroy ;

  win#show ()

(* ============================================================================= *)
(* = Creation d'une pixmap pour faire ensuite un stipple                       = *)
(* ============================================================================= *)
let create_stipple_pixmap_from_data data width height =
  try
	let s = String.create (width*height) and i = ref 0 in
	List.iter (fun v -> s.[!i] <- Char.chr v; incr i) data ;

	let c1 = GDraw.color `WHITE and c2 = GDraw.color `BLACK in
	(new GDraw.pixmap (Gdk.Pixmap.create_from_data ~width:width ~height:height
						~depth:1 ~fg:c1 ~bg:c2 s))#pixmap
  with _ -> Printf.printf "Erreur dans create_stipple_pixmap_from_data\n";
	flush stdout; exit 1

(* ============================================================================= *)
(* = Fenetre de selection d'une fonte                                          = *)
(* ============================================================================= *)
let select_font_dlg tooltips fonte_init selection_func =
  let (win, vb)= create_modal_window "Selection de fonte" 0 0 in
  let fn_dlg = GMisc.font_selection ~packing:vb#add ~show:true () in
  let lst_buts = [("OK", "Selection de la fonte");
				  ("Annuler", "Ferme la fenetre")]
  in

  if fonte_init <> "" then fn_dlg#set_font_name fonte_init ;

  let get_font () =
	if fn_dlg#font_name<>"" then selection_func fn_dlg#font_name ;
	win#destroy ()
  in

  let buts = create_buttons lst_buts tooltips (vb#pack ~from:`END) in
  create_buttons_connect buts
	[get_font; win#destroy] ;

  win#show () ;
  fn_dlg

(* ============================================================================= *)
(* = Selection du texte contenu dans un widget                                 = *)
(* ============================================================================= *)
let text_entry_select_text entry =
  entry#select_region ~start:0 ~stop:entry#text_length

(* ============================================================================= *)
(* = Creation d'une pixmap rectangulaire coloree                               = *)
(* ============================================================================= *)
let rectangle_pixmap window color width height =
  (* ================================================================== *)
  (* La ligne suivante marche jusqu'a lablgtk2-20040304, a partir des   *)
  (* versions suivantes on obtient un bug a l'execution si on n'utilise *)
  (* pas GDraw.pixmap_from_xpm_d a la place de GDraw.pixmap             *)
  (* ================================================================== *)
(*	let pm = GDraw.pixmap ~window ~width ~height () in*)
  let size = Printf.sprintf "%d %d 2 1" width height in
  let data = [size ; ". c None"; "# c #000000"] in
  let one_line = ref "" in
  for i = 0 to width-1 do one_line:=!one_line^"#" done ;
  let lines = ref [] in
  for i = 0 to height-1 do lines:=!one_line::!lines done ;

  let data = Array.of_list (data @ !lines) in
  let pm = GDraw.pixmap_from_xpm_d ~data:data
	  ~window:(window:GWindow.window) () in
  (pm:GDraw.pixmap)#set_foreground color ;
  pm#rectangle ~filled:true ~x:0 ~y:0 ~width ~height () ;
  pm

(* ============================================================================= *)
(* = Creation d'une fenetre on top                                             = *)
(* ============================================================================= *)
let create_window_on_top title width height window =
  let x = create_window title width height in
  (fst x)#set_transient_for window#as_window; x

let create_window_on_top2 title width height window =
  let x = create_window title width height in
  (match window with
	None -> () | Some window -> (fst x)#set_transient_for window#as_window) ;
  x

(* ============================================================================= *)
(* = Creation d'une fenetre modale on top                                      = *)
(* ============================================================================= *)
let create_modal_window_on_top title width height window =
  let x = create_modal_window title width height in
  (fst x)#set_transient_for window#as_window; x

let create_modal_window_on_top2 title width height window =
  let x = create_modal_window title width height in
  (match window with
	None -> () | Some window -> (fst x)#set_transient_for window#as_window) ;
  x

(* =============================== FIN ========================================= *)
