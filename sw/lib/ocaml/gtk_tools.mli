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
(** Module outils pour lablgtk-2.4.0

   Nouveaux widgets et encapsulation de fonctions GTK pour faciliter
   l'utilisation de la bibliotheque lablgtk

   {b Dépendences : Platform, Ocaml_Tools}

   {e Yann Le Fablec, version 4.10, 26/08/2004}
 *)

(** Chaine indiquant la version de la librairie *)
val version : string


(** {6 Fonctions principales} *)


(** force la mise à jour de l'interface *)
val force_update_interface : unit -> unit

(** initialise les couleurs *)
val init_colors : unit -> unit

(** lance la mainloop de l'interface *)
val main_loop : unit -> unit

(** initialise le systeme d'aide contextuelle et renvoie l'objet correspondant *)
val init_tooltips : unit -> GData.tooltips

(** [gtk_tools_add_tooltips tooltips widget texte] ajoute une aide contextuelle
   à un widget *)
val add_tooltips :
  GData.tooltips -> < coerce : GObj.widget; .. > -> string -> unit

(** renvoie la largeur et la hauteur de l'écran en pixels *)
val get_screen_size : unit -> int * int

(** [gtk_tools_disconnect wid id] déconnecte le signal [id] du widget [wid] *)
val disconnect :
  < misc : < disconnect : 'a -> 'b; .. >; .. > -> 'a -> 'b

(** [gtk_tools_set_sensitive widget sensitive] active/désactive un widget *)
val set_sensitive :
  < misc : < set_sensitive : 'a -> 'b; .. >; .. > -> 'a -> unit

(** [gtk_tools_set_sensitive_list widgets sensitive] active/désactive une liste
   de widgets *)
val set_sensitive_list :
  < misc : < set_sensitive : 'a -> 'b; .. >; .. > list -> 'a -> unit

(** [gtk_tools_set_cursor window cursor] met à jour la forme du curseur dans la
   fenetre indiquée *)
val set_cursor : Gdk.window -> Gdk.cursor -> unit

(** [gtk_tools_get_widget_size widget] renvoie un couple donnant la largeur et
   la hauteur du widget indiqué *)
val get_widget_size :
  < misc : < allocation : Gtk.rectangle; .. >; .. > -> int * int

(** [gtk_tools_set_widget_back_color widget couleur] modifie la couleur de fond
   d'un widget. Marche avec les widgets créant une fenetre (les boutons
   par exemple), ça ne fonctionne donc pas avec les labels pour lesquels il faut
   une event_box...*)
val set_widget_back_color :
    < misc : < set_style : (< set_bg : ([> `NORMAL] * GDraw.color) list ->
                                       'c;
                              .. > as 'a) ->
                           'd;
               style : < copy : 'a; .. >; .. >;
      .. > -> GDraw.color -> unit

(** [gtk_tools_set_widget_front_color w color] modifie la couleur du texte d'un widget
   (du type label) *)
val set_widget_front_color :
    < misc : < set_style : (< set_fg : ([> `NORMAL] * GDraw.color) list ->
	  'b; .. > as 'a) -> 'c; style : < copy : 'a; .. >; .. >; .. > ->
		GDraw.color -> unit

(** [gtk_tools_set_button_front_color bouton color] modifie la couleur du texte d'un
   bouton (la fonction précédente ne marche pas avec un bouton car le texte d'un bouton
   ne se trouve pas directement dans le bouton mais dans un label fils de ce bouton *)
val set_button_front_color : GButton.button -> GDraw.color -> unit

(** [gtk_tools_set_button_back_color bouton color] modifie la couleur de fond d'un bouton. *)
val set_button_back_color : GButton.button -> GDraw.color -> unit

(** [gtk_tools_set_entry_front_color entry color] modifie la couleur du texte dans une entry *)
val set_entry_front_color : GEdit.entry -> GDraw.color -> unit

(** [gtk_tools_set_entry_back_color entry color] modifie la couleur de fond d'une entry *)
val set_entry_back_color : GEdit.entry -> GDraw.color -> unit

(** [gtk_tools_set_entry_outline_color entry color] modifie la couleur du contour d'une entry *)
val set_entry_outline_color : GEdit.entry -> GDraw.color -> unit


(** [gtk_tools_scroll_adjustment adj scroll_up delta] effectue le scrolling d'un
   adjustment [adj] (d'une scrollbar par exemple) vers le haut si [scroll_up] est vrai
   et vers le bas sinon. [delta] désigne la valeur absolue du deplacement *)
val scroll_adjustment : GData.adjustment -> bool -> float -> unit

(** [gtk_tools_connect_mouse_wheel_scroll widget scrollbar delta] connecte
   un scroll de valeur absolue [delta] de la scrollbar [scrollbar]
   lors de l'utilisation de la molette souris dans le widget [widget] *)
val connect_mouse_wheel_scroll :
	< event : < connect : < button_press : callback:(GdkEvent.Button.t ->
      bool) -> 'a; scroll : callback:(GdkEvent.Scroll.t -> bool) -> 'a; .. >; .. >; .. >
		-> GRange.range -> float -> unit

(** {6 Evénements souris} *)


(** Boutons souris *)
type bouton_souris = B_GAUCHE | B_DROIT | B_MILIEU | B_NONE

(** [gtk_tools_test_mouse_but event] renvoie un element de type
   {!Gtk_tools.gtk_tools_bouton_souris} indiquant le bouton de souris pressé lors
   d'un click souris *)
val test_mouse_but : GdkEvent.Button.t -> bouton_souris

(** [gtk_tools_get_mouse_pos_click event] renvoie un couple d'entiers donnant
   la position de la souris lors d'un click *)
val get_mouse_pos_click : GdkEvent.Button.t -> int * int

(** [gtk_tools_get_mouse_pos_move event] renvoie un element de type
   {!Gtk_tools.gtk_tools_bouton_souris} indiquant le bouton de souris pressé
   pendant un deplacement de la souris *)
val get_mouse_pos_move : GdkEvent.Motion.t -> int * int

(** [gtk_tools_check_dbl_click event] teste le double click souris *)
val check_dbl_click : [> `TWO_BUTTON_PRESS] Gdk.event -> bool


(** {6 Fontes/Texte} *)


(** renvoie la fonte 'fixed' 8 points *)
val get_fixed_font : unit -> Gdk.font

(** renvoie la fonte 'fixed' 13 points *)
val get_fixed_font2 : unit -> Gdk.font

(** [gtk_tools_set_widget_font widget font] modifie la fonte d'un widget *)
val set_widget_font :
  < misc : < set_style : (< set_font : 'b -> 'c; .. > as 'a) -> 'd;
             style : < copy : 'a; .. >; .. >;
    .. > ->
  'b -> unit

(** [gtk_tools_string_width_height font string] indique la largeur et la hauteur,
   en pixels, du texte donné par [string] affiché dans la fonte [font] *)
val string_width_height : Gdk.font -> string -> int * int


(** {6 Boites} *)


(** {0 Boites de widgets} *)

(** [gtk_tools_create_bbox pack_method] crée une boite de boutons *)
val create_bbox : (GObj.widget -> unit) -> GPack.button_box

(** [gtk_tools_create_hbox pack_method] création d'une boite horizontale *)
val create_hbox : (GObj.widget -> unit) -> GPack.box

(** [gtk_tools_create_hom_hbox pack_method] création d'une boite horizontale où
   la taille des widgets est homogène *)
val create_hom_hbox : (GObj.widget -> unit) -> GPack.box

(** [gtk_tools_create_spaced_hbox pack_method] création d'une boite horizontale
 avec espacement des widgets de 5 pixels *)
val create_spaced_hbox : (GObj.widget -> unit) -> GPack.box

(** [gtk_tools_create_hom_spaced_hbox pack_method] création d'une boite horizontale
   avec espacement des widgets de 5 pixels. Les widgets ont en plus une taille
   homogène *)
val create_hom_spaced_hbox : (GObj.widget -> unit) -> GPack.box

(** [gtk_tools_create_vbox pack_method] création d'une boite verticale *)
val create_vbox : (GObj.widget -> unit) -> GPack.box

(** [gtk_tools_create_hom_vbox pack_method] création d'une boite verticale où
   la taille des widgets est homogène *)
val create_hom_vbox : (GObj.widget -> unit) -> GPack.box

(** [gtk_tools_create_spaced_vbox pack_method] création d'une boite verticale
 avec espacement des widgets de 5 pixels *)
val create_spaced_vbox : (GObj.widget -> unit) -> GPack.box

(** [gtk_tools_create_hom_spaced_vbox pack_method] création d'une boite verticale
   avec espacement des widgets de 5 pixels. Les widgets ont en plus une taille
   homogène *)
val create_hom_spaced_vbox : (GObj.widget -> unit) -> GPack.box


(** {0 Frames} *)


(** [gtk_tools_create_vframe title pack_method] crée une frame contenant une vbox*)
val create_vframe :
  string -> (GObj.widget -> unit) -> GBin.frame * GPack.box

(** [gtk_tools_create_spaced_vframe title pack_method] crée une frame contenant une vbox
   avec espaces *)
val create_spaced_vframe :
  string -> (GObj.widget -> unit) -> GBin.frame * GPack.box

(** [gtk_tools_create_hframe title pack_method] crée une frame contenant une hbox*)
val create_hframe :
  string -> (GObj.widget -> unit) -> GBin.frame * GPack.box

(** [gtk_tools_create_spaced_hframe title pack_method] crée une frame contenant une hbox
   avec espaces *)
val create_spaced_hframe :
  string -> (GObj.widget -> unit) -> GBin.frame * GPack.box


(** {0 Boites scrollables} *)


(** [gtk_tools_create_scrolled_box pack_method] crée une zone scrollable contenant
   une vbox. Renvoie la zone et la vbox *)
val create_scrolled_box :
  (GObj.widget -> unit) -> GBin.scrolled_window * GPack.box

(** [gtk_tools_change_scrolled_box scrolled_window old_box] détruit puis recrée
   la vbox dans une zone scrollable. Renvoie la nouvelle vbox *)
val change_scrolled_box :
  < add_with_viewport : GObj.widget -> unit; .. > ->
  < destroy : unit -> 'a; .. > -> GPack.box


(** {0 Notebooks} *)


(** [gtk_tools_create_notebook pack_method] crée un notebook (widget contenant
   différentes pages *)
val create_notebook : (GObj.widget -> unit) -> GPack.notebook

(** [gtk_tools_notebook_add_page notebook page_label] ajoute une page nommée
   [page_label] au notebook indiqué. En retour un couple contenant une frame
   et une boite verticale dans cette frame est retourné *)
val notebook_add_page :
  GPack.notebook -> string -> GBin.frame * GPack.box


(** {0 Paned windows} *)


(** [gtk_tools_create_hpaned pack_method] création d'une zone avec division
   mobile horizontale *)
val create_hpaned : (GObj.widget -> unit) -> GPack.paned

(** [gtk_tools_create_vpaned pack_method] création d'une zone avec division
   mobile verticale *)
val create_vpaned : (GObj.widget -> unit) -> GPack.paned


(** {6 Boutons} *)


(** [gtk_tools_create_button label pack_method] création d'un bouton *)
val create_button :
  string -> (GObj.widget -> unit) -> GButton.button

(** [gtk_tools_create_sized_button label width pack_method] création d'un bouton
   ayant une taille fixée *)
val create_sized_button :
  string -> int -> (GObj.widget -> unit) -> GButton.button

(** [gtk_tools_but_connect but func] connecte le callback [func] au bouton [but] *)
val but_connect :
  < connect : < clicked : callback:'a -> 'b; .. >; .. > -> 'a -> unit

(** [gtk_tools_but_set_label but label] change le texte dans le bouton pour le
   remplacer par [label] *)
val but_set_label :
	< children : < as_widget : 'a Gtk.obj; .. > list; .. > -> string -> unit

(** [gtk_tools_set_button_align but pos] change l'alignement du texte
   d'un bouton *)
val set_button_align :
	< children : < as_widget : 'a Gtk.obj; .. > list; .. > -> float -> unit

(** [gtk_tools_set_button_align_left but] alignement du texte
   du bouton à gauche*)
val set_button_align_left :
	< children : < as_widget : 'a Gtk.obj; .. > list; .. > -> unit

(** [gtk_tools_set_button_align_right but] idem à droite *)
val set_button_align_right :
	< children : < as_widget : 'a Gtk.obj; .. > list; .. > -> unit

(** [gtk_tools_set_button_align_center but] idem au milieu *)
val set_button_align_center :
	< children : < as_widget : 'a Gtk.obj; .. > list; .. > -> unit

(** [gtk_tools_set_button_padding but xpad] met a jour la position du texte dans le bouton *)
val set_button_padding :
	< children : < as_widget : 'a Gtk.obj; .. > list; .. > -> int -> unit

(** [gtk_tools_but_set_width but width] met a jour la largeur d'un bouton *)
val but_set_width : GButton.button -> int -> unit

(** [gtk_tools_but_set_height but height] met a jour la hauteur d'un bouton *)
val but_set_height : GButton.button -> int -> unit

(** [gtk_tools_create_buttons lst_buts tooltips pack_method] crée une rangée de
   boutons placés dans une {!Gtk_tools.gtk_tools_create_bbox}. Le paramètre
   [lst_buts] est une liste de couples du type [(nom du bouton, aide)] *)
val create_buttons :
  (string * string) list ->
  GData.tooltips -> (GObj.widget -> unit) -> GButton.button list

(** [gtk_tools_create_buttons_connect lst_buts lst_callbacks] connecte une liste
   de callbacks à une liste de boutons *)
val create_buttons_connect :
  < connect : < clicked : callback:'a -> 'b; .. >; .. > list ->
  'a list -> unit

(** [gtk_tools_create_checkbutton_simple active label pack_method tip tooltips]
 crée un check button sans callback mais avec une aide contextuelle si [tip]<>""*)
val create_checkbutton_simple :
  bool ->
  string ->
  (GObj.widget -> unit) -> string -> GData.tooltips -> GButton.toggle_button

(** [gtk_tools_create_checkbutton active label pack_method tip tooltips callback]
   crée un check button avec callback et une aide contextuelle (si [tip]<>"") *)
val create_checkbutton :
  bool ->
  string ->
  (GObj.widget -> unit) ->
  string -> GData.tooltips -> (bool -> unit) -> GButton.toggle_button

(** [gtk_tools_create_radiobuttons_simple lst_names func_active pack_method] crée
   une liste de radio boutons :
   - [lst_names] est une liste contenant des couples [(nom_bouton, type_associe)]
   - [func_active] indique quel est le bouton actif au depart
   - [pack_method] indique où mettre les boutons
 *)
val create_radiobuttons_simple :
  (string * 'a) list ->
  ('a -> bool) -> (GObj.widget -> unit) -> GButton.radio_button list

(** [gtk_tools_radiobuttons_connect lst_names lst_but func_select] connecte
   un callback lié à la modification du radiobutton selectionné. Le callback
   reçoit en paramètre le type correspondant à ce bouton *)
val radiobuttons_connect :
  ('a * 'b) list -> GButton.radio_button list -> ('b -> unit) -> unit

(** [gtk_tools_create_radiobuttons lst_names func_active func_select pack_method]
   crée des radiobuttons avec callback *)
val create_radiobuttons :
  (string * 'a) list ->
  ('a -> bool) ->
  ('a -> unit) -> (GObj.widget -> unit) -> GButton.radio_button list

(** [gtk_tools_create_togglebutton label active pack_method] crée un togglebutton*)
val create_togglebutton :
  string -> bool -> (GObj.widget -> unit) -> GButton.toggle_button

(** [gtk_tools_create_pixbutton pixmap pack_method] création d'un bouton contenant
   une pixmap *)
val create_pixbutton :
  GDraw.pixmap -> (GObj.widget -> unit) -> GButton.button


(** {6 Labels} *)


(** [gtk_tools_create_label label pack_method] création d'un label *)
val create_label : string -> (GObj.widget -> unit) -> GMisc.label

(** [gtk_tools_create_sized_label label width pack_method] création d'un label
   avec une taille fixée *)
val create_sized_label :
  string -> int -> (GObj.widget -> unit) -> GMisc.label

(** {0 Alignement des labels} *)

(** [gtk_tools_create_sized_label_align label width pos pack_method] création d'un label
   avec une taille fixée. Le texte est positionné suivant [pos] qui est compris entre
   0.0 et 1.0 (0.0 = à gauche et 1.0 = à droite) *)
val create_sized_label_align :
  string -> int -> float -> (GObj.widget -> unit) -> GMisc.label

(** [gtk_tools_create_sized_label_align_left label width pack_method] création d'un label
   avec une taille fixée. Le texte est positionné à gauche dans le label *)
val create_sized_label_align_left :
  string -> int -> (GObj.widget -> unit) -> GMisc.label

(** [gtk_tools_create_sized_label_align_right label width pack_method] création d'un label
   avec une taille fixée. Le texte est positionné à droite dans le label *)
val create_sized_label_align_right :
  string -> int -> (GObj.widget -> unit) -> GMisc.label

(** [gtk_tools_set_label_align label pos] fixe l'alignement du texte dans un label.
   [pos] indique où se fait l'alignement : 0.0 = à gauche et 1.0 = à droite *)
val set_label_align : GMisc.label -> float -> unit

(** [gtk_tools_set_label_align_left label] fixe l'alignement du texte dans un label
   à gauche *)
val set_label_align_left : GMisc.label -> unit

(** [gtk_tools_set_label_align_right label] fixe l'alignement du texte dans un label
   à droite *)
val set_label_align_right : GMisc.label -> unit

(** [gtk_tools_set_label_align_center label] fixe l'alignement du texte dans un label
   au centre de ce dernier *)
val set_label_align_center : GMisc.label -> unit

(** [gtk_tools_set_label_padding label xpad] fixe la position gauche du texte
   dans un label *)
val set_label_padding : GMisc.label -> int -> unit

(** {6 Boites de dialogue} *)


(** [gtk_tools_question_box window title question_msg default_is_cancel] crée une
   fenetre posant une question à l'utilisateur :
   - [window] désigne la fenetre mère
   - [title] le titre à donner à la fenetre
   - [question_msg] le message à afficher
   - [defaut_is_cancel] indique si par defaut c'est le bouton Annuler qui est
   selectionné

   Renvoie vrai si OK a été choisi, faux sinon
 *)
val question_box :
  < misc : #GDraw.misc_ops; .. > -> string -> string -> bool -> bool

(** [gtk_tools_error_box window title error_msg] affiche une boite contenant un
   message d'erreur *)
val error_box :
  < misc : #GDraw.misc_ops; .. > -> string -> string -> unit

(** [gtk_tools_animated_msg_box title msg lst_pixmaps] crée une boite de message
   contenant un icone animé dont [lst_pixmaps] désigne les noms des différentes
   images (pixmaps) de l'animation *)
val animated_msg_box : string -> string -> string list -> unit

(** [gtk_tools_open_file_dlg title read_func update_func default_filename
   check_overwrite] crée une boite de selection de fichier :
   - [title] indique le titre à donner à la fenetre
   - [read_func] fonction appelée pour traiter le fichier selectionné
   - [update_func] : [None] ou [Some f] fonction optionnelle appelée après
   le traitement du fichier par la fonction précédente
   - [default_filename] nom de fichier/répertoire par defaut
   - [check_overwrite] indique si on doit tester l'existence du fichier (pour
   eviter un écrasement
 *)
val open_file_dlg :
  string -> (string -> 'a) -> (unit -> unit) option -> string -> bool -> unit

(** [gtk_tools_select_font_dlg tooltips fonte_init selection_func] ouvre une fenetre
   de selection de fonte. Si une fonte est choisie, [selection_func] est
   appelée avec son nom. [fonte_init] désigne le nom de la fonte sélectionnée par
   défaut ("" pour rien). *)
val select_font_dlg : GData.tooltips -> string -> (string -> unit)
  -> GMisc.font_selection

(** {6 Timers} *)


(** Type de timer *)
type timer_type =
	TIMER_TIME          (** Heure uniquement *)
  | TIMER_DATE          (** Date uniquement  *)
  | TIMER_TIME_AND_DATE (** Affichage de l'heure et de la date *)

(** [gtk_tools_insert_timer label timer_type force_beginning] place un timer
   de type [timer_type] dans le widget [label]. Si [force_beginning] est vrai
   alors l'affichage commence dès la creation du timer *)
val insert_timer :
  GMisc.label -> timer_type -> bool -> unit


(** {6 Menus} *)

(** [gtk_tools_popup menus_entries] affiche un popup menu *)
val popup : GToolbox.menu_entry list -> unit

(** [gtk_tools_connect_popup_menu wid button test_cond_func menu_entries] connecte
   un popup menu à un widget :
   - [wid] désigne le widget concerné
   - [button] le bouton à presser pour afficher le popup menu
   - [test_cond_func] fonction de test indiquant si le menu doit etre affiché
   - [menu_entries] contient la liste des éléments du menu
*)
val connect_popup_menu :
  < event : < connect : < button_press : callback:(GdkEvent.Button.t -> bool) ->
                                         'a; .. >; .. >; .. > ->
  bouton_souris ->
  (unit -> bool) -> GToolbox.menu_entry list -> unit

(** [gtk_tools_connect_func_popup_menu wid button test_cond_func get_menu_entries]
   connecte un popup menu construit de manière dynamique à un widget :
   - [wid] désigne le widget concerné
   - [button] le bouton à presser pour afficher le popup menu
   - [test_cond_func] fonction de test indiquant si le menu doit etre affiché
   - [get_menu_entries] désigne la fonction appelée au moment de l'appui sur
   le bouton et qui renvoie la liste des éléments du menu *)
val connect_func_popup_menu :
  < event : < connect : < button_press : callback:(GdkEvent.Button.t -> bool) ->
                                         'a; .. >; .. >; .. > ->
  bouton_souris ->
  (unit -> bool) -> (unit -> GToolbox.menu_entry list) -> unit

(** [gtk_tools_create_popup_menu title event data] crée un popup menu la où se
   trouve la souris. [data] est une liste d'éléments indiquant le contenu du menu
   sous la forme [(texte, Some fonction, parametre fonction, sous menu)].
   [titre] désigne le titre du menu, peut etre egal à "" pour ne pas mettre
   de titre *)
val create_popup_menu :
  string ->
  GdkEvent.Button.t ->
  (string * ('a -> unit) option * 'a *
   (string * ('b -> unit) option * 'b) list)
  list -> unit

(** [gtk_tools_create_optionmenu lst_names func_active func_select pack_method]
   creation d'un menu à options :
   - [lst_names] contient la liste des options avec leur type sous la forme
   [(nom, type)]
   - [func_active] indique quelle est l'option active par defaut
   - [func_select] appelée lors de la modification de l'option
   - [pack_method] désigne l'endroit où mettre le menu

   Renvoie le menu ainsi qu'une fonction permettant de mettre à jour l'option
   courante et une seconde fonction mettant à jour l'option courante et qui
   appelle la fonction [func_select] en plus :
   [(option_menu, set_option, set_option_and_activate)]
 *)
val create_optionmenu :
  (string * 'a) list ->
  ('a -> bool) ->
  ('a -> unit) -> (GObj.widget -> unit) ->
  GMenu.option_menu * ('a -> unit) * ('a -> unit)

(** [separateur dans un menu] *)
val menu_separator : string * (unit -> unit)

(** [gtk_tools_create_simple_menu menu lst_items] crée un menu attaché à [menu]
   et défini par [lst_items]. [lst_items] est une liste d'éléments du type
   [(texte_menu, action)] définissant les éléments du menu à créer *)
val create_simple_menu :
  #GMenu.menu_shell -> (string * (unit -> unit)) list -> unit

(** [gtk_tools_create_menu menus nb_menus lst_items] : identique à la fonction
   précédente sauf que [menus] désigne un tableau de menus et [nb_menus] la référence
   sur une variable indiquant le menu courant dans ce tableau *)
val create_menu :
  #GMenu.menu_shell array ->
  int ref -> (string * (unit -> unit)) list -> unit

(** Initialise la variable de stockage des menus (permet de les activer ou de les
   desactiver) *)
val init_menus_sens : unit -> (string, (string*GMenu.menu_item) list) Hashtbl.t

(** [gtk_tools_create_simple_menu_sens menu lst_items] crée un menu dont les sous-menus
   peuvent etre activés ou désactivés *)
val create_simple_menu_sens : #GMenu.menu_shell -> GToolbox.menu_entry list ->
  (string * GMenu.menu_item) list

(** [gtk_tools_create_menu_sens menus tab_menus_names store_menus nb_menus lst_items] créé un menu
   dont les sous-menus peuvent etre rendus actifs ou inactifs. [menus] désigne le tableau des menus,
   [tab_menus_names] est un tableau contenant les noms des ces menus, [store_menus] contient la
   variable de stockage créée avec {!Gtk_tools.gtk_tools_init_menus_sens}, nb_menus est une référence
   sur la variable contenant le menu en cours de création. *)
val create_menu_sens : #GMenu.menu_shell array ->
  string array -> (string, (string*GMenu.menu_item) list) Hashtbl.t ->
	int ref -> GToolbox.menu_entry list -> unit

(** [gtk_tools_set_sub_menu_sensitive store_menus menu_name sub_menu_name sensitive] active ou
   désactive (suivant la valeur de [sensitive]) le sous-menu de nom [sub_menu_name] dans le
   menu de nom [menu_name] *)
val set_sub_menu_sensitive :
	(string, (string*GMenu.menu_item) list) Hashtbl.t -> string -> string -> bool -> unit


(** {6 Pixmaps} *)


(** pixmap d'ouverture d'un fichier *)
val open_file_pixmap : string array

(** [gtk_tools_pixmap_from_file filename window] création d'un [GDraw.pixmap] à
   partir d'un fichier xpm *)
val pixmap_from_file : string -> GWindow.window -> GDraw.pixmap

(** [gtk_tools_create_pixmap window width height] crée une pixmap de taille
   [width]*[height] où [window] désigne la fenetre mère de la pixmap.
   En retour, un couple [(pix, pixmap)] où [pix] sert à mettre la pixmap dans
   une zone de dessin (par ex.) et [pixmap] sert au dessin *)
val create_pixmap :
  GWindow.window -> int -> int -> Gdk.pixmap * GDraw.pixmap

(** [gtk_tools_create_d_pixmap window width height] crée un drawable de taille
   [width]*[height] où [window] désigne la fenetre mère du drawable.
   En retour, un couple [(pix, pixmap)] où [pix] sert à mettre le drawable dans
   une zone de dessin (par ex.) et [pixmap] sert au dessin *)
val create_d_pixmap :
  GWindow.window -> int -> int -> Gdk.pixmap * GDraw.drawable

(** [gtk_tools_create_stipple_pixmap_from_data data width height] crée une pixmap
   utilisable pour faire un stipple a partir de [data] qui est une liste d'entiers *)
val create_stipple_pixmap_from_data : int list -> int -> int -> Gdk.pixmap

(** [gtk_tools_rectangle_pixmap window color width height] crée une pixmap
   rectangulaire de taille [width]*[height] et de couleur [color] *)
val rectangle_pixmap : GWindow.window -> GDraw.color -> int -> int ->
   GDraw.pixmap


(** {6 Fenetres} *)


(** [gtk_tools_create_window title width height] crée une fenetre contenant une
   boite verticale. Si [width] et [height] sont nuls, la fenetre n'a pas de
   taille par défaut *)
val create_window :
  string -> int -> int -> GWindow.window * GPack.box

(** [gtk_tools_create_window_on_top title width height window] *)
val create_window_on_top :
   string -> int -> int -> GWindow.window -> GWindow.window * GPack.box

(** [gtk_tools_create_window_on_top2 title width height window] *)
val create_window_on_top2 :
   string -> int -> int -> GWindow.window option -> GWindow.window * GPack.box

(** [gtk_tools_create_modal_window title width height] est identique à la fonction
   précédente sauf que la fenetre est modale *)
val create_modal_window :
  string -> int -> int -> GWindow.window * GPack.box

(** [gtk_tools_create_modal_window_on_top title width height window] *)
val create_modal_window_on_top :
   string -> int -> int -> GWindow.window -> GWindow.window * GPack.box

(** [gtk_tools_create_modal_window_on_top2 title width height window] *)
val create_modal_window_on_top2 :
   string -> int -> int -> GWindow.window option -> GWindow.window * GPack.box

(** [gtk_tools_create_window_with_menubar title width height menubar_items] crée
   une fenetre contenant une barre de menus *)
val create_window_with_menubar :
  string ->
  int ->
  int ->
  string list ->
  GWindow.window * GPack.box * GMenu.menu_shell GMenu.factory *
  Gtk.accel_group * GMenu.menu array

(** [gtk_tools_create_window_with_menubar_help title width height menubar_items]
   crée une fenetre avec une barre de menus et un menu d'aide à gauche de
   cette barre *)
val create_window_with_menubar_help :
  string ->
  int ->
  int ->
  string list ->
  GWindow.window * GPack.box * GMenu.menu_shell GMenu.factory *
  Gtk.accel_group * GMenu.menu array * GMenu.menu

(** [gtk_tools_window_set_front window] met la fenetre [window] en avant plan *)
val window_set_front : GWindow.window -> unit

(** [gtk_tools_get_window_geometry window] renvoie la position et la taille de
   la fenetre sous la forme [((x, y), (largeur, hauteur))] *)
val get_window_geometry :
  GWindow.window -> (int * int) * (int * int)

(** set_window_position window (x, y) déplace la fenetre pour que son
   coin superieur à gauche soit à la position [(x, y)] *)
val set_window_position : GWindow.window -> int * int -> unit

(** [gtk_tools_window_modify_connect window callback] connection d'un callback
   appelé lors du deplacement ou du changement de taille de la fenetre [window].
   Le callback reçoit en paramètres la position et la taille de la fenetre *)
val window_modify_connect :
  GWindow.window -> ((int * int) * (int * int) -> 'a) -> GtkSignal.id

(** [gtk_tools_connect_win_focus_change window focus_in focus_out] connecte les
   callbacks correspondants aux événements focus_in et focus_out à une fenetre *)
val connect_win_focus_change :
  GWindow.window -> (unit -> 'a) option -> (unit -> 'b) option -> unit



(** {6 Zones de dessin (Drawing Areas)} *)


(** [gtk_tools_create_draw_area_simple width height pack_method pix_expose] crée
   une zone de dessin simple où [pix_expose] désigne la pixmap à utiliser pour
   redessiner la zone après un event expose. En retour, un couple contenant
   la zone créée et la fonction de mise à jour du dessin est renvoyé *)
val create_draw_area_simple :
  int ->
  int ->
  (GObj.widget -> unit) -> Gdk.pixmap -> GMisc.drawing_area * (unit -> unit)

(** [gtk_tools_area_mouse_connect area mouse_press mouse_move mouse_release]
   connecte les événements souris à la zone de dessin [area] *)
val area_mouse_connect :
  GMisc.drawing_area ->
  (GdkEvent.Button.t -> bool) ->
  (GdkEvent.Motion.t -> bool) -> (GdkEvent.Button.t -> bool) -> unit

(** [gtk_tools_area_key_connect area key_press key_release] connecte les
   événements claviers à la zone de dessin [area] *)
val area_key_connect :
  GMisc.drawing_area -> (Gdk.keysym -> bool) -> (Gdk.keysym -> bool) -> unit

(** [gtk_tools_create_draw_area width height pack_method pix_expose
   mouse_press mouse_move mouse_release] crée une zone de dessin et connecte
   les événements souris *)
val create_draw_area :
  int ->
  int ->
  (GObj.widget -> unit) ->
  Gdk.pixmap ->
  (GdkEvent.Button.t -> bool) ->
  (GdkEvent.Motion.t -> bool) -> (GdkEvent.Button.t -> bool) -> unit -> unit


(** {6 Sélection de couleurs} *)


(** [gtk_tools_select_color update_func] crée une boite de selection de couleur.
   La fonction [update_func] est appelée après sélection avec en paramètre la
   couleur choisie *)
val select_color : ([> `RGB of int * int * int] -> unit) -> unit

(** [gtk_tools_create_color_selection_button window taille_x taille_y color
	pack_method callback] crée un bouton coloré permettant le choix d'une couleur.
   - [window] désigne la fenetre mère
   - [taille_x] largeur du bouton
   - [taille_y] hauteur du bouton
   - [color] couleur initiale du bouton
   - [pack_method] indique où mettre le bouton
   - [callback] fonction appelée lorsqu'une nouvelle couleur est selectionnée.
   Cette fonction reçoit en paramètre la nouvelle couleur

   Lors du choix d'une nouvelle couleur, la couleur du bouton n'est pas mise à jour
 *)
val create_color_selection_button :
  < misc : #GDraw.misc_ops; .. > ->
  int ->
  int ->
  GDraw.color ->
  (GObj.widget -> unit) ->
  ([> `RGB of int * int * int] -> unit) -> GButton.button

(** [gtk_tools_create_color_selection_button2 window taille_x taille_y color
	pack_method callback] identique à la fonction précédente sauf que la couleur
   selectionnee est mise à jour dans le bouton *)
val create_color_selection_button2 :
  < misc : #GDraw.misc_ops; .. > ->
  int ->
  int ->
  GDraw.color ->
  (GObj.widget -> unit) -> (GDraw.color -> unit) -> GButton.button

(** [gtk_tools_select_colors_widget window colors tooltips update_func vbox] crée
   un widget de selection de plusieurs couleurs. [colors] est une liste d'éléments
   au format [(nom_frame, (label, couleur ref, couleur_defaut) list)] *)
val select_colors_widget :
  GWindow.window ->
  (string * (string * GDraw.color ref * GDraw.color) list) list ->
  GData.tooltips option -> (unit -> unit) -> GPack.box -> unit

(** [gtk_tools_select_colors title width height colors tooltips update_func] crée
   une fenetre de sélection de plusieurs couleurs *)
val select_colors :
  string ->
  int ->
  int ->
  (string * (string * GDraw.color ref * GDraw.color) list) list ->
  GData.tooltips option -> (unit -> unit) -> unit


(** {6 Captures d'écran} *)


(** [creation_fen_capture default_filename default_format tooltips with_caption]
 fonction interne de création d'une boite de capture d'écran *)
val creation_fen_capture :
  string ->
  Gtk_image.format_capture ->
  GData.tooltips option ->
  bool ->
  string ref * Gtk_image.format_capture ref *
  (Gtk_image.progress_save -> float -> unit) option * [> `DELETE_EVENT ] GWindow.dialog *
  GButton.button * GEdit.entry *
  (unit ->
   (string * GDraw.color * GDraw.color * GDraw.color * Gdk.font) option)

(** [gtk_tools_screenshot_box default_filename default_format drawable
   tooltips x y width height] crée une boite de capture d'écran sans légende *)
val screenshot_box :
  string ->
  Gtk_image.format_capture ->
  [> `drawable ] Gobject.obj ->
  GData.tooltips option -> int -> int -> int -> int -> unit

(** [gtk_tools_screenshot_box_with_func default_filename default_format drawable
	tooltips x y width height after_func] identique à la fonction précédente sauf
   que la fonction [after_func] est appelée après la capture *)
val screenshot_box_with_func :
  string ->
  Gtk_image.format_capture ->
  [> `drawable ] Gobject.obj ->
  GData.tooltips option -> int -> int -> int -> int -> (unit -> unit) -> unit

(** [gtk_tools_screenshot_box_with_caption default_filename default_format
	drawable tooltips x y width height] boite de capture écran avec légende. Ne
   fonctionne qu'avec des pixmaps à cause de la légende *)
val screenshot_box_with_caption :
  string ->
  Gtk_image.format_capture ->
  Gdk.pixmap -> GData.tooltips option -> int -> int -> int -> int -> unit


(** {6 Sélection de valeurs entières} *)


(** [gtk_tools_create_int_spinner_simple label lab_width init_value min_value
	max_value value_width step_incr page_incr tip tooltips pack_method] crée un
   widget de sélection d'une valeur entière *)
val create_int_spinner_simple :
  string ->
  int ->
  int ->
  int ->
  int ->
  int ->
  int ->
  int ->
  string -> GData.tooltips -> (GObj.widget -> unit) -> GEdit.spin_button

(** [gtk_tools_int_spinner_connect sp callback] connecte le callback à un widget de
   sélection de valeur entière. Le callback est appelé avec la valeur selectionnée
   à chaque fois que celle-ci change *)
val int_spinner_connect :	GEdit.spin_button -> (int -> unit) -> unit

(** [gtk_tools_create_int_spinner label lab_width init_value min_value max_value
	value_width step_incr page_incr tip tooltips pack_method callback] crée un
   widget de sélection de valeur entière avec callback *)
val create_int_spinner :
  string ->
  int ->
  int ->
  int ->
  int ->
  int ->
  int ->
  int ->
  string ->
  GData.tooltips ->
  (GObj.widget -> unit) -> (int -> unit) -> GEdit.spin_button

(** [gtk_tools_create_vslider_simple init_val min_val max_val step page draw_val
   pack_method] création d'un slider vertical de sélection d'une valeur entière *)
val create_vslider_simple :
  int ->
  int ->
  int ->
  int ->
  int -> bool -> (GObj.widget -> unit) -> GData.adjustment * GRange.scale

(** [gtk_tools_create_hslider_simple init_val min_val max_val step page draw_val
   pack_method] création d'un slider horizontal de sélection d'une valeur entière *)
val create_hslider_simple :
  int ->
  int ->
  int ->
  int ->
  int -> bool -> (GObj.widget -> unit) -> GData.adjustment * GRange.scale

(** [gtk_tools_slider_connect slider callback] connection d'un callback de
   modification d'un slider de valeur entière *)
val slider_connect :
  GData.adjustment -> (int -> unit) -> GtkSignal.id

(** [gtk_tools_create_vslider init_val min_val max_val step page draw_val
	pack_method callback] création d'un slider vertical de sélection d'une valeur
   entière avec callback *)
val create_vslider :
  int ->
  int ->
  int ->
  int ->
  int ->
  bool ->
  (GObj.widget -> unit) -> (int -> unit) -> GData.adjustment * GRange.scale

(** [gtk_tools_create_hslider init_val min_val max_val step page draw_val
	pack_method callback] création d'un slider horizontal de sélection d'une valeur
   entière avec callback *)
val create_hslider :
  int ->
  int ->
  int ->
  int ->
  int ->
  bool ->
  (GObj.widget -> unit) -> (int -> unit) -> GData.adjustment * GRange.scale


(** {6 Sélection de valeurs flottantes} *)


(** [gtk_tools_create_float_spinner_simple label lab_width init_value min_value
   max_value value_width nb_digits step_incr page_incr tip tooltips pack_method]
   crée un widget de sélection d'une valeur flottante *)
val create_float_spinner_simple :
  string ->
  int ->
  float ->
  float ->
  float ->
  int ->
  int ->
  float ->
  float ->
  string -> GData.tooltips -> (GObj.widget -> unit) -> GEdit.spin_button

(** [gtk_tools_float_spinner_connect sp callback] connecte le callback à un widget de
   sélection de valeur flottante. Le callback est appelé avec la valeur selectionnée
   à chaque fois que celle-ci change *)
val float_spinner_connect : GEdit.spin_button -> (float -> unit) -> unit

(** [gtk_tools_create_float_spinner label lab_width init_value min_value max_value
	value_width nb_digits step_incr page_incr tip tooltips pack_method callback]
   crée un widget de sélection de valeur flottante avec callback *)
val create_float_spinner :
  string ->
  int ->
  float ->
  float ->
  float ->
  int ->
  int ->
  float ->
  float ->
  string ->
  GData.tooltips ->
  (GObj.widget -> unit) -> (float -> unit) -> GEdit.spin_button


(** {6 Zones de texte} *)


(** [gtk_tools_create_text_entry_simple label lab_width init_value value_width
	tip tooltips pack_method] : zone de sélection de texte. Renvoie le label et
   la zone de sélection de texte *)
val create_text_entry_simple :
  string ->
  int ->
  string ->
  int ->
  string ->
  GData.tooltips -> (GObj.widget -> unit) -> GMisc.label * GEdit.entry

(** [gtk_tools_text_entry_connect entry callback] connecte un callback lié à
   l'appui sur la touche entrée dans la zone [entry] *)
val text_entry_connect :
  GEdit.entry -> (string -> unit) -> GtkSignal.id

(** [gtk_tools_text_entry_connect_modify entry callback] connecte un callback
   appelé à chaque fois que le texte de la zone [entry] est modifié. Le callback
   reçoit la chaine contenue dans la zone en majuscules *)
val text_entry_connect_modify :
  GEdit.entry -> (string -> unit) -> GtkSignal.id

(** [gtk_tools_create_text_entry label lab_width init_value value_width
	tip tooltips pack_method callback] zone de sélection de texte avec callback
   lié à la modification du texte dans la zone *)
val create_text_entry :
  string ->
  int ->
  string ->
  int ->
  string ->
  GData.tooltips ->
  (GObj.widget -> unit) -> (string -> unit) -> GMisc.label * GEdit.entry

(** [gtk_tools_text_entry_select_text entry] selectionne le texte présent dans
   la zone de texte *)
val text_entry_select_text : GEdit.entry -> unit

(** [gtk_tools_create_text_edit editable with_vert_scroll with_hor_scroll pack_method]
   crée une zone de texte multiligne (editeur) avec éventuellement des barres de
   défilement ([with_vert_scroll] et [with_hor_scroll]). [editable] indique si
   la zone créée est éditable par l'utilisateur *)
val create_text_edit : bool -> bool -> bool -> (GObj.widget -> unit)
	-> GText.view

(** [gtk_tools_text_edit_clear edit] efface le contenu d'une zone de texte *)
val text_edit_clear : GText.view -> unit

(** [gtk_tools_text_edit_get_text edit] renvoie l'intégralité du texte contenu
   dans [edit] *)
val text_edit_get_text : GText.view -> string

(** [gtk_tools_text_edit_get_lines edit] identique à la fonction précédente sauf
   que le texte contenu dans le widget est renvoyé sous la forme d'une liste de
   chaines de caractères, chacune d'elles correspondant à une ligne dans la
   zone de texte *)
val text_edit_get_lines : GText.view -> string list

(** [gtk_tools_text_edit_set_text_list edit text_lst] insère la liste de chaines
   de caractères dans la zone de texte *)
val text_edit_set_text_list : GText.view -> string list -> unit

(** [gtk_tools_text_edit_set_text edit text] insère le texte indiqué dans la
   zone de texte *)
val text_edit_set_text : GText.view -> string -> unit


(** {6 Fenetres enregistrées} *)


(** Type pour les fenetres enregistrées *)
type registered_win = {
  reg_win_id : int;
  mutable reg_win_handle : GWindow.window option;
  reg_win_build : unit -> GWindow.window;
} 

(** Exception levée lors de l'appel à un numéro de fenetre non enregistrée *)
exception GTK_TOOLS_UNREGISTERED_WINDOW of int

(** [gtk_tools_register_window build_window_func] enregistre une fenetre.
   [build_window_func] est la fonction de création de la fenetre en question *)
val register_window : (unit -> GWindow.window) -> int

(** [gtk_tools_show_registered_window id] crée la fenetre enregistrée designée par
   [id]. Si la fenetre existe déjà elle est mise en avant plan *)
val show_registered_window : int -> unit

(** [gtk_tools_hide_registered_window id] détruit la fenetre enregistrée si elle
   existe *)
val hide_registered_window : int -> unit

(** [gtk_tools_get_registered_window id] renvoie la fenetre enregistrée
   correspondant à l'identifiant [id] *)
val get_registered_window : int -> GWindow.window option


(** {6 Listes} *)


(** [gtk_tools_create_list lst_titles (sortable_titles, first_sort) pack_method]
   crée une liste :
   - [lst_titles] liste des titres des colonnes
   - [sortable_titles] les titres sont-ils sélectionnables pour trier la liste ?
   - [first_sort] indique quelle est la colonne qui sert de tri initialement
   - [pack_method] où mettre la liste *)
val create_list :
  string list -> bool * int -> (GObj.widget -> unit) -> string GList.clist

(** [gtk_tools_create_list_with_hor_scroll lst_titles
   (sortable_titles, first_sort) pack_method] crée une liste avec une barre de
   scroll horizontale *)
val create_list_with_hor_scroll :
  string list -> bool * int -> (GObj.widget -> unit) -> string GList.clist

(** [gtk_tools_set_columns_sizes list sizes] met à jour la taille des colonnes
   de la liste [list] *)
val set_columns_sizes : string GList.clist -> int list -> unit

(** [gtk_tools_list_connect clist
   callback_select callback_deselect callback_select_column] connecte les
   événements sélection/désélection et sélection d'un titre de colonne. Les
   callbacks [callback_select] et [callback_deselect] reçoivent en paramètres
   la ligne et la colonne (dé)sélectionnées. [callback_select_column] reçoit le
   numéro de la colonne *)
val list_connect :
  string GList.clist ->
  (int -> int -> unit) option ->
  (int -> int -> unit) option -> (int -> unit) option -> unit

(** [gtk_tools_list_connect_check_dbl_click clist
	callback_select callback_deselect callback_select_column] meme chose que la
   fonction précédente sauf que l'on teste s'il y a double click. Ici,
   [callback_select] et [callback_deselect] reçoivent la ligne, la colonne et
   un booleen indiquant s'il y a eu double click *)
val list_connect_check_dbl_click :
  string GList.clist ->
  (int -> int -> bool -> unit) option ->
  (int -> int -> bool -> unit) option -> (int -> unit) option -> unit

(** [gtk_tools_list_connect_up_down_keys clist callback_up callback_down]
   connecte les callbacks liés à l'appui sur les touches flechées haut et bas *)
val list_connect_up_down_keys :
  string GList.clist -> (unit -> 'a) -> (unit -> 'b) -> unit

(** [gtk_tools_create_managed_list titles_sizes pack_method] crée une liste managée.
   [titles_sizes] est une liste du type [(nom, taille)] indiquant le titre
   ainsi que la largeur des différentes colonnes. Cet objet contient en plus
   un label indiquant le nombre d'éléments contenus dans la liste.

   En retour, la liste et le label sont renvoyes *)
val create_managed_list :
  (string * int) list ->
  (GObj.widget -> unit) -> string GList.clist * GMisc.label

(** [gtk_tools_connect_managed_list (lst, lab) item_column selection_callback
   (name, female, cap)] connecte le callback de (dé)sélection à une liste
   managée, les callbacks de selection avec les touches flechées et indique le
   format du label de titre :
   - [(lst, lab)] = la liste et le label renvoyés par la fonction précédente
   - [item_column] = numéro de la colonne contenant l'item de référence
   - [selection_callback] = la fonction appelée lors d'une (dé)sélection.
   Elle appelée avec en paramètres : l'élément sélectionné (i.e élément de la ligne
   sélectionnée et dans la colonne [item_column]), un triplet [(ligne, colonne,
   double_ckick)] et un booleen valant vrai si sélection et faux si désélection
   - [(name, female, cap)] indique le format à utiliser pour afficher le nombre
   d'éléments de la liste (voir [eval_string])

   En retour est renvoyée la fonction permettant de mettre à jour le contenu de
   la liste. Cett fonction prend en paramètres :
   - l'identifiant de la ligne à sélectionner par defaut ("" si pas de
   sélection initiale)
   - une liste contenant des listes correspondant aux différentes lignes *)
val connect_managed_list :
  string GList.clist * GMisc.label ->
  int ->
  (string -> int * int * bool -> bool -> unit) ->
  string * bool * bool -> string -> string list list -> unit


(** {6 Widget Lat/Lon} *)


(** Type contenant un widget de sélection de coordonnées lat/lon *)
type latlon = {
  latlon_lat_val : float ref;
  latlon_lon_val : float ref;
  latlon_update : float -> float -> unit;
  latlon_change_callback : (unit -> unit) option ref;
} 

(** [gtk_tools_create_latlon_selection (lat, lon) pack_method tooltips] crée un
   widget de sélection de coordonnées géographiques lat/lon *)
val create_latlon_selection :
  float * float ->
  (GObj.widget -> unit) -> GData.tooltips -> latlon

(** [gtk_tools_update_latlon_selection latlon_widget new_lat new_lon] met à jour
   les coordonnées dans un widget de sélection lat/lon *)
val update_latlon_selection :
  latlon -> float -> float -> unit

(** [gtk_tools_latlon_selection_get latlon_widget] recupère les coordonnées
   selectionnées dans le widget *)
val latlon_selection_get : latlon -> float * float

(** [gtk_tools_latlon_selection_change latlon_widget callback] connecte un
   callback appelé lors de la modification des coordonnées. Il ne prend pas de
   paramètre : pour avoir connaitre la valeur des coordonnées, il faut utiliser
   {!Gtk_tools.gtk_tools_latlon_selection_get} *)
val latlon_selection_change :
  latlon -> (unit -> unit) -> unit


(** {6 Fenetre de Log} *)

(** Exception levée lors de l'ajout de texte dans la fenetre de log alors que
   celle-ci n'a pas encore ete créée *)
exception GTK_TOOLS_NO_LOG_WIN

(** renvoie la zone de texte de la fenetre de log si cette derniere a ete créée.
   Si ce n'est pas le cas alors l'exception {!Gtk_tools.GTK_TOOLS_NO_LOG_WIN}
   est levée *)
val get_log_wid : unit -> GText.view

(** [gtk_tools_add_log text level] ajoute un texte dans la fenetre de log
   si [level]<[log_verbose_level]*)
val add_log : string -> int -> unit

(** [gtk_tools_add_log_with_color text level color] meme chose que la
   fonction précédente sauf que [color] indique la couleur du texte à ajouter *)
val add_log_with_color : string -> int -> GDraw.color -> unit

(** efface le contenu de la fenetre de log *)
val clear_log : unit -> unit

(** [gtk_tools_create_log tooltips] crée la fenetre de log *)
val create_log : GData.tooltips -> unit

(** force l'affichage de la fenetre de log *)
val show_log : unit -> unit

(** détruit la fenetre de log *)
val hide_log : unit -> unit

(** [gtk_tools_set_log_verbose_level level] met à jour le niveau de verbose
   dans la fenetre de log *)
val set_log_verbose_level : int -> unit


(** {6 Barres de progression} *)


(** [gtk_tools_create_progress_bar_win nb_blocks title] crée une barre de progression
   dans une fenetre externe. [nb_blocks] désigne le nombre de subdivisions de
   la barre. En sortie, la fonction de mise à jour de la barre de progression
   est renvoyée. Cette fonction prend en paramètre un flottant compris entre
   0.0 et 1.0, lorsqu'on lui passe 1.0, la fenetre est fermée *)
val create_progress_bar_win : int -> string -> float -> unit

(** [gtk_tools_create_progress_bar pack_method] creation d'une barre de
   progression continue et sans fenetre (donc différente de
   {!Gtk_tools.gtk_tools_create_progress_bar_win}.
   La fonction de mise à jour de la progression est renvoyée et prend en
   paramètre un flottant compris entre 0.0 et 1.0 *)
val create_progress_bar : (GObj.widget -> unit) -> float -> unit


(** {6 Widget de selection d'opérateur de comparaison} *)


(** Opérateurs de comparaison *)
type t_ops_compare = T_EQ | T_L | T_LEQ | T_G | T_GEQ

(** [gtk_tools_create_ops_compare variable callback_modified pack_method] crée un
   widget de sélection d'opérateur de comparaison.
   - [variable] est une référence sur l'opérateur en cours
   - [callback_modified] est appelé lorsque l'opérateur est modifié. Cette fonction
   est optionnelle. Si elle est utilisée alors elle prend en paramètre la
   valeur courante de l'opérateur de comparaison
   - [pack_method] indique où mettre le widget de sélection
 *)
val create_ops_compare :
  t_ops_compare ref ->
  (t_ops_compare -> unit) option -> (GObj.widget -> unit) -> unit


(** {6 Widget de sélection d'une heure} *)


(** [gtk_tools_create_time_select variable callback_modified pack_method] crée un
   widget de sélection d'heure. [variable] est une référence sur un entier
   contenant l'heure en secondes *)
val create_time_select :
  int ref -> (int -> unit) option -> (GObj.widget -> unit) -> unit


(** {6 Fenetre d'affichage d'infos} *)


(** [gtk_tools_create_infos_win title width height] crée une fenetre d'affichage
   d'informations sous forme de labels.

   En retour est renvoyé un couple contenant la fonction d'ajout d'un nouveau texte
   ainsi que la fonction de fermeture de la fenetre *)
val create_infos_win :
  string -> int -> int -> (string -> unit) * (unit -> unit)


(** {6 Widget affichant le contenu d'un fichier texte} *)


(** [gtk_tools_display_file filename title width height tooltips font] affiche
   le contenu d'un fichier dans une fenetre *)
val display_file :
  string -> string -> int -> int -> GData.tooltips -> string option -> unit


(** {6 Combo box} *)


(** [gtk_tools_create_combo lst_items pack_method] crée une combo box *)
val create_combo_simple :
	string list -> (GObj.widget -> unit) -> GEdit.entry

(** [gtk_tools_combo_connect entry lst_items callback] connecte [callback] qui est
   appelé lors de la modification de la valeur dans la combo box *)
val combo_connect : GEdit.entry -> (string*'a) list -> ('a->unit) -> unit


(** {6 Widget calendrier} *)

(** [gtk_tools_calendar lst_dates callback_select only_available_dates_selectable
	init_with_last_available_date tooltips win pack_method] crée un widget
   calendrier permettant de choisir une date. Les paramètres sont :

   - [lst_dates] contient (éventuellement) la liste des dates autorisées sous
   la forme d'un entier (ex. : 20030721)
   - [callback_select] désigne la fonction appelée après sélection. La date choisie
   est passée en paramètre
   - [only_available_dates_selectable] indique si celles les dates autorisées sont
   selectionnables (ainsi, seules ces dernières peuvent appeler [callback_select])
   - [init_with_last_available_date] sert à l'initialisation du calendrier.
   Si cette valeur vaut vrai, le calendrier affiche le mois et l'année correspondant
   à la date la plus tardive parmi les dates autorisées. Sinon, le mois courant
   est affiché
   - [tooltips] désigne le système d'aide contextuelle
   - [win] correspond à la fenetre mère
   - [pack_method] indique où mettre le widget
 *)
val calendar : int list -> (int -> unit) -> bool -> bool ->
  GData.tooltips -> GWindow.window -> (GObj.widget -> unit) -> unit

(** [gtk_tools_calendar_window lst_dates callback_select only_available_dates_selectable
   init_with_last_available_date is_modal tooltips] crée une boite de dialogue
   affichant un calendrier et permettant de choisir une date.

   Les paramètres utilisés sont les suivants :

   - [lst_dates] contient (éventuellement) la liste des dates autorisées sous
   la forme d'un entier (ex. : 20030721)
   - [callback_select] désigne la fonction appelée après sélection. La date choisie
   est passée en paramètre
   - [only_available_dates_selectable] indique si celles les dates autorisées sont
   selectionnables (ainsi, seules ces dernières peuvent appeler [callback_select])
   - [init_with_last_available_date] sert à l'initialisation du calendrier.
   Si cette valeur vaut vrai, le calendrier affiche le mois et l'année correspondant
   à la date la plus tardive parmi les dates autorisées. Sinon, le mois courant
   est affiché
   - [is_modal] indique si la fenetre doit etre modale
 *)
val calendar_window : int list -> (int -> unit) -> bool -> bool -> bool ->
  GData.tooltips -> unit
