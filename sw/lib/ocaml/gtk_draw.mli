(** Module de dessin GTK

   {b Dépendences : Gtk_Tools}
 *)


(** {6 Fonctions utilitaires} *)


(** [gd_do_cast drawable] : fonction de casting d'un drawable ('window ou 'pixmap)
   pour pouvoir y dessiner *)
val gd_do_cast : GDraw.drawable -> GDraw.drawable


(** {6 Divers} *)


(** [gd_clear drawable color width height] efface une pixmap avec la couleur
   [color] en dessinant un rectangle plein de taille [width]*[height] *)
val gd_clear : GDraw.drawable -> GDraw.color -> int -> int -> unit

(** [gd_set_background_pixmap pixmap drawable] place la [pixmap] en fond de la
   zone de dessin [drawable] *)
val gd_set_background_pixmap : Gdk.pixmap -> GDraw.drawable -> unit

(** [gd_set_background_pixmap pixmap drawable x y] place la [pixmap] dans la zone
   de dessin à la position [(x, y)]. Cette pixmap est transparente *)
val gd_put_transp_pixmap : GDraw.pixmap -> GDraw.drawable -> int -> int -> unit

(** {6 Transformations de couleurs} *)


(** [gd_rgb_of_color color] renvoie un triplet (r, g, b) donnant les composantes
   entières rouge, verte et bleue de [color]. Les composantes RGB sont dans
   l'intervalle [\[0, 65535\]] *)
val gd_rgb_of_color : GDraw.color -> int * int * int

(** [gd_float_rgb_of_color color] renvoie un triplet (r, g, b) donnant les
   composantes flottantes rouge, verte et bleue de [color] *)
val gd_float_rgb_of_color : GDraw.color -> float * float * float

(** [gd_color_of_rgb (r, g, b)] crée une [GDraw.color] à partir des composantes
   entières *)
val gd_color_of_rgb : 'a * 'b * 'c -> [> `RGB of 'a * 'b * 'c]

(** [gd_color_of_float_rgb (r, g, b)] crée une [GDraw.color] à partir des
   composantes flottantes *)
val gd_color_of_float_rgb :
  float * float * float -> [> `RGB of int * int * int]


(** {6 Modification du tracé} *)


(** [gd_set_color drawable color] met à jour la couleur de dessin *)
val gd_set_color : GDraw.drawable -> GDraw.color -> unit

(** [gd_set_faded_color drawable color pct] met à jour la couleur de dessin avec
   une couleur à [pct]% de [color] *)
val gd_set_faded_color : GDraw.drawable -> GDraw.color -> int -> unit

(** [gd_set_style_solid drawable] met le style de dessin normal *)
val gd_set_style_solid : GDraw.drawable -> unit

(** [gd_set_style_dash drawable] met le style de dessin pointillés *)
val gd_set_style_dash : GDraw.drawable -> unit

(** [gd_set_style_double_dash drawable] met le style de dessin en doubles
   pointillés *)
val gd_set_style_double_dash : GDraw.drawable -> unit

(** [gd_set_mode_xor drawable] fixe le mode dessin en XOR *)
(*val gd_set_mode_xor : GDraw.drawable -> unit*)

(** [gd_set_mode_std drawable] remet le dessin en mode normal *)
(*val gd_set_mode_std : GDraw.drawable -> unit*)

(** [gd_set_line_width drawable width] fixe l'épaisseur des lignes *)
val gd_set_line_width : GDraw.drawable -> int -> unit


(** {6 Texte} *)


(** [gd_center_text (x, y) chaine fonte] centre le texte contenu dans [chaine]
   a la position [(x, y)] *)
val gd_center_text : int * int -> string -> Gdk.font -> int * int

(** [gd_draw_non_centered_text drawable (x, y) chaine fonte] dessine le texte
   contenu dans [chaine] à la position [(x, y)] sans le centrer *)
val gd_draw_non_centered_text :
  GDraw.drawable -> int * int -> string -> Gdk.font -> unit

(** [gd_draw_text drawable (x, y) chaine fonte] dessine le texte
   contenu dans [chaine] centré sur la position [(x, y)] *)
val gd_draw_text :
  GDraw.drawable -> int * int -> string -> Gdk.font -> unit


(** {6 Formes géométriques} *)


(** [gd_draw_point drawable (x, y)] dessine un point à la position indiquée *)
val gd_draw_point : GDraw.drawable -> int * int -> unit

(** [gd_draw_segment drawable (x1, y1) (x2, y2)] dessine un segment entre les
   points [(x1, y1)] et [(x2, y2)] *)
val gd_draw_segment : GDraw.drawable -> int * int -> int * int -> unit

(** [gd_draw_line drawable liste_points] dessine une ligne *)
val gd_draw_line : GDraw.drawable -> (int * int) list -> unit

(** [gd_draw_polygon drawable liste_points] dessine un polygone non rempli *)
val gd_draw_polygon : GDraw.drawable -> (int * int) list -> unit

(** [gd_draw_filled_polygon drawable liste_points] dessine un polygone rempli *)
val gd_draw_filled_polygon : GDraw.drawable -> (int * int) list -> unit

(** [gd_draw_circle drawable (x, y) rayon] dessine un cercle non rempli *)
val gd_draw_circle : GDraw.drawable -> int * int -> int -> unit

(** [gd_draw_filled_circle drawable (x, y) rayon] dessine un cercle rempli *)
val gd_draw_filled_circle : GDraw.drawable -> int * int -> int -> unit

(** [gd_draw_rect drawable (x1, y1, x2, y2)] dessine un rectangle non rempli
   dont les points extrèmes sont [(x1, y1)] et [(x2, y2)] *)
val gd_draw_rect : GDraw.drawable -> int * int * int * int -> unit

(** [gd_draw_filled_rect drawable (x1, y1, x2, y2)] dessine un rectangle rempli
   dont les points extrèmes sont [(x1, y1)] et [(x2, y2)] *)
val gd_draw_filled_rect : GDraw.drawable -> int * int * int * int -> unit

(** [gd_draw_triangle drawable (x, y) size] dessine un triangle non rempli *)
val gd_draw_triangle : GDraw.drawable -> int * int -> int -> unit

(** [gd_draw_filled_triangle drawable (x, y) size] dessine un triangle rempli *)
val gd_draw_filled_triangle : GDraw.drawable -> int * int -> int -> unit
