(*
 * Lablgtk2 utils
 *
 * Copyright (C) 2009 ENAC, Pascal Brisset
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
(** GTK utilities
*)

(** Allocate a drawing area and filling pixmap on request.
    if ~drawing_area is provided, width, heigh and packing are ignored *)
class pixmap_in_drawin_area :
  ?drawing_area:GMisc.drawing_area ->
    ?width:int ->
      ?height:int ->
        ?packing:(GObj.widget -> unit) ->
          unit ->
object
  method drawing_area : GMisc.drawing_area

  method get_pixmap : unit -> GDraw.pixmap
    (** Lazyly allocate a pixmap filling the drawing area *)

  method redraw : unit -> unit
      (** Redraw the pixmap *)
end


(*** Utilities for a combo box widget ***)
type combo
val combo_widget : combo -> GEdit.combo_box
val combo_model : combo -> (GTree.list_store * string GTree.column)

val combo : ?width:int -> string list -> < add : GObj.widget -> unit; .. > -> combo

val add_to_combo : combo -> string -> unit
val combo_separator : string

val combo_value : combo -> string
val combo_values_list : combo -> string list
val select_in_combo : combo -> string -> unit
val combo_connect : combo -> (string -> unit) -> unit

(*** Utilities for a tree view widget ***)
type tree
val tree_widget : tree -> GTree.view
val tree_model : tree -> (GTree.list_store * string GTree.column * bool GTree.column * GTree.cell_renderer_toggle_signals)

val tree : ?check_box:bool -> GTree.view -> tree
val tree_of : GTree.view -> (GTree.list_store * string GTree.column * bool GTree.column * GTree.cell_renderer_toggle_signals) -> tree

val tree_values : ?only_checked:bool -> tree -> string
val get_selected_in_tree : tree -> GTree.row_reference list
val add_to_tree : ?force_unselect:bool -> tree -> string -> unit
val remove_selected_from_tree : tree -> unit
val clear_tree : tree -> unit

