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

class pixmap_in_drawin_area = fun ?drawing_area ?width ?height ?packing () ->
  let da =
    match drawing_area with
      None ->
	GMisc.drawing_area ?width ?height ~show:true ?packing ()
    | Some d -> d in
  object
    val mutable pixmap = None

    method drawing_area = da

    method redraw = fun () ->
      match pixmap with
	  None -> ()
	| Some pm ->
	    (new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 pm#pixmap

    method get_pixmap = fun () ->
      let {Gtk.width=width; height=height} = da#misc#allocation in
      let create = fun () -> GDraw.pixmap ~width ~height ~window:da () in
      let pm =
	match pixmap with
	  None -> create ()
	| Some pm ->
	    if pm#size = (width, height)
	    then pm
	    else begin
	      Gdk.Pixmap.destroy pm#pixmap;
	      create ()
	    end in
      pixmap <- Some pm;
      pm
  end

type combo = GEdit.combo_box * (GTree.list_store * string GTree.column)
let combo_widget = fst
let combo_model = snd

let combo_value = fun ((combo: #GEdit.combo_box), (_,column)) ->
  match combo#active_iter with
  | None -> raise Not_found
  | Some row -> combo#model#get ~row ~column

let combo_values_list = fun (combo : combo) ->
  let (store, column) = combo_model combo in
  let values = ref [] in
  store#foreach (fun _ row ->
    values := !values @ [store#get ~row ~column];
    false);
  !values

let combo_separator = "--"

let combo = fun strings vbox ->
  let (combo, (tree, column)) =
    GEdit.combo_box_text ~packing:vbox#add ~strings () in
  combo#set_active 0;
  combo#set_row_separator_func
    (Some (fun m row -> m#get ~row ~column = combo_separator)) ;
  (combo, (tree, column))

let add_to_combo = fun (combo : combo) string ->
  let (store, column) = combo_model combo in
  let row = store#append () in
  store#set ~row ~column string;
  (combo_widget combo)#set_active_iter (Some row)


let select_in_combo = fun  (combo : combo) string ->
  let (store, column) = combo_model combo in
  store#foreach
    (fun _path row ->
      if store#get ~row ~column = string then begin
	(combo_widget combo)#set_active_iter (Some row);
	true
      end else
	false)

let combo_connect = fun ((combo: #GEdit.combo_box), (_,column)) cb ->
  ignore (combo#connect#changed
	    (fun () ->
	      match combo#active_iter with
	      | None -> ()
	      | Some row ->
		  let data = combo#model#get ~row ~column in
		  cb data))


type tree = GTree.view * (GTree.list_store * string GTree.column)
let tree_widget = fst
let tree_model = snd

let tree = fun (t:GTree.view) ->
  let cols = new GTree.column_list in
  let col_name = cols#add Gobject.Data.string in
  let store = GTree.list_store cols in
  t#set_model (Some store#coerce);
  let col1 = GTree.view_column ~renderer:(GTree.cell_renderer_text [], ["text",col_name]) () in
  ignore (t#append_column col1);
  (t , (store, col_name))

let tree_of = fun (t:GTree.view) (m:(GTree.list_store * string GTree.column)) ->
  (t, m)

let tree_values = fun (tree : tree) ->
  let (store, column) = tree_model tree in
  let values = ref "" in
  store#foreach (fun _ row ->
    values := !values^" "^(store#get ~row ~column);
    false);
  !values

let get_selected_in_tree = fun  (tree : tree) ->
  let (store, column) = tree_model tree in
  let t = tree_widget tree in
  let sel_paths = t#selection#get_selected_rows in
  List.map (fun p -> store#get_row_reference p) sel_paths

let add_to_tree = fun (tree : tree) string ->
  let (store, column) = tree_model tree in
  let row = store#append () in
  store#set ~row ~column string

let remove_selected_from_tree = fun (tree : tree) ->
  let selected = get_selected_in_tree tree in
  let (store, _) = tree_model tree in
  List.iter (fun r -> ignore (store#remove r#iter)) selected

let clear_tree = fun (tree : tree) ->
  let (store, _) = tree_model tree in
  store#clear ()
