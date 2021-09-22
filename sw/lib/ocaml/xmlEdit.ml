(*
 * XML editor
 *
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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


let default_background = "white"

type gtkTreeViewDropPosition =
    GTK_TREE_VIEW_DROP_BEFORE
  | GTK_TREE_VIEW_DROP_AFTER
  | GTK_TREE_VIEW_DROP_INTO_OR_BEFORE
  | GTK_TREE_VIEW_DROP_INTO_OR_AFTER

external gtk_tree_view_get_drag_dest_row : 'a Gtk.obj -> Gtk.tree_path * gtkTreeViewDropPosition = "ml_gtk_tree_view_get_drag_dest_row"

open Printf

type tag = string
type attribute = string * string
type attributes = attribute list
type t = GTree.tree_store * GTree.view
type node = GTree.tree_store * Gtk.tree_path

let cols = new GTree.column_list
let attribute = cols#add Gobject.Data.string
let value = cols#add Gobject.Data.string

let model_of_attribs = fun () ->
  GTree.tree_store cols

let set_attr_value = fun (store:GTree.tree_store) row (a, v) ->
  store#set ~row ~column:attribute a;
  store#set ~row ~column:value v

let set_attributes = fun (store:GTree.tree_store) attribs ->
  List.iter
    (fun (a, v) ->
      let row = store#append () in
      set_attr_value store row (a,v))
    attribs

let attribs_of_model = fun (store:GTree.tree_store) ->
  let l = ref [] in
  store#foreach
    (fun _path row ->
      l := (store#get ~row ~column:attribute, store#get ~row ~column:value):: !l;
      false);
  List.rev !l


let editable_renderer = fun (model:GTree.tree_store) column ->
  let r = GTree.cell_renderer_text [`EDITABLE true] in
  let _ = r#connect#edited ~callback:
    (fun path s ->
      model#set ~row:(model#get_iter path) ~column s
    ) in
  r

let attribs_view = fun model ->
  let view = GTree.view ~model () in
  view#set_rules_hint true;
  let r = editable_renderer model attribute in
  let col = GTree.view_column ~title:"Attribute" ()
    ~renderer:(r, ["text",attribute]) in

  ignore (view#append_column col);
  col#set_max_width 100;

  let r = editable_renderer model value in
  let col = GTree.view_column ~title:"Value" () ~renderer:(r, ["text",value]) in
  ignore (view#append_column col);
  view#set_headers_visible false;
  view

type event = Deleted | Modified of attributes | New_child of node

let cols = new GTree.column_list
let tag_col = cols#add Gobject.Data.string
let attributes = cols#add Gobject.Data.caml
let event = cols#add Gobject.Data.caml
let background = cols#add Gobject.Data.string
let id = cols#add Gobject.Data.int

let string_of_attribs = fun attribs ->
  match attribs with
      ["PCData", data] -> data
    | _ ->
      String.concat " " (List.map (fun (a,v) -> sprintf "%s=\"%s\"" a v) attribs)

type id = int
let gen_id =
  let x = ref 0 in
  fun () -> incr x; !x

let set_xml = fun (store:GTree.tree_store) row xml ->
  store#set ~row ~column:tag_col (Xml.tag xml);
  store#set ~row ~column:background default_background;
  store#set ~row ~column:attributes (Xml.attribs xml);
  store#set ~row ~column:event (fun _ -> ());
  store#set ~row ~column:id (gen_id ())

let encode_crs =
  let r = Str.regexp "\n" in
  fun s ->
    Str.global_replace r "\\n" s

(** Doesn' work. OCaml bug ?
    let recode_crs =
    let r = Str.regexp "\\n" in
    fun s ->
    Str.global_replace r "\n" s
*)

let recode_crs = fun s ->
  let n = String.length s in
  let s' = Bytes.create n in
  let i = ref 0 and j = ref 0 in
  while !i < n do
    if !i < n-1 && s.[!i] == '\\' && s.[!i+1] == 'n' then begin
      Bytes.set s' (!j)  '\n';
      incr i
    end else
      Bytes.set s' (!j) s.[!i];
    incr i; incr j
  done;
  Bytes.to_string (Bytes.sub s' 0 !j)




let rec insert_xml = fun (store:GTree.tree_store) parent xml ->
  match xml with
      Xml.Element _ ->
        let row = store#append ~parent () in
        set_xml store row xml;
        List.iter (fun x -> insert_xml store row x) (Xml.children xml)
    | Xml.PCData data ->
      let row = store#append ~parent () in
      store#set ~row ~column:tag_col "PCData";
      store#set ~row ~column:background default_background;
      store#set ~row ~column:attributes ["PCData", encode_crs data];
      store#set ~row ~column:event (fun _ -> ());
      store#set ~row ~column:id (gen_id ())



let tree_model_of_xml = fun xml ->
  let store = GTree.tree_store cols in
  let row = store#append () in
  set_xml store row xml;
  List.iter (fun x -> insert_xml store row x) (Xml.children xml);
  store;;


let attrib_cell_data_func = fun format_attribs renderer (model:GTree.model) iter ->
  let value = model#get ~row:iter ~column:attributes in
  let bg = model#get ~row:iter ~column:background in
  renderer#set_properties [`TEXT (format_attribs value); `CELL_BACKGROUND bg]

let set_bg_color = fun renderer (model:GTree.model) iter ->
  let bg = model#get ~row:iter ~column:background in
  renderer#set_properties [`CELL_BACKGROUND bg]

let tree_view = fun format_attribs ?(edit=true) (model:GTree.tree_store) window ->
  let view = GTree.view ~model ~enable_search:edit ~reorderable:edit ~packing:window#add () in
  let r = GTree.cell_renderer_text [] in
  let col = GTree.view_column ~title:"Tag" () ~renderer:(r, ["text",tag_col]) in
  col#set_cell_data_func r (set_bg_color r);
  let _ = r#connect#edited ~callback:
    (fun path s ->
      model#set ~row:(model#get_iter path) ~column:tag_col s
    ) in
  ignore (view#append_column col);
  let r = GTree.cell_renderer_text [] in
  let col = GTree.view_column ~title:"Attributes" () ~renderer:(r, []) in
  col#set_cell_data_func r (attrib_cell_data_func format_attribs r);
  col#set_max_width 300;
  ignore (view#append_column col);
  view#set_headers_visible false;
  view

(** Returns the list of all the tags appearing in the given DTD element *)
let rec tags r = function
Dtd.DTDTag s -> s::r
  | Dtd.DTDPCData -> r
  | Dtd.DTDOptional dtd_child | Dtd.DTDZeroOrMore dtd_child | Dtd.DTDOneOrMore dtd_child ->
    tags r dtd_child
  | Dtd.DTDChoice dtd_childs | Dtd.DTDChildren dtd_childs ->
    List.fold_right (fun dc r -> tags r dc) dtd_childs r

(** Returns the list of tags of possible children of the given [tag] *)
let dtd_children = fun tag dtd ->
  let rec search = function
  Dtd.DTDElement (t,det)::_ when t = tag -> det
    | _::is -> search is
    | [] -> raise Not_found in
  match search dtd with
      Dtd.DTDChild dc ->
        tags [] dc
    | _ -> []


(** Make a submenu with labels from [labels]. Attach the generic [callback]
    which argument is the selected label *)
let submenu = fun ?(filter = fun _ -> true) menuitem ss connect ->
  let submenu = GMenu.menu () in
  List.iter
    (fun tag ->
      if filter tag then
        let menuitem = GMenu.menu_item ~label:tag ~packing:submenu#append () in
        let _c = menuitem#connect#activate ~callback:(fun () -> connect tag) in
        ())
    ss;
  menuitem#set_submenu submenu

(** Returns the compulsory attributes of a given tag *)
let required_attributes = fun tag dtd ->
  let rec filter = function
  Dtd.DTDAttribute (t, a, _, (Dtd.DTDDefault s|Dtd.DTDFixed s))::dis when t = tag -> (a,s)::filter dis
    | Dtd.DTDAttribute (t, a, _, Dtd.DTDRequired)::dis when t = tag -> (a,"???")::filter dis
    | _::dis -> filter dis
    | [] -> [] in
  filter dtd

let allowed_attributes = fun tag dtd ->
  let rec filter = function
    | Dtd.DTDAttribute (t, a, _, _)::dis when t = tag -> a::filter dis
    | _::dis -> filter dis
    | [] -> [] in
  filter dtd

let attr_submenu = fun ?filter menuitem tag dtd connect ->
  submenu ?filter menuitem (allowed_attributes tag dtd) connect


let selection = fun (tree_store, tree_view) ->
  match tree_view#selection#get_selected_rows with
      path::_ ->
        tree_store, path
    | _ -> raise Not_found

let attribs_menu_popup = fun dtd (tree_view:GTree.view) (model:GTree.tree_store) (attrib_row:Gtk.tree_iter) ->
  let menu = GMenu.menu () in
  begin
    match tree_view#selection#get_selected_rows with
        path::_ ->
          let tree_model = tree_view#model in
          let row = tree_model#get_iter path in
          let current_tag = tree_model#get ~row ~column:tag_col in
          let menuitem = GMenu.menu_item ~label:"Add after" ~packing:menu#append () in
          let current_attrib = model#get ~row:attrib_row ~column:attribute in
          if not (List.mem_assoc current_attrib (required_attributes current_tag dtd)) then begin
            let menuitem = GMenu.menu_item ~label:"Delete" ~packing:menu#append () in
            ignore (menuitem#connect#activate ~callback:(fun () -> ignore (model#remove attrib_row)))
          end;

          let l = ref [] in
          model#foreach (fun _path row ->
            l := model#get ~row ~column:attribute :: !l; false);
          let filter = fun x -> not (List.mem x !l) in

          let connect = fun a ->
            let row = model#insert_after attrib_row in
            let av = (a, "???") in
            set_attr_value model row av in
          attr_submenu ~filter menuitem current_tag dtd connect
      | _ -> ()
  end;
  menu#popup ~button:1 ~time:(GtkMain.Main.get_current_event_time ())

let add_one_menu = fun dtd (tree_view:GTree.view) (model:GTree.tree_store) ->
  match tree_view#selection#get_selected_rows with
      path::_ ->
        let tree_model = tree_view#model in
        let row = tree_model#get_iter path in
        let current_tag = tree_model#get ~row ~column:tag_col in
        let menu = GMenu.menu () in
        let menuitem = GMenu.menu_item ~label:"Add one" ~packing:menu#append () in
        let connect = fun a ->
          let row = model#append () in
          let av = (a, "???") in
          set_attr_value model row av in
        attr_submenu menuitem current_tag dtd connect;
        menu#popup ~button:1 ~time:(GtkMain.Main.get_current_event_time ())
    | _ -> ()




let add_context_menu = fun model view ?noselection_menu menu ->
  view#event#connect#button_press ~callback:
    (fun ev ->
      GdkEvent.Button.button ev = 3
      &&
        match view#selection#get_selected_rows, noselection_menu with
            path::_, _ ->
              let row = model#get_iter path in
              menu model row;
              true
          | [], Some menu ->
            menu model;
            true
          | _ -> false)

let add_delete_key = fun (model:GTree.tree_store) (view:GTree.view) ->
  view#event#connect#key_press (fun ev ->
    if GdkEvent.Key.keyval ev = GdkKeysyms._Delete then
      match view#selection#get_selected_rows with
          path::_ ->
            let row = model#get_iter path in
            model#get ~row ~column:event Deleted;
            ignore (model#remove row);
            true
        | _ -> false
    else false)


let root = fun ((model:GTree.tree_store), _) ->
  match model#get_iter_first with
      None -> invalid_arg "XmlEdit.root"
    | Some i -> (model, model#get_path i)


let attribs = fun ((model, path):node) ->
  let row = model#get_iter path in
  model#get ~row ~column:attributes

let set_attribs = fun ((model, path):node) attribs ->
  let row = model#get_iter path in
  model#set ~row ~column:attributes attribs

let rec replace_assoc a v = function
[] -> [(a, v)]
  | (a', v')::l ->
    if a = String.uppercase_ascii a'
    then (a, v)::l
    else (a', v')::replace_assoc a v l

let set_attrib = fun node (a, v) ->
  let atbs = attribs node in
  set_attribs node (replace_assoc (String.uppercase_ascii a) v atbs)

let attrib = fun node at ->
  let at = String.uppercase_ascii at in
  let ats = attribs node in
  let rec loop = function
  [] -> raise Not_found
    | (a,v)::avs ->
      if String.uppercase_ascii a = at then v else loop avs in
  loop ats

let tag = fun ((model, path):node) ->
  let row = model#get_iter path in
  model#get ~row ~column:tag_col

let children = fun ((model, path):node) ->
  let row = model#get_iter path in
  if model#iter_has_child row then
    let i = model#iter_children (Some row) in
    let l = ref [model, model#get_path i] in
    while model#iter_next i do
      l := (model, model#get_path i):: !l;
    done;
    List.rev !l
  else
    []

let rec xml_of_node = fun (node:node) ->
  let attrs = attribs node
  and tag = tag node in
  if tag = "PCData" then
    match attrs with
        ["PCData", data] -> Xml.PCData (sprintf "\n%s\n" (recode_crs data))
      | _ -> failwith (sprintf "Wrong data in %s\n" tag)
  else
    let children = List.map xml_of_node (children node) in
    Xml.Element (tag, List.sort compare attrs, children)

let xml_of_view = fun (tree:t) ->
  xml_of_node (root tree)

let child = fun ((model, path):node) (t:string) ->
  let row = model#get_iter path in
  if model#iter_has_child row then
    let i = model#iter_children (Some row) in
    let rec loop = fun () ->
      if model#get ~row:i ~column:tag_col = t then
        (model, model#get_path i)
      else if model#iter_next i then
        loop ()
      else failwith (sprintf "XmlEdit.child: %s" t) in
    loop ()
  else
    failwith (sprintf "XmlEdit.child: %s" t)

let rec parent = fun ((model, path):node) (t:string) ->
  let row = model#get_iter path in
  let tag = model#get ~row ~column:tag_col in
  if tag = t then
    (model, path)
  else
    match model#iter_parent row with
        None -> failwith (sprintf "XmlEdit.parent: %s" t)
      | Some p ->
        parent (model, model#get_path p) t


let delete = fun (model, path) ->
  let row = model#get_iter path in
  if model#iter_is_valid row then
    ignore (model#remove row)

let add_child = fun ((model, path):node) tag attribs ->
  let parent = model#get_iter path in
  let row = model#append ~parent () in
  set_xml model row (Xml.Element (tag, attribs, []));
  model, model#get_path row

let id = fun ((model, path):node) ->
  let row = model#get_iter path in
  model#get ~row ~column:id

let connect = fun ((model, path):node) cb ->
  let row = model#get_iter path in
  let current_cb = try model#get ~row ~column:event with _ -> fun _ -> () in
  model#set ~row ~column:event (fun e -> cb e; current_cb e)

let activated_cbs = Hashtbl.create 3
let connect_activated = fun (model,_) cb ->
  let l = try Hashtbl.find activated_cbs model with Not_found -> [] in
  Hashtbl.replace activated_cbs model (cb::l)



let expand_node = fun ?all (_, (tree_view:GTree.view)) ((model, path):node) ->
  tree_view#expand_row ?all path

let rec set_background = fun ?(all=false) ((model, path):node) color ->
  let row = model#get_iter path in
  model#set ~row ~column:background color;
  if all then
    List.iter (fun x -> set_background ~all x color) (children (model,path))



let tree_menu_popup = fun dtd (model:GTree.tree_store) (row:Gtk.tree_iter) ->
  let menu = GMenu.menu () in
  let menuitem = GMenu.menu_item ~label:"Delete" ~packing:menu#append () in
  ignore (menuitem#connect#activate ~callback:(fun () -> ignore (model#get ~row ~column:event Deleted; model#remove row)));
  let row_tag = model#get ~row ~column:tag_col in
  let tags = dtd_children row_tag dtd in
  if tags <> [] then begin
    let menuitem = GMenu.menu_item ~label:"Add child" ~packing:menu#append () in
    let connect = fun t ->
      let parent = row in
      let row = model#append ~parent () in
      let attrs = required_attributes t dtd in
      let xml = Xml.Element (t, attrs, []) in
      set_xml model row xml;
      model#get ~row:parent ~column:event (New_child (model, model#get_path row)) in
    submenu menuitem tags connect
  end;
  begin
    match model#iter_parent row with
        Some parent ->
          let copy = fun () ->
            let xml = xml_of_node (model,(model#get_path row)) in
            let row = model#insert_after ~parent row in
            set_xml model row xml;
            model#get ~row:parent ~column:event (New_child (model, model#get_path row));
            List.iter (insert_xml model row) (Xml.children xml)
          in
          let menuitem = GMenu.menu_item ~label:"Copy after" ~packing:menu#append () in
          ignore (menuitem#connect#activate ~callback:copy);

          let menuitem = GMenu.menu_item ~label:"Add after" ~packing:menu#append () in
          let parent_tag = model#get ~row:parent ~column:tag_col in
          let connect = fun t ->
            let row = model#insert_after ~parent row in
            let attrs = required_attributes t dtd in
            let xml = Xml.Element (t, attrs, []) in
            set_xml model row xml;
            model#get ~row:parent ~column:event (New_child (model, model#get_path row))
          in
          let tags = dtd_children parent_tag dtd in
          submenu menuitem tags connect
      | _ -> ()
  end;
  menu#popup ~button:1 ~time:(GtkMain.Main.get_current_event_time ())




let create = fun ?(format_attribs = string_of_attribs) ?(editable=true) ?(width = 400) dtd xml ->
  let tree_model = tree_model_of_xml xml in
  let attribs_model = model_of_attribs () in
  let hbox = GPack.hbox () in
  let sw = GBin.scrolled_window ~width ~hpolicy:`AUTOMATIC
    ~vpolicy:`AUTOMATIC ~packing:hbox#add () in
  let tree_view = tree_view format_attribs ~edit:editable tree_model sw in
  tree_view#set_border_width 10;

  let sw = GBin.scrolled_window ~width:150 ~hpolicy:`AUTOMATIC
    ~vpolicy:`AUTOMATIC () in
  let attribs_view = attribs_view attribs_model in
  attribs_view#set_border_width 10;
  sw#add attribs_view#coerce;
  if editable then
    hbox#add sw#coerce;

  let update_tree = fun _path ->
    match tree_view#selection#get_selected_rows with
        path::_ ->
          let row = tree_model#get_iter path in
          let new_attribs = attribs_of_model attribs_model in
          tree_model#set ~row ~column:attributes new_attribs;
          tree_model#get ~row ~column:event (Modified new_attribs)
      | _ -> ()
  in
  let _attribs_changed = attribs_model#connect#row_changed ~callback:(fun p _i -> update_tree p) in
  ignore (attribs_model#connect#row_deleted ~callback:update_tree);

  let tag_of_last_selection = ref "" in

  let selection_changed = fun () ->
    match tree_view#selection#get_selected_rows with
        path::_ ->
          let row = tree_model#get_iter path in
          let attribs = tree_model#get ~row ~column:attributes in
          attribs_model#clear ();
          tag_of_last_selection := tree_model#get ~row ~column:tag_col;
          set_attributes attribs_model attribs
      | _ -> () in

  let _c = tree_view#selection#connect#after#changed ~callback:selection_changed in

  let _ = tree_view#connect#after#row_activated ~callback:
    (fun path vcol ->
      let cbs = try  (Hashtbl.find activated_cbs tree_model) with Not_found -> [] in
      List.iter (fun cb -> cb (tree_model, path)) cbs) in

  if editable then begin
    let _c = add_context_menu tree_model tree_view (tree_menu_popup dtd) in
    let _c = add_context_menu attribs_model attribs_view ~noselection_menu:(add_one_menu dtd tree_view) (attribs_menu_popup dtd tree_view) in

    ignore (add_delete_key tree_model tree_view);
    ignore (add_delete_key attribs_model attribs_view)
  end;

  (* Controlled drag and drop.
     Handling of dropable row cannot be done inside motion handling since
     the context refers to the whole widget, not the current row. The trick
     here is to use a boolean, set during motion and checked in drop event *)
  let dropable = ref false in
  let motion = fun _context ~x ~y ~time ->
    try
      let path, i = gtk_tree_view_get_drag_dest_row tree_view#as_widget in
      let row = tree_model#get_iter path in
      let row_tag = (tree_model#get ~row ~column:tag_col) in
      dropable := begin
        match i with
            GTK_TREE_VIEW_DROP_INTO_OR_BEFORE
          | GTK_TREE_VIEW_DROP_INTO_OR_AFTER ->
            List.mem !tag_of_last_selection (dtd_children row_tag dtd)
          | _ ->
            match tree_model#iter_parent row with
                None -> false
              | Some parent ->
                let parent_tag = tree_model#get ~row:parent ~column:tag_col in
                List.mem !tag_of_last_selection (dtd_children parent_tag dtd)
      end;
      false
    with
        Gpointer.Null -> false in
  let drop = fun (context:GObj.drag_context) ~x ~y ~time ->
    if !dropable then
      false
    else begin
      context#status None;
      true
    end in
  let _ = tree_view#drag#connect#motion ~callback:motion in
  let _ = tree_view#drag#connect#drop ~callback:drop in
  (tree_model, tree_view), hbox#coerce
