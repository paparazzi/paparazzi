(*
 * Multi aircrafts receiver, logger and broadcaster
 *
 * Copyright (C) 2005 CENA/ENAC, Pascal Brisset, Antoine Drouin
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

open Printf

let list_sort = fun f l -> List.sort (fun x y -> compare (f x) (f y)) l

let display_delay = 500 (* Time in milliseconds between two updates *)
let led_delay = 500 (* Time in milliseconds while the green led is displayed *)

let dnd_targets = [ { Gtk.target = "STRING"; flags = []; info = 0} ]

let pipe_regexp = Str.regexp "|"
let values_of_field = fun field ->
  try
    Array.of_list (Str.split pipe_regexp (Xml.attrib field "values"))
  with
      _ -> [||]

(** Display one page for a message *)
let one_page = fun sender class_name (notebook:GPack.notebook) bind m ->
  let id = (Xml.attrib m "name") in
  let h = GPack.hbox () in
  h#misc#set_property "name" (`STRING (Some id));
  let v = GPack.vbox ~width:200 () in

  let _l = GMisc.label ~text:id ~packing:h#add () in
  let eb = GBin.event_box ~packing:h#pack () in
  let time = GMisc.label ~width:40 ~packing:eb#add () in
  eb#coerce#misc#modify_bg [`SELECTED, `NAME "green"];
  let fields =
    List.fold_left
      (fun rest f ->
        try
          let unit = try "("^Xml.attrib f "unit"^")" with _ -> "" in
          let field_name =  Xml.attrib f "name" in
          let type_ = ExtXml.attrib f "type" in
          let name = Printf.sprintf "%s %s %s: " type_ field_name unit in
          let h = GPack.hbox ~packing:v#pack () in
          let field_label = GButton.button ~label:name ~packing:h#pack () in

          let value = ref "XXXX" in
          let l = GMisc.label ~text: !value ~packing:h#pack () in
          let literal_values = values_of_field f in
          let alt_value =
            try
              let coeff = float_of_string (Pprz.alt_unit_coef_of_xml ~auto:"display" f)
              and unit = Xml.attrib f "alt_unit" in
              fun value -> sprintf "%s (%f%s)" value (coeff*.float_of_string value) unit
            with
                _ -> fun value -> value in
          let update = fun (_a, x) ->
            value :=
              try
                let i = Pprz.int_of_value x in
                sprintf "%s (%d)" literal_values.(i) i
              with _ ->
                alt_value (Pprz.string_of_value x)
          and display_value = fun () ->
            if notebook#page_num v#coerce = notebook#current_page then
              if l#label <> !value then l#set_text !value in

          (* box dragger *)
          field_label#drag#source_set dnd_targets ~modi:[`BUTTON1] ~actions:[`COPY];
          let data_get = fun _ (sel:GObj.selection_context) ~info ~time ->
            let scale = Pprz.alt_unit_coef_of_xml ~auto:"display" f in
            let field_descr =
              if Pprz.is_array_type type_ then
                match GToolbox.input_string ~title:"Index of value to drag" ~text:"0" "Index in the array ?" with
                    None -> field_name
                  | Some i -> sprintf "%s[%s]" field_name i
              else
                field_name in
            sel#return (sprintf "%s:%s:%s:%s:%s" sender class_name id field_descr scale) in
          ignore (field_label#drag#connect#data_get ~callback:data_get);

          (update, display_value)::rest
        with
            _ ->
              fprintf stderr "Warning: Ignoring '%s'\n%!" (Xml.to_string f);
              rest
      )
      []
      (Xml.children m)
  in
  let (update_values, display_values) = List.split fields in
  let n = List.length fields in
  let time_since_last = ref 0 in
  let shown = ref false in
  let update_time = fun () ->
    incr time_since_last;
    let t = sprintf "%2d" !time_since_last in
    if !shown && notebook#page_num v#coerce = notebook#current_page && time#text <> t then begin
      time#set_text t
    end;
    true in
  ignore (GMain.Timeout.add 1000 update_time);
  let display_values = fun () ->
    List.iter (fun f -> f ()) display_values in
  ignore (GMain.Timeout.add display_delay (fun () -> display_values (); true));

  (** The message is shown only on its first arrival *)
  let display = fun _sender values ->
    if not !shown then begin
      shown := true;
      (** Look for the right position in alphabetic order *)
      let rec loop = fun i ->
        let p = notebook#get_nth_page i in
        let t = notebook#get_tab_label p in
        match t#misc#get_property "name" with
          | `STRING (Some x) -> if x < id then loop (i+1) else i
          | _ -> raise Not_found
      in
      try
        let pos = loop 0 in
        ignore (notebook#insert_page ~pos ~tab_label:h#coerce v#coerce)
      with _ ->
        ignore (notebook#append_page ~tab_label:h#coerce v#coerce)
    end;
    time_since_last := 0;
    try
      List.iter2 (fun f x -> f x) update_values (List.rev values);

      eb#coerce#misc#set_state `SELECTED;
      ignore (GMain.Timeout.add led_delay (fun () -> eb#coerce#misc#set_state `NORMAL; false))
    with
        Invalid_argument "List.iter2" ->
          Printf.fprintf stderr "%s: expected %d args, got %d\n" id n (List.length values); flush stderr
      | exc -> prerr_endline (Printexc.to_string exc)
  in
  bind id display

let rec one_class = fun (notebook:GPack.notebook) (ident, xml_class, sender) ->
  let class_name = (Xml.attrib xml_class "name") in
  let messages = Xml.children xml_class in
  let module P = Pprz.Messages (struct let name = class_name end) in
  let senders = Hashtbl.create 5 in
  match sender with
    | Some "*" ->
      (* Waiting for a new sender in this class *)
      let get_one = fun sender _vs ->
        if not (Hashtbl.mem senders sender) then begin
          Hashtbl.add senders sender ();
          one_class notebook (ident,  xml_class, Some sender)
        end in
      List.iter
        (fun m -> ignore (P.message_bind (Xml.attrib m "name") get_one))
        messages
    | _ ->
      let class_notebook = GPack.notebook ~tab_border:0 ~tab_pos:`LEFT () in
      let l = match sender with None -> "" | Some s -> ":"^s in
      let label = GMisc.label ~text:(ident^l) () in
      ignore (notebook#append_page ~tab_label:label#coerce class_notebook#coerce);
      let bind, sender_name = match sender with
          None -> (fun m cb -> (P.message_bind m cb)), "*"
        | Some sender -> (fun m cb -> (P.message_bind ~sender m cb)), sender in

      (** Forall messages in the class *)
      let messages = list_sort (fun x -> Xml.attrib x "name") messages in
      List.iter (fun m -> ignore (one_page sender_name class_name class_notebook bind m)) messages




(*********************** Main ************************************************)
let _ =
  let ivy_bus = ref Defivybus.default_ivy_bus in
  let classes = ref ["telemetry:*"] in
  Arg.parse
    [ "-b", Arg.String (fun x -> ivy_bus := x), (sprintf "<ivy bus> Default is %s" !ivy_bus);
      "-c",  Arg.String (fun x -> classes := x :: !classes), "class name"]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi messages" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (** Open the window container with its notebook*)
  let icon = GdkPixbuf.from_file Env.icon_mes_file in
  let window = GWindow.window ~type_hint:`DIALOG ~icon ~title:"Messages" () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let notebook = GPack.notebook ~packing:window#add ~tab_pos:`TOP () in

  (** Get the XML description of the required classes *)
  let xml_classes =
    let xml = Pprz.messages_xml () in
    let class_of = fun n ->
      try
        List.find (fun x -> ExtXml.attrib x "name" = n) (Xml.children xml)
      with Not_found -> failwith (sprintf "Unknown messages class: %s" n) in

    List.map (fun x ->
      match Str.split (Str.regexp ":") x with
          [cl; s] -> (cl, class_of cl, Some s)
        | [cl] -> (x, class_of cl, None)
        | _ -> failwith (sprintf "Wrong class '%s', class[:sender] expected" x))
      !classes in

  (* Insert the message classes in the notebook *)
  List.iter (one_class notebook) xml_classes;

  (** Start the main loop *)
  window#show ();
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
