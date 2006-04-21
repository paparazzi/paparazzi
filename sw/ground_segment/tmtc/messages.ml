(*
 * $Id$
 *
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

let update_delay = 0.25 (* Min time in second before two updates *)
let led_delay = 500 (* Time in milliseconds while the green led is displayed *)


let (//) = Filename.concat

let xml_file = Env.paparazzi_src // "conf" // "messages.xml"


let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let classes = ref [] in
  Arg.parse
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.255:2010";
      "-c",  Arg.String (fun x -> classes := x :: !classes), "class name"]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";


  Ivy.init "Paparazzi messages" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  let xml = Xml.parse_file xml_file in

  let window = GWindow.window ~title:"Paparazzi messages" () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let notebook = GPack.notebook ~packing:window#add ~tab_pos:`TOP () in


  let one_page = fun (notebook:GPack.notebook) bind m ->
    let id = (Xml.attrib m "name") in
    let h = GPack.hbox () in
    let v = GPack.vbox ~width:200 () in
    let _l = GMisc.label ~text:id ~packing:h#add () in
    let eb = GBin.event_box ~packing:h#pack () in
    let time = GMisc.label ~width:40 ~packing:eb#add () in
    eb#coerce#misc#modify_bg [`SELECTED, `NAME "green"];
    notebook#append_page ~tab_label:h#coerce v#coerce;
    let page_num = notebook#page_num v#coerce in
    let fields = 
      List.fold_left
	(fun rest f ->
	  try
	    let unit = try "("^Xml.attrib f "unit"^")" with _ -> "" in
	    let name = Printf.sprintf "%s %s %s: " (ExtXml.attrib f "type") (Xml.attrib f "name") unit in
	    let h = GPack.hbox ~packing:v#pack () in
	    let _ = GMisc.label ~text:name ~packing:h#pack () in
	    let l = GMisc.label ~text:"XXXX" ~packing:h#pack () in
	    let update = fun (_a, x) -> 
	      if notebook#current_page = page_num then
		let fx = Pprz.string_of_value x in
		if l#label <> fx then l#set_text fx in
	    update::rest
	  with
	    _ -> 
	      fprintf stderr "Warning: Ignoring '%s'\n%!" (Xml.to_string f);
	      rest
	)
	[]
	(Xml.children m)
    in
    let n = List.length fields in
    let last_update = ref (Unix.gettimeofday ()) in
    let time_since_last = ref 0 in
    let update_time = fun () ->
      incr time_since_last;
      time#set_text (sprintf "%2d" !time_since_last); true in
    ignore (GMain.Timeout.add 1000 update_time);
    let display = fun _sender values ->
      time_since_last := 0;
      let t = Unix.gettimeofday () in
      if t > !last_update +. update_delay then begin
	last_update := t;
	try
	  List.iter2 (fun f x -> f x) fields (List.rev values);
	  
	  eb#coerce#misc#set_state `SELECTED;
 	  ignore (GMain.Timeout.add led_delay (fun () -> eb#coerce#misc#set_state `NORMAL; false))
	with
	  Invalid_argument "List.iter2" ->
	    Printf.fprintf stderr "%s: expected %d, got %d\n" id n (List.length values); flush stderr
      end
    in
    bind id display in


  let xml_classes =
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

  let rec one_class = fun (ident, xml_class, sender) ->
    let name = (Xml.attrib xml_class "name") in
    let messages = Xml.children xml_class in
    let module P = Pprz.Messages (struct let name = name end) in
    let senders = Hashtbl.create 5 in
    match sender with
    | Some "*" ->
	(* Waiting for a new sender in this class *)
	let get_one = fun sender _vs ->
	  if not (Hashtbl.mem senders sender) then begin
	    Hashtbl.add senders sender ();
	    one_class (ident,  xml_class, Some sender)
	  end in
	List.iter 
	  (fun m -> ignore (P.message_bind (Xml.attrib m "name") get_one))
	  messages
    | _ -> 
	let class_notebook = GPack.notebook ~tab_pos:`LEFT () in
	let l = match sender with None -> "" | Some s -> ":"^s in
	let label = GMisc.label ~text:(ident^l) () in
(***	let label = GButton.button ~label:(ident^l) () in ***)
	notebook#append_page ~tab_label:label#coerce class_notebook#coerce;
	let bind = match sender with
	  None -> fun m cb -> P.message_bind m cb
	| Some sender -> fun m cb -> P.message_bind ~sender m cb in
	let _bindings = 
	  (** Forall messages in the class *)
	  List.map (fun m -> one_page class_notebook bind m) messages in
(***	label#connect#clicked ~callback:(fun () -> prerr_endline "clicked"); ***)
	() in
  
  List.iter one_class xml_classes;
  
  window#show ();
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
