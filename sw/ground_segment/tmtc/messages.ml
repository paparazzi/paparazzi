(*
 * $Id$
 *
 * Multi aircrafts receiver, logger and broadcaster
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

open Printf

let update_delay = 1. (* Min time in second before two updates *)
let led_delay = 500 (* Time in milliseconds while the green led is displayed *)


let space = Str.regexp "[ \t]+"

let (//) = Filename.concat

let xml_file = Env.paparazzi_src // "conf" // "messages.xml"

let green = "#00e000"
let red = "#ff0000"
let black = "#000000"
let yellow = "#ffff00"

let led2 color1 color2 = [|
  "16 16 5 1";
  "       c None";
  ".      c black";
  "X      c white";
  "o c "^color1;
  "O c "^color2;
  "     ......     ";
  "   ........XX   ";
  "  ...ooooooXXX  ";
  " ..ooooooooooXX ";
  " ..oooXXXooooXX ";
  "..oooXXXooooooXX";
  "..ooXXooooooooXX";
  "..ooXXooooooooXX";
  "..ooXoooooooooXX";
  "..ooooooooooooXX";
  "..ooooooooooooXX";
  " ..ooooooooooXX ";
  " ..ooooooooooXX ";
  "  ...ooooooXXX  ";
  "   ..XXXXXXXX   ";
  "     XXXXXX     "|]

let led color = led2 color "None"

let format = fun field ->
  try
    match Xml.attrib field "type", Xml.attrib field "format" with
      "float", f -> fun x -> Printf.sprintf (Obj.magic f) (float_of_string x)
    | _ -> fun x -> x
  with _ -> fun x -> x





let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
(*  let classes = ref ["telemetry_ap";"ground"] in *)
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

  let pm = fun color ->
    GDraw.pixmap_from_xpm_d ~data:(led color) ~window:window () in
  let black_led = pm black
  and green_led = pm green
  and yellow_led = pm yellow
  and red_led = pm red in

let one_page = fun (notebook:GPack.notebook) bind m ->
  let id = (Xml.attrib m "name") in
  let h = GPack.hbox () in
  let v = GPack.vbox ~width:200 () in
  let l = GMisc.label ~text:id ~packing:h#add () in
  let led = GMisc.pixmap black_led ~packing:h#pack () in
  let time = GMisc.label ~text:"___" ~packing:h#pack () in
  notebook#append_page ~tab_label:h#coerce v#coerce;
  let fields = 
    List.fold_right
      (fun f rest ->
	try
	  let unit = try "("^Xml.attrib f "unit"^")" with _ -> "" in
	  let name = Printf.sprintf "%s %s %s: " (ExtXml.attrib f "type") (Xml.attrib f "name") unit in
	  let h = GPack.hbox ~packing:v#pack () in
	  let _ = GMisc.label ~text:name ~packing:h#pack () in
	  let l = GMisc.label ~text:"XXXX" ~packing:h#pack () in
	  let update = fun (a, x) -> 
	    let fx = Pprz.string_of_value x in
	    if l#label <> fx then l#set_text fx in
	  update::rest
	with
	  _ -> 
	    fprintf stderr "Warning: Ignoring '%s'\n%!" (Xml.to_string f);
	    rest
      )
      (Xml.children m)
      []
  in
  let n = List.length fields in
  let last_update = ref (Unix.gettimeofday ()) in
  let time_since_last = ref 0 in
  ignore (GMain.Timeout.add 1000 (fun () -> incr time_since_last; time#set_text (sprintf "%2d" !time_since_last); true));
  let display = fun sender values ->
    time_since_last := 0;
    let t = Unix.gettimeofday () in
    if t > !last_update +. update_delay then begin
      last_update := t;
      try
	List.iter2 (fun f x -> f x) fields values;
	
	led#set_pixmap green_led;
 	ignore (GMain.Timeout.add led_delay (fun () -> led#set_pixmap yellow_led; false))
      with
	Invalid_argument "List.iter2" ->
	  led#set_pixmap red_led;
	  Printf.fprintf stderr "%s: expected %d, got %d\n" id n (List.length values); flush stderr
    end
  in
  ignore (bind id display);
  (id, (led, fields)) in


  let xml_classes =
    let class_of = fun n -> List.find (fun x -> ExtXml.attrib x "name" = n) (Xml.children xml) in
    List.map (fun x -> 
      match Str.split (Str.regexp ":") x with
	[cl; s] -> (x, class_of cl, Some s)
      | [cl] -> (x, class_of cl, None)
      | _ -> failwith (sprintf "Wrong class '%s', class[:sender] expected" x))
      !classes in

  List.iter
    (fun (ident, xml_class, sender) ->
      let name = (Xml.attrib xml_class "name") in
      let messages = Xml.children xml_class in
      let class_notebook = GPack.notebook ~tab_pos:`LEFT () in
      let label = GMisc.label ~text:ident () in
      notebook#append_page ~tab_label:label#coerce class_notebook#coerce;
      let module P = Pprz.Protocol (struct let name = name end) in
      let bind = match sender with
	None -> fun m cb -> P.message_bind m cb
      | Some sender -> fun m cb -> P.message_bind ~sender m cb in
      let pages = List.map (fun m -> one_page class_notebook bind m) messages in
      ())
    xml_classes;

  window#show ();
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
