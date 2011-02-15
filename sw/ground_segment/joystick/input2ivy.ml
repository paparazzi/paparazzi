(*
 * $Id$
 *
 * Forwarding events from an USB input device to the ivy bus
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

open Printf

let (//) = Filename.concat
let conf_dir = Env.paparazzi_home // "conf"
let verbose = ref false (* Command line option *)

(** Messages libraries *)
module DL = Pprz.Messages(struct let name = "datalink" end)
module G = Pprz.Messages(struct let name = "ground" end)

(** Syntax for expressions *)
module Syntax = Expr_syntax

(** Hash table of name-index associations for the settings of the A/C *)
let index_of_settings = Hashtbl.create 13

(**Hash table of name-index associations for the flightplan blocks of the A/C*)
let index_of_blocks = Hashtbl.create 13

(** External C functions to access the input device *)
external stick_init : string -> int = "ml_stick_init"
(** [stick_init device] Return 0 on success. Search for a device if [device]
 is the empty string *)

external stick_read : unit -> int * int * int array = "ml_stick_read"
(** Return the number of buttons, an integer of bits for the buttons values
    and an array of signed integers for the axis *)

(** Range for the input axis *)
let max_input = 127
let min_input = -127

(** Representation of an input value *)
type input =
    Axis of int * int (* (index, deadband) *)
  | Button of int

(** Description of a message *)
type msg = {
    msg_name : string;
    msg_class : string;
    fields : (string * Syntax.expression) list;
    on_event : Syntax.expression option
  }

(** Represenation of an input device and of the messages to send *)
type actions = {
    period_ms : int;
    inputs : (string*input) list;
    messages : msg list
  }

(** Get a message description from its name (and class name) *)
let get_message = fun class_name msg_name ->
  match class_name with
    "datalink" -> snd (DL.message_of_name msg_name)
  | "ground" -> snd (G.message_of_name msg_name)
  | _ -> failwith class_name

(** Get the A/C id from its name in conf/conf.xml *)
let ac_id_of_name = fun ac_name ->
  let conf_xml = Xml.parse_file (conf_dir // "conf.xml") in
  try
    let aircraft = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = ac_name) conf_xml "aircraft" in
    ExtXml.int_attrib aircraft "ac_id"
  with
    Not_found ->
      failwith (sprintf "A/C '%s' not found" ac_name)

(** Fill the index_of_settings table from var/AC/settings.xml *)
let hash_index_of_settings = fun ac_name ->
  let xml_file = Env.paparazzi_home // "var" // ac_name // "settings.xml" in
  let xml = Xml.parse_file xml_file in
  let index = ref 0 in
  let rec loop = fun xml ->
    if Xml.tag xml = "dl_settings" then
      List.iter loop (Xml.children xml)
    else begin (* dl_setting *)
      Hashtbl.add index_of_settings (Xml.attrib xml "var") !index;
      incr index
    end in
  loop (ExtXml.child xml "dl_settings")


(** Fill the index_of_blocks table from var/AC/flight_plan.xml *)
let hash_index_of_blocks = fun ac_name ->
  let xml_file = Env.paparazzi_home // "var" // ac_name // "flight_plan.xml" in
  let dump = Xml.parse_file xml_file in
  let flight_plan = ExtXml.child dump "flight_plan" in
  let blocks = ExtXml.child flight_plan "blocks" in
  List.iter (fun block ->
    Hashtbl.add
      index_of_blocks
      (Xml.attrib block "name") 
      (ExtXml.int_attrib block "no"))
    (Xml.children blocks)


(* Return the rank of an element in a list, first is 0 *)
let rank = fun x l ->
  let rec loop i = function
      [] -> raise Not_found
    | y::ys -> if x = y then i else loop (i+1) ys in
  loop 0 l

(** Eval IndexOfEnum, IndexOfSetting and IndexOfBlock built-in functions
   in an expression *)
let eval_settings_and_blocks = fun field_descr expr ->
  let rec loop = function
    Syntax.Call ("IndexOfEnum", [Syntax.Ident enum]) -> begin
      try Syntax.Int (rank enum field_descr.Pprz.enum) with
	Not_found -> failwith (sprintf "IndexOfEnum: unknown value '%s'" enum)
    end
  | Syntax.Call ("IndexOfSetting", [Syntax.Ident var]) -> begin
      try Syntax.Int (Hashtbl.find index_of_settings var) with
	Not_found -> failwith (sprintf "IndexOfSetting: unknown var '%s'" var)
    end
  | Syntax.Call ("IndexOfBlock", [Syntax.Ident name]) -> begin
      try Syntax.Int (Hashtbl.find index_of_blocks name) with
	Not_found -> failwith (sprintf "IndexOfBlock: unknown block '%s'" name)
    end
  | Syntax.Call (ident, exprs) | Syntax.CallOperator (ident, exprs) ->
      Syntax.Call (ident, List.map loop exprs)
  | e -> e in
  loop expr

(** Parse an XML list of input channels *)
let parse_input = fun input ->
  List.map
    (fun x ->
      let name = Xml.attrib x "name"
      and index = ExtXml.int_attrib x "index" in
      let value =
	match Xml.tag x with
	  "axis" -> 
             let deadband = try ExtXml.int_attrib x "deadband" with _ -> 0 in
             Axis (index, deadband)
	| "button" -> Button index
	| _ -> failwith "parse_input: unexepcted tag" in
      (name, value))
    (Xml.children input)

(** Parse a 'à la C' expression *)
let parse_value = fun s ->
  Fp_proc.parse_expression s

(** Parse a message field and eval *)
let parse_msg_field = fun msg_descr field ->
  let name = Xml.attrib field "name" in
  let field_descr = List.assoc name msg_descr.Pprz.fields in

  let value = eval_settings_and_blocks field_descr (parse_value (Xml.attrib field "value")) in
  (name, value)

(** Parse a complete message and build its representation *)
let parse_msg = fun msg ->
  let msg_name = Xml.attrib msg "name"
  and msg_class = Xml.attrib msg "class" in

  let msg_descr = get_message msg_class msg_name in

  let fields = List.map (parse_msg_field msg_descr) (Xml.children msg) in

  let on_event =
    try Some (parse_value (Xml.attrib msg "on_event")) with _ -> None in

  { msg_name = msg_name;
    msg_class = msg_class;
    fields = fields;
    on_event = on_event
  }

(** Parse the complete (input and messages) XML desxription *)
let parse_descr = fun xml_file ->
  let xml = Xml.parse_file xml_file in

  let inputs = parse_input (ExtXml.child xml "input")
  and messages_xml = ExtXml.child xml "messages" in

  let period_ms =truncate (1000.*.ExtXml.float_attrib messages_xml "period")
  and messages = List.map parse_msg (Xml.children messages_xml) in

  { period_ms = period_ms; inputs = inputs; messages = messages }

(** Verbose List.assco *)
let my_assoc = fun x l ->
  try List.assoc x l with Not_found ->
    failwith (sprintf "my_assoc: %s not found" x)

let apply_deadband = fun x min ->
  if abs x < min then 0 else x

(** Access to an input value, button or axis *)
let eval_input = fun buttons axis input ->
  match input with
    Axis (i, deadband) -> apply_deadband axis.(i) deadband
  | Button i -> (buttons lsr i) land 0x1

(** Scale a value in the given bounds *)
let scale = fun x min max ->
  min + ((x - min_input) * (max - min)) / (max_input - min_input)

(** Eval a function call (TO BE COMPLETED) *)
let eval_call = fun f args ->
  match f, args with
    "-", [a1; a2] -> a1 - a2
  | "+", [a1; a2] -> a1 + a2
  | "*", [a1; a2] -> a1 * a2
  | "&&", [a1; a2] -> a1 land a2
  | "Scale", [x; min; max] -> scale (x) (min) (max)
  | f, args -> failwith (sprintf "eval_call: unknown function '%s'" f)

(** Eval an expression *)
let eval_expr = fun buttons axis inputs expr ->
  let rec eval = function
      Syntax.Ident ident ->
	let input = my_assoc ident inputs in
	eval_input buttons axis input
    | Syntax.Int int -> int
    | Syntax.Float float -> failwith "eval_expr: float"
    | Syntax.Call (ident, exprs) | Syntax.CallOperator (ident, exprs) ->
	eval_call ident (List.map eval exprs)
    | Syntax.Index _ -> failwith "eval_expr: index"
    | Syntax.Field _ -> failwith "eval_expr: Field" in

  eval expr
  
(** Store for the last sent values: msg_name->values *)
let last_values = Hashtbl.create 5

(** Get the previous sent values for a given message *)
let get_previous_values = fun msg_name ->
  try Hashtbl.find last_values msg_name with Not_found -> (false, [])

(** Record the current values for a given message *)
let record_values = fun msg_name values ->
  Hashtbl.replace last_values msg_name values

(**Send an ivy message if needed: new values and/or 'on_event' condition true*)
let execute_action = fun ac_id inputs buttons axis message ->
  let values =
    List.map
      (fun (name, expr) -> (name, Pprz.Int (eval_expr buttons axis inputs expr)))
      message.fields

  and on_event = 
    match message.on_event with
      None -> true
    | Some expr -> eval_expr buttons axis inputs expr <> 0 in

  let previous_values = get_previous_values message.msg_name in
  if (on_event, values) <> previous_values && on_event then begin
    let vs = ("ac_id", Pprz.Int ac_id) :: values in
    match message.msg_class with
      "datalink" -> DL.message_send "input2ivy" message.msg_name vs
    | "ground" -> G.message_send "input2ivy" message.msg_name vs
    | c -> failwith (sprintf "execute_action: unknown class '%s'" c)
  end;
  record_values message.msg_name (on_event, values)


(** Output on stderr the values from the input device *)
let print_inputs = fun nb_buttons buttons axis ->
  fprintf stderr "buttons: ";
  for i = 0 to nb_buttons - 1 do
    fprintf stderr "%d:%d " i (eval_input buttons axis (Button i))
  done;
  fprintf stderr "\naxis: ";
  for i = 0 to Array.length axis - 1 do
    fprintf stderr "%d:%d " i (eval_input buttons axis (Axis (i, 0)))
  done;
  fprintf stderr "\n%!"
  

(** Get the values from the input values and send messages *)
let execute_actions = fun actions ac_id ->
  try
    let (nb_buttons, buttons, axis) = stick_read () in

    if !verbose then
      print_inputs nb_buttons buttons axis;
    
    List.iter (execute_action ac_id actions.inputs buttons axis) actions.messages
  with
    exc -> prerr_endline (Printexc.to_string exc)



(************************************* MAIN **********************************)
let () =
let ivy_bus = Defivybus.default_ivy_bus  in
  let device_name = ref ""
  and ac_name = ref "MYAC"
  and xml_descr = ref "" in

  let anon_fun = (fun x -> xml_descr := x) in
  let speclist =  
    [ "-b", Arg.String (fun x -> ivy_bus := x),(sprintf "<ivy bus> Default is %s" !ivy_bus);
      "-ac",  Arg.Set_string ac_name, "<A/C name>";
      "-d",  Arg.Set_string device_name, "<device name>";
      "-v",  Arg.Set verbose, "Verbose mode (useful to identify the channels of an input device)";
      "-", Arg.String anon_fun, "<xml file of actions>"
    ]
  and usage_msg = "Usage: " in

  Arg.parse speclist anon_fun usage_msg;

  if !xml_descr = "" then begin
    Arg.usage speclist usage_msg;
    exit 1
  end;

  let ac_id = ac_id_of_name !ac_name in

  hash_index_of_settings !ac_name;
  hash_index_of_blocks !ac_name;

  let actions = parse_descr !xml_descr in

  if stick_init !device_name <> 0 then
    failwith (sprintf "Error: cannot open device %s\n" !device_name);

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi joystick" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  ignore (Glib.Timeout.add actions.period_ms (fun () -> execute_actions actions ac_id; true));
  
  (** Start the main loop *)
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done

