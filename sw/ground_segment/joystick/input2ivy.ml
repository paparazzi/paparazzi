(*
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

(*  1/26/2011 - Additional functionality added by jpeverill:
    Joystick xml config files now loaded from PAPARAZZI_HOME/conf/joystick/
    Exponential output setting (per channel)
    Limit output setting (per channel)
    Per channel trim, controlled through joystick buttons
    Trim can also be saved into auxilliary file, based on aircraft, and loaded at runtime if it exists
    File will be called <xml joystick profile name>.<A/C name>.trim and is saved in the conf/joystick directory as well
    Division in channel mapping
    Interactive keyboard trim control (primitive)
*)

open Printf
open Random


let (//) = Filename.concat
let conf_dir = Env.paparazzi_home // "conf"
let verbose = ref false (* Command line option *)
let axis_check = ref false (* Command line option *)

(** global trim file name *)
let trim_file_name = ref ""

(** global joystick id *)
let joystick_id = ref (Random.int 255)

(** Messages libraries *)
module DL = PprzLink.Messages(struct let name = "datalink" end)
module G = PprzLink.Messages(struct let name = "ground" end)

(** Syntax for expressions *)
module Syntax = Expr_syntax

(** Hash table of name-index associations for the settings of the A/C *)
let index_of_settings = Hashtbl.create 13

(**Hash table of name-index associations for the flightplan blocks of the A/C*)
let index_of_blocks = Hashtbl.create 13

(** External C functions to access the input device *)
external stick_init : int -> int = "ml_stick_init"
(** [stick_init device] Return 0 on success. Search for a device if [device]
    is the empty string *)

external stick_check_axis : unit -> int =  "ml_stick_check_axis"
(** Return 1 once all axis received events. Axis positions are unknown until an event has been received. *)

external stick_read : unit -> int * int * int * int array = "ml_stick_read"
(** Return the number of buttons, an integer of bits for the buttons values,
    an integer for the hat and an array of signed integers for the axis *)

(** Range for the input axis *)
let max_input = 127
let min_input = -127
let trim_step = 0.2

(** Representation of an input value *)
type input =
    Axis of int * int * float * float * float ref (* (index, deadband, limit, exponent, trim ) *)
  | Button of int
  | Hat of int

(** Description of a message *)
type msg = {
  msg_name : string;
  msg_class : string;
  fields : (string * PprzLink._type * Syntax.expression) list;
  on_event : Syntax.expression option;
  send_always : bool;
  has_ac_id : bool
}

(** Representation of a variable *)
type var = {
  mutable value : int;
  var_event : (int * Syntax.expression) list;
}

(** Represenation of an input device, the messages to send and the variables *)
type actions = {
  period_ms : int;
  inputs : (string*input) list;
  messages : msg list;
  variables : (string*var) list;
}

(** adjust the trim on an axis given its name *)
let trim_adjust = fun axis_name adjustment ->
  None

(** Get message class type *)
let get_message_type = fun class_name ->
  match class_name with
      "datalink" -> "Message"
    | "ground" -> "Message"
    | "trim_plus" -> "Trim"
    | "trim_minus" -> "Trim"
    | "trim_save" -> "Trim"
    | _ -> failwith class_name

(** Get a message description from its name (and class name) *)
(**   class_names with entries above as "Message" should be listed here  *)
let get_message = fun class_name msg_name ->
  match class_name with
      "datalink" -> snd (DL.message_of_name msg_name)
    | "ground" -> snd (G.message_of_name msg_name)
    | _ -> failwith class_name

(** Get the A/C id from its name in conf/conf.xml *)
let ac_id_of_name = fun ac_name ->
  let conf_xml = ExtXml.parse_file (conf_dir // "conf.xml") in
  try
    let aircraft = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = ac_name) conf_xml "aircraft" in
    ExtXml.int_attrib aircraft "ac_id"
  with
      Not_found ->
        if ac_name = "GCS" then
          0 (* return GCS id *)
        else
          failwith (sprintf "A/C '%s' not found" ac_name)

(** Fill the index_of_settings table from var/AC/settings.xml *)
let hash_index_of_settings = fun ac_name ->
  let xml_file = Env.paparazzi_home // "var" // "aircrafts" // ac_name // "settings.xml" in
  let xml = ExtXml.parse_file xml_file in
  let index = ref 0 in
  let rec loop = fun xml ->
    if Xml.tag xml = "dl_settings" then
      List.iter loop (Xml.children xml)
    else begin (* dl_setting *)
      Hashtbl.add index_of_settings (Xml.attrib xml "var") !index;
      incr index
    end in
  loop (ExtXml.child xml "dl_settings")


(** Fill the index_of_blocks table from var/aircrafts/AC/flight_plan.xml *)
let hash_index_of_blocks = fun ac_name ->
  let xml_file = Env.paparazzi_home // "var" // "aircrafts" // ac_name // "flight_plan.xml" in
  let dump = ExtXml.parse_file xml_file in
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
    try Syntax.Int (rank enum field_descr.PprzLink.enum) with
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
  List.map (fun x ->
    let name = Xml.attrib x "name"
    and index = ExtXml.int_attrib x "index" in
    let value =
      match Xml.tag x with
          "axis" ->
            let trim = try ExtXml.float_attrib x "trim" with _ -> 0.0 in
            let exponent = try ExtXml.float_attrib x "exponent" with _ -> 0.0 in
            let limit = try ExtXml.float_attrib x "limit" with _ -> 1.0 in
            let deadband = try ExtXml.int_attrib x "deadband" with _ -> 0 in
            Axis (index, deadband, limit, exponent, ref trim)
        | "button" -> Button index
        | "hat" -> Hat index
        | _ -> failwith "parse_input: unexepcted tag" in
    (name, value))
    (Xml.children input)

(** Parse a 'à la C' expression *)
let parse_value = fun s ->
  Fp_proc.parse_expression s

(** Parse a message field and eval *)
let parse_msg_field = fun msg_descr field ->
  let name = Xml.attrib field "name" in
  let field_descr = try List.assoc name msg_descr.PprzLink.fields with _ ->
    Printf.printf "parse_msg_field: field %s not found\n" name;
    raise (Failure "field not found") in

  let value = eval_settings_and_blocks field_descr (parse_value (Xml.attrib field "value")) in
  (name, field_descr.PprzLink._type, value)

(** Parse a complete message and build its representation *)
let parse_msg = fun msg ->
  let msg_name = Xml.attrib msg "name"
  and msg_class = Xml.attrib msg "class"
  and send_always = (try (Xml.attrib msg "send_always") = "true" with _ -> false) in

  let fields, has_ac_id =
    match get_message_type msg_class with
        "Message" ->
          begin
            let msg_descr = get_message msg_class msg_name in
            try
              (List.map (parse_msg_field msg_descr) (Xml.children msg),
               List.mem_assoc "ac_id" msg_descr.PprzLink.fields)
            with Failure e ->
              failwith (sprintf "Couldn't parse message %s (%s)" msg_name e)
          end
      | "Trim" -> ([], false)
      | _ -> failwith ("Unknown message class type") in

  let on_event =
    try Some (parse_value (Xml.attrib msg "on_event")) with _ -> None in

  { msg_name = msg_name;
    msg_class = msg_class;
    fields = fields;
    on_event = on_event;
    send_always = send_always;
    has_ac_id = has_ac_id
  }

(** Parse an XML list of variables and set function *)
let parse_variables = fun variables ->
  let l = ref [] in
  List.iter (fun x ->
    match Xml.tag x with
        "var" ->
          let name = Xml.attrib x "name"
          and default = ExtXml.int_attrib x "default" in
          if List.mem_assoc name !l then failwith (sprintf "Variable %s already declared" name);
        (* filter all "set" node for this variable *)
          let set = List.filter (fun vs ->
            (ExtXml.tag_is vs "set") &&
              (compare (ExtXml.attrib_or_default vs "var" "") name) = 0)
            (Xml.children variables) in
          let var_event = List.map (fun s ->
            let value = ExtXml.int_attrib s "value"
            and on_event = parse_value (Xml.attrib s "on_event") in
            (value, on_event)
          ) set in
          l := (name, { value = default; var_event = var_event; }) :: !l;
          ()
      | _ -> ()
  ) (Xml.children variables);
  !l


(** Verbose List.assoc *)
let my_assoc = fun x l ->
  try List.assoc x l with Not_found ->
    failwith (sprintf "my_assoc: %s not found" x)

let first_of_two (x,_) = x

let second_of_two (_,x) = x

(** set a trim value given an inputs array and a trim_values tuple *)
let trim_set = fun inputs value ->
  let input = my_assoc (first_of_two value) inputs in
  match input with
      Axis (i, deadband, limit, exponent, trim) -> trim := (second_of_two value)
    | Button i -> failwith "No trim for buttons"
    | Hat _ -> failwith "No trim for hats"


(** Input the trim file if it exists *)
let parse_trim_file = fun trim_file_name inputs ->
  if Sys.file_exists trim_file_name then begin
    let trim = ExtXml.parse_file trim_file_name in
    let trim_values = List.map
      (fun x ->
        let axis = ExtXml.attrib x "axis"
        and trimval = ExtXml.float_attrib x "value" in
        (axis, trimval))
      (Xml.children trim) in
    List.iter (trim_set inputs) trim_values;
  end

(** Parse the complete (input and messages) XML desxription
    Also parses the trim xml file if it exists *)
let parse_descr = fun xml_file trim_file ->
  let xml = ExtXml.parse_file xml_file in

  let inputs = parse_input (ExtXml.child xml "input")
  and messages_xml = ExtXml.child xml "messages"
  and variables = try parse_variables (ExtXml.child xml "variables") with _ -> [] in

  let period_ms = int_of_float (1000.*.ExtXml.float_attrib messages_xml "period")
  and messages = List.map parse_msg (Xml.children messages_xml) in

  (** check for trim file *)
  parse_trim_file trim_file inputs;

  { period_ms = period_ms; inputs = inputs; messages = messages; variables = variables }

(** apply deadband  - applied first *)
let apply_deadband = fun x min ->
  if abs x < min then 0 else x

(** apply limit - applied third *)
let apply_limit = fun x limit ->
  limit *. x

(** apply exponent  - applied second *)
let apply_exponent = fun x expon ->
  let pow_value = (float_of_int x) ** 3. /. (float_of_int max_input) ** 2. in
  ( (float_of_int x) *. (1. -. expon)) +. (pow_value *. expon)

(** apply trim  - applied fourth *)
let apply_trim = fun x trim ->
  let x_new = (int_of_float (x +. trim)) in
  if x_new > max_input then max_input else (if x_new < min_input then min_input else x_new)

(** Access to an input value, button or axis *)
let eval_input = fun buttons hat axis input ->
  match input with
      Axis (i, deadband, limit, exponent, trim) ->  (apply_trim (apply_limit (apply_exponent (apply_deadband axis.(i) deadband) exponent) limit) trim.contents)
    | Button i -> (buttons lsr i) land 0x1
    | Hat _ -> hat

(** Scale a value in the given bounds *)
let scale = fun x min max ->
  min + ((x - min_input) * (max - min)) / (max_input - min_input)

(** Scale a value in the given bounds *)
let bound = fun x min max ->
  if x < min then min else (if x > max then max else x)

(** Fit a given interval of value into [min_input; max_input] *)
let fit = fun x min max min_input max_input ->
  let v = min_input + ((x - min) * (max_input - min_input)) / (max - min) in
  bound v min_input max_input

(** Return a pprz RC mode
    * mode > max -> 2
    * mode < min -> 0
    * else 1
*)
let pprz_threshold = max_input / 2
let pprz_mode = fun mode ->
  if mode > pprz_threshold then 2
  else if mode < -pprz_threshold then 0
  else 1

(** Eval a function call (TO BE COMPLETED) *)
let eval_call = fun f args hat->
  match f, args with
      "-", [a] -> - a
    | "-", [a1; a2] -> a1 - a2
    | "+", [a1; a2] -> a1 + a2
    | "*", [a1; a2] -> a1 * a2
    | "%", [a1; a2] -> a1 / a2
    | "&&", [a1; a2] -> a1 land a2
    | "||", [a1; a2] -> a1 lor a2
    | "<",  [a1; a2] -> if a1 < a2 then 1 else 0
    | ">",  [a1; a2] -> if a1 > a2 then 1 else 0
    | "HatCentered",  [_] -> if hat = 0 then 1 else 0
    | "HatUp",        [_] -> if hat = 1 then 1 else 0
    | "HatRight",     [_] -> if hat = 2 then 1 else 0
    | "HatRightUp",   [_] -> if hat = 3 then 1 else 0
    | "HatDown",      [_] -> if hat = 4 then 1 else 0
    | "HatRightDown", [_] -> if hat = 6 then 1 else 0
    | "HatLeft",      [_] -> if hat = 8 then 1 else 0
    | "HatLeftUp",    [_] -> if hat = 9 then 1 else 0
    | "HatLeftDown",  [_] -> if hat = 12 then 1 else 0
    | "Scale", [x; min; max] -> scale (x) (min) (max)
    | "Fit", [x; min; max; min_input; max_input] -> fit (x) (min) (max) (min_input) (max_input)
    | "Bound", [x; min; max] -> bound (x) (min) (max)
    | "PprzMode", [x] -> pprz_mode (x)
    | "JoystickID", [] -> !joystick_id
    | f, args -> failwith (sprintf "eval_call: unknown function '%s'" f)

(** Eval an expression *)
let eval_expr = fun buttons hat axis inputs variables expr ->
  let rec eval = function
    | Syntax.Ident ident ->
      (* try input first, then variables and hat position *)
      let i = match (List.mem_assoc ident inputs, List.mem_assoc ident variables) with
      | (true, _) -> eval_input buttons hat axis (List.assoc ident inputs)
      | (false, true) ->
          let v = List.assoc ident variables in
          v.value
      | (false, false) -> failwith (sprintf "eval_expr: %s not found" ident)
      in
      i
    | Syntax.Int int -> int
    | Syntax.Float float -> failwith "eval_expr: float"
    | Syntax.Call (ident, exprs) | Syntax.CallOperator (ident, exprs) ->
      eval_call ident (List.map eval exprs) hat
    | Syntax.Index _ -> failwith "eval_expr: index"
    | Syntax.Field _ -> failwith "eval_expr: Field"
    | Syntax.Deref _ -> failwith "eval_expr: deref" in

  eval expr

(** Store for the last sent values: msg_name->values *)
let last_values = Hashtbl.create 5

(** Get the previous sent values for a given message *)
let get_previous_values = fun msg_name ->
  try Hashtbl.find last_values msg_name with Not_found -> (false, [])

(** Record the current values for a given message *)
let record_values = fun msg_name values ->
  Hashtbl.replace last_values msg_name values

let second_list (_,x) = x
let first_list (x,_) = x

(** add a leaf *)
let trim_save_add_leaf = fun x channel_pair ->
  let chan_name = first_list channel_pair in
  let channel = second_list channel_pair in
  match channel with
      Axis (i, deadband, limit, exponent, trim) -> x := x.contents ^ (Printf.sprintf "<trim axis='%s' value = '%f'/>" chan_name trim.contents)
    | Button i -> Printf.printf "%d" i
    | Hat _ -> Printf.printf "hat"

(** save trim settings to file *)
let trim_save = fun inputs ->
  let xmlstring = ref "<trims>" in
  List.iter (trim_save_add_leaf xmlstring) inputs;
  xmlstring := xmlstring.contents ^ "</trims>";
  let x = Xml.parse_string xmlstring.contents in
  let pretty_xml_string = Xml.to_string_fmt x in
  let output_trim_file = open_out trim_file_name.contents in
  output_string output_trim_file pretty_xml_string;
  close_out output_trim_file

(** Adjust the trim on a specified channel *)
let trim_adjust = fun axis_name adjustment inputs ->
  let input = my_assoc axis_name inputs in
  match input with
      Axis (i, deadband, limit, exponent, trim) -> trim := trim.contents +. adjustment
    | Button _ -> failwith "No trim for buttons"
    | Hat _ -> failwith "No trim for hats"

(** Update variables state *)
let update_variables = fun inputs buttons hat axis variables ->
  List.iter (fun (_,var) ->
    List.iter (fun (value, expr) ->
      let event = eval_expr buttons hat axis inputs variables expr in
      if event <> 0 then begin
        (* remove and add again ? *)
        var.value <- value
      end
    ) var.var_event
  ) variables

(** Send an ivy message if needed: new values and/or 'on_event' condition true*)
let execute_action = fun ac_id inputs buttons hat axis variables message ->
  let values =
    List.map
      (fun (name, _type, expr) ->
        let v = eval_expr buttons hat axis inputs variables expr in
        (name, PprzLink.value _type (sprintf "%d" v))
      )
      message.fields

  and on_event =
    match message.on_event with
        None -> true
      | Some expr -> eval_expr buttons hat axis inputs variables expr <> 0 in

  let previous_values = get_previous_values message.msg_name in
  (* FIXME ((value <> previous) && on_event) || send_always ??? *)
  if ( ( (on_event, values) <> previous_values ) || message.send_always ) && on_event then begin
    match message.msg_class with
        "datalink" ->
          let vs = if message.has_ac_id then ("ac_id", PprzLink.Int ac_id) :: values else values in
          DL.message_send "input2ivy" message.msg_name vs
      | "ground" ->
          let vs = if message.has_ac_id then ("ac_id", PprzLink.String (sprintf "%d" ac_id)) :: values else values in
          G.message_send "input2ivy" message.msg_name vs
      | "trim_plus" -> trim_adjust message.msg_name trim_step inputs
      | "trim_minus" -> trim_adjust message.msg_name (-.trim_step) inputs
      | "trim_save" -> trim_save inputs
      | c -> failwith (sprintf "execute_action: unknown class '%s'" c)
  end;
  record_values message.msg_name (on_event, values)


(** Output on stderr the values from the input device *)
let print_inputs = fun nb_buttons buttons hat axis ->
  print_string "buttons: ";
  for i = 0 to nb_buttons - 1 do
    printf "%d:%d " i (eval_input buttons hat axis (Button i))
  done;
  printf "\nhat: %d" (eval_input buttons hat axis (Hat 0));
  print_string "\naxis: ";
  for i = 0 to Array.length axis - 1 do
    printf "%d:%d " i (eval_input buttons hat axis (Axis (i, 0, 1.0, 0.0, ref 0.0)))
  done;
  ignore (print_newline ())


(** Get the values from the input values and send messages
    This is called at a rate programmed in the xml  *)
let execute_actions = fun actions ac_id ->
  try
    let (nb_buttons, buttons, hat, axis) = stick_read () in

    if !verbose then
      print_inputs nb_buttons buttons hat axis;

    (* TODO update variables before msg *)
    update_variables actions.inputs buttons hat axis actions.variables;
    List.iter (execute_action ac_id actions.inputs buttons hat axis actions.variables) actions.messages
  with
      exc -> prerr_endline (Printexc.to_string exc)


(**  process keyboard commands *)
(**   used for adjusting trims interactively from the keyboard *)
(**   this capability is mostly for bench-time trimming when a joystick does not have adequate buttons *)
(**   it is not a very complete capability  *)
let execute_kb_action = fun actions conditions ->
  let ch = input_byte stdin in
  (** esdx for left stick
      ijkm for right  *)

  if true then begin
    match ch with
        101 -> trim_adjust "ly" 1.0 actions.inputs
      | 115 -> trim_adjust "lx" (-1.0) actions.inputs
      | 100 -> trim_adjust "lx" 1.0 actions.inputs
      | 120 -> trim_adjust "ly" (-1.0) actions.inputs
      | 105 -> trim_adjust "ry" 1.0 actions.inputs
      | 106 -> trim_adjust "rx" (-1.0) actions.inputs
      | 107 -> trim_adjust "rx" 1.0 actions.inputs
      | 109 -> trim_adjust "ry" (-1.0) actions.inputs
      | _ -> trim_adjust "ly" 0.0 actions.inputs
  end;

  true



(************************************* MAIN **********************************)
let () =
  let ivy_bus = ref Defivybus.default_ivy_bus  in
  let device_index = ref 0
  and ac_name = ref "MYAC"
  and xml_descr = ref "" in

  let anon_fun = (fun x -> xml_descr := x) in
  let speclist =
    [ "-b", Arg.String (fun x -> ivy_bus := x),(sprintf "<ivy bus> Default is %s" !ivy_bus);
      "-ac",  Arg.Set_string ac_name, "<A/C name>";
      "-d",  Arg.Set_int device_index, "<device index>";
      "-v",  Arg.Set verbose, "Verbose mode (useful to identify the channels of an input device)";
      "-id", Arg.Set_int joystick_id, "Joystick ID, from 0-255.  Each joystick requires a unique ID in a multiple joystick configuration.";
      "-c", Arg.Set axis_check, "Check all axis moved for correct initilization before starting normal behavior";
      "-", Arg.String anon_fun, "<xml file of actions>"
    ]
  and usage_msg = "Usage: " in

  Arg.parse speclist anon_fun usage_msg;

  if !xml_descr = "" then begin
    Arg.usage speclist usage_msg;
    exit 1
  end;

  let ac_id = ac_id_of_name !ac_name in

  if ac_id > 0 then begin
    (* build hash only for real AC, not for GCS *)
    hash_index_of_settings !ac_name;
    hash_index_of_blocks !ac_name;
  end;

  Printf.printf "Joystick ID (option -id): %u\n" !joystick_id;
  Printf.printf "Joystick SDL device index (option -d): %u\n" !device_index;

  let joystick_conf_dir = conf_dir ^ "/joystick/" in
  let xml_descr_full = joystick_conf_dir ^ !xml_descr in
  trim_file_name := String.concat "." [xml_descr_full ; !ac_name ; "trim"];

  let actions = parse_descr xml_descr_full trim_file_name.contents in

  if stick_init !device_index <> 0 then
    failwith (sprintf "Error: cannot open device with SDL index %i\n" !device_index);

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi joystick" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (** setup stdin *) (* TODO find a better way to change trim, use a GUI ? *)
  (*let tstatus = (Unix.tcgetattr Unix.stdin) in
    tstatus.c_icanon <- false;
    Unix.tcsetattr Unix.stdin Unix.TCSANOW tstatus;*)
  
  if !axis_check then
    while stick_check_axis() <> 1 do
      (* It seems there is no proper way to wait less than a second before OCAML 4.03.0, introducing Unix.sleepf *)
      ignore (Unix.select [] [] [] 0.1)
    done;

  ignore (Glib.Timeout.add ~ms:actions.period_ms ~callback:(fun () -> execute_actions actions ac_id; true));
  (*ignore (Glib.Io.add_watch ~cond:[`IN] ~callback:(fun x -> execute_kb_action actions x) (Glib.Io.channel_of_descr Unix.stdin));*)

  (** Start the main loop *)
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done

