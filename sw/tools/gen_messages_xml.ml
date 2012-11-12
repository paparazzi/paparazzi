(*

 * XML generation of messages.xml for downlink protocol
 *
 * Copyright (C) 2012 Xavier Gibert
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

(* ------------ PATHS & FILES ------------ *)
(** File to get the includes *)
let includes_file = Sys.argv.(1)
(** Path to find spread messages files *)
let spread_messages_path = Sys.argv.(2)
(** Generated messages file *)
let generated_file = Sys.argv.(3)
(** Generated macros path *)
let var_include_path = Sys.argv.(4)
(** Generated macros uplink file *)
let uplink_msg_h = Sys.argv.(5)
(** Generated macros downlink file *)
let downlink_msg_h = Sys.argv.(6)
(* --------------------------------------- *)

(** Type include_xml: the structure of a XML include element *)
type include_xml = {
  file : string;
  class_id : int;
  class_name : string;
}

type generated_class = {
  g_id : string;
  g_name : string;
  g_type : string;
}


(** Module to obtain the desired messages files to include in messages.xml *)
module Includes = struct

  exception Invalid_include_structure of string
  exception Duplicated_class_id of string
  exception Duplicated_class_name of string
  exception Repeated_element of string

  (** Translates an "include" XML element into a value of the 'include' type  *)
  let struct_of_xml_include = fun conf_xml ->
    try
      let file = ExtXml.attrib conf_xml "file"
      and class_id = ExtXml.int_attrib conf_xml "class_id"
      and class_name = ExtXml.attrib conf_xml "class_name" in
    { file = file; class_id = class_id; class_name = class_name }
    with e -> raise (Invalid_include_structure (Printexc.to_string e))

  let check_repeated_elements = fun list ->
    let rec loop = fun list ->
      match list with
        | [] | [_] -> list
        | x::((x'::_) as xs) -> if x = x' then raise (Repeated_element x) else x::loop xs in
    loop (List.sort compare list)

  let check_single_names = fun xml_includes ->
    let names = List.map (fun inc -> inc.class_name) xml_includes in
    try
      check_repeated_elements names
    with
      | Repeated_element el -> raise (Duplicated_class_name el)
      | e -> raise e

  let check_single_ids = fun xml_includes ->
    let ids = List.map (fun inc -> string_of_int inc.class_id) xml_includes in
    try
      check_repeated_elements ids;
    with
      | Repeated_element el -> raise (Duplicated_class_id el)
      | e -> raise e

  (** Returns a list of all the includes in the messages_conf.xml file *)
  let get_includes = fun () ->
    let conf_xml = Xml.parse_file includes_file in
    try
      let xml_includes = List.map struct_of_xml_include (Xml.children conf_xml) in
      ignore (check_single_ids xml_includes);
      ignore (check_single_names xml_includes);
      xml_includes
    with
      | Duplicated_class_id ii -> raise (Duplicated_class_id ii)
      | Duplicated_class_name n -> raise (Duplicated_class_name n)
      | Invalid_include_structure exc -> raise (Invalid_include_structure exc)
      | e -> raise e

end

(** Module to obtain the classes and their messages from the included files *)
module Classes = struct

  exception Invalid_class_structure of string
  exception Invalid_class_type of string
  exception XML_parsing_error of string * string * string
  exception Invalid_class_xml_node of string
  exception Class_id_out_of_range of int

  (** Checks if the class type is one of the allowed ones *)
  let check_class_type = fun attribute ->
    match attribute with
      | "datalink" | "uplink" | "downlink" | "airborne" | "ground" -> attribute
      | t -> raise (Invalid_class_type t)

  (** Translates a "class" XML element into a value of the 'class' type  *)
  let struct_of_xml_class = fun xml ->
    try
      let _type = ExtXml.attrib xml "type" in
      check_class_type(_type)
    with
      | Invalid_class_type t -> raise (Invalid_class_type t)
      | e -> raise (Invalid_class_structure (Printexc.to_string e))


  type parameters_list = (string * string) list

  (** Builds a XML Node with the class and its messages from a given file *)
  let get_class_from_include = fun _include ->
    let file = spread_messages_path // _include.file
    and class_id = _include.class_id
    and class_name = _include.class_name in
    try
      (** Get XML code from file *)
      let spread_xml = Xml.parse_file file in
      (** Get class type *)
      let class_type = struct_of_xml_class spread_xml in
      if((class_type<>"ground")&&(class_id > 31 || class_id < 0)) then raise (Class_id_out_of_range class_id) else
      (** Get messages piece of xml *)
      let xml_piece_messages = Xml.children spread_xml in
      (** Build the parameters list for the XML node *)
      let param_list = ["id",string_of_int class_id;"name",class_name;"type",class_type] in
      (** Buid XML node with class tag & params + child (all messages) *)
      Xml.Element("class",param_list,xml_piece_messages)
    with
      | Invalid_class_type t -> raise (Invalid_class_type t)
      | Invalid_class_structure exc -> raise (Invalid_class_structure exc)
      | Class_id_out_of_range i -> raise (Class_id_out_of_range i)
      | Xml.Error (msg, pos) -> raise (XML_parsing_error (file,(Xml.error_msg msg),(string_of_int (Xml.line pos))))
      | e -> raise e

  (** Extracts the class parameters from a XML class node *)
  let extract_name_type = fun xml_class ->
    try
      { g_id = ExtXml.attrib xml_class "id"; g_name = ExtXml.attrib xml_class "name"; g_type = ExtXml.attrib xml_class "type" }
    with
      | e -> raise (Invalid_class_xml_node (Printexc.to_string e))
end

(** Module to save the generated messages file *)
module SaveXml = struct

  exception Cannot_open_file of string * string
  exception Cannot_move_file of string

  (** Saved the generated file with some header *)
  let save = fun xml_code ->
    try
      let (temp_filename,h) = Filename.open_temp_file "TEMP" "TEMP" in
      (*let h = open_out (generated_file) in*)
      (*let h = stdout in*)
      Printf.fprintf h "<!-- ************************************************************************************************* -->\n";
      Printf.fprintf h "<!-- *                 AUTO-GENERATED MESSAGES FILE FROM messages_conf.xml INCLUDES                  * -->\n";
      Printf.fprintf h "<!-- *                                    Please DO NOT EDIT                                         * -->\n";
      Printf.fprintf h "<!-- * TIP: You can see an example of messages_conf.xml format in messages_conf.xml.example file     * -->\n";
      Printf.fprintf h "<!-- ************************************************************************************************* -->\n";
      Printf.fprintf h "%s" xml_code;
      close_out h;
      let c = sprintf "mv %s %s " temp_filename generated_file in
      let returned_code = Sys.command c in
      if returned_code <> 0 then raise ( Cannot_move_file (string_of_int returned_code))
    with
      | e -> raise (Cannot_open_file ("messages.xml",(Printexc.to_string e)))
end

(** MAIN MODULE: DOES ALL THE PROCEDURE TO GENERATE THE messages.xml FROM THE messages_conf.xml INCLUDES *)
module SpreadMessages = struct

  (** Generates a XML file called messages.xml with all the included classes in messages_conf.xml and their messages *)
  let generate_messages_xml = fun () ->
    try
      (** Get list of inluded files *)
      let includes = Includes.get_includes () in
      (** Get list of included classes as XML Elements *)
      let classes = List.map Classes.get_class_from_include includes in
      (** Create the complete XML element *)
      let final_xml = Xml.Element("protocol",["version","2.0"; "message_version","2.0"],classes) in
      (** User-readable xml formating *)
      let formated_xml = Xml.to_string_fmt final_xml in
      (** Save the xml code into a xml file *)
      SaveXml.save formated_xml;
      (** Prepare and return data for the next function (List of class_ids, class_names and class_types) *)
      let class_ids_names_types = List.map Classes.extract_name_type classes in
      class_ids_names_types
    with
      | Includes.Invalid_include_structure exc -> failwith (sprintf "Invalid <include> structure (Exception: %s)" exc)
      | Includes.Duplicated_class_id ii -> failwith (sprintf "Duplicated class id (%s) at includes file" ii)
      | Classes.Class_id_out_of_range i -> failwith (sprintf "Class id (%d) out of range [0->31] at includes file. Only ground type classes can exceed 31" i)
      | Includes.Duplicated_class_name n -> failwith (sprintf "Duplicated class name (%s) at includes file" n)
      | Classes.Invalid_class_structure exc -> failwith (sprintf "Invalid <class> structure (Exception: %s)" exc)
      | Classes.Invalid_class_type typ -> failwith (sprintf "Invalid class type: %s" typ)
      | Classes.XML_parsing_error (file,msg,pos) -> failwith (sprintf "Error parsing XML file: %s (Error: %s Line: %s)" (file) (msg) (pos))
      | SaveXml.Cannot_open_file (file,exc) -> failwith (sprintf "Cannot open the file to generate: %s (Exception: %s)" file exc)
      | SaveXml.Cannot_move_file err -> failwith (sprintf "Cannot move the generated file to the final destination (Error code for command MV: %s)" err)
      | Classes.Invalid_class_xml_node exc -> failwith (sprintf "Invalid <class> xml node in generated file (Exception: %s)" exc)
      | e -> failwith (sprintf "Unhandled exception raised: %s" (Printexc.to_string e))
end

module MakeCalls = struct

  let make_options = ""

  let make = fun class_name class_id check_alignment ->
    let file = Env.paparazzi_home // "Makefile" in
    let macros_target = var_include_path // ("messages_"^(String.lowercase class_name)^".h") in
    let c = sprintf "make -f %s MACROS_TARGET=%s MACROS_CLASS=%s MACROS_CLASS_ID=%s MACROS_ALIGN=%u %s %s" file macros_target class_name class_id check_alignment make_options macros_target in
    let returned_code = Sys.command c in
    if returned_code <> 0 then failwith (sprintf "Make command error (Error code: %d)" returned_code)


  let generate_macros = fun classes ->
    List.iter (fun clas -> match (clas.g_type) with
      | "datalink" -> prerr_endline ("\t Datalink Class -> Generate macros ("^clas.g_name^") [Check Alignment]"); make clas.g_name clas.g_id 1
      | "uplink" -> prerr_endline ("\t Uplink Class   -> Generate macros ("^clas.g_name^") [Check Alignment]"); make clas.g_name clas.g_id 1
      | "downlink" -> prerr_endline ("\t Downlink Class -> Generate macros ("^clas.g_name^")"); make clas.g_name clas.g_id 0
      | "ground" -> prerr_endline ("\t Ground Class   -> Do nothing ("^clas.g_name^")")
      | "airborne" -> prerr_endline ("\t Airborne Class -> Generate macros ("^clas.g_name^") FIXME!")
      | t -> failwith (sprintf "Invalid class type in generated file: %s" t)
      ) classes;

end

module FinalMacros = struct
  exception Cannot_open_file of string
  exception Cannot_move_file of string * string

  let create_both_files = fun () ->
    try
      let (temp_filename_up,h_up) = Filename.open_temp_file "TEMP" "TEMP" in
      let (temp_filename_down,h_down) = Filename.open_temp_file "TEMP" "TEMP" in
      (temp_filename_up, h_up, temp_filename_down, h_down)
    with
      | e -> raise (Cannot_open_file (Printexc.to_string e))

  let close_both_files = fun temp_filename_up h_up temp_filename_down h_down ->
    close_out h_up;
    close_out h_down;
    let cu = sprintf "mv %s %s " temp_filename_up uplink_msg_h in
    let returned_code_u = Sys.command cu in
    prerr_endline ("\t Uplink    -> "^uplink_msg_h);
    let cd = sprintf "mv %s %s " temp_filename_down downlink_msg_h in
    let returned_code_d = Sys.command cd in
    prerr_endline ("\t Downlink  -> "^downlink_msg_h);
    if returned_code_u <> 0 then raise ( Cannot_move_file ("uplink",string_of_int returned_code_u));
    if returned_code_d <> 0 then raise ( Cannot_move_file ("downlink",string_of_int returned_code_d))


  let include_in_file = fun clas h ->
    Printf.fprintf h "#include \"messages_%s.h\"\n" clas.g_name

  let generate_files = fun classes ->
    try
      let (temp_filename_up, h_up, temp_filename_down, h_down) = create_both_files () in
      ignore(
      List.map (fun clas -> match (clas.g_type) with
        | "uplink" -> include_in_file clas h_up
        | "downlink" -> include_in_file clas h_down
        | "datalink" -> include_in_file clas h_up; include_in_file clas h_down
        | _ -> ()
        ) classes);
      close_both_files temp_filename_up h_up temp_filename_down h_down
    with
      | Cannot_open_file s -> failwith (sprintf "Cannot open file to inlcude macro files (Exception: %s)" s)
      | Cannot_move_file (f,s) -> failwith (sprintf "Cannot move file of %s macros to it's final destination (Error code for command MV: %s)" f s )

end

(********************* Main **************************************************)
let () =
  if Array.length Sys.argv <> 7 then
    failwith (sprintf "Usage: %s <messages config file> <spread messages path> <generated file> <var_include_path>" Sys.argv.(0));
  let classes_info = SpreadMessages.generate_messages_xml () in

  prerr_endline ("----------- GENERATING MESSAGES MACROS (Spread) -----------");
  ignore (MakeCalls.generate_macros classes_info);
  prerr_endline ("-----------------------------------------------------------");

  prerr_endline ("----------- GENERATING MESSAGES MACROS (Global) -----------");
  ignore (FinalMacros.generate_files classes_info);
  prerr_endline ("-----------------------------------------------------------");
