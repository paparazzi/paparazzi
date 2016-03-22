open Printf
module U = Unix

let (//) = Filename.concat

let print_params = fun outfile xml ->
  List.iter (fun tag ->
    if ExtXml.tag_is tag "param" then begin
      begin try
        fprintf outfile " param %s %s \n" (Xml.attrib tag "name") (Xml.attrib tag "values");
      with _ -> () end;
    end)
    (Xml.children xml)

let print_subsystems = fun outfile xml ->
  List.iter (fun tag ->
    if ExtXml.tag_is tag "subsystem" then begin
      fprintf outfile "\n";
      fprintf outfile   " subsystem '%s' \n" (Xml.attrib tag "name");
      fprintf outfile   "   types: '%s'\n"   (Xml.attrib tag "types");
(**      fprintf outfile   "####################################################\n"; **)
      print_params outfile tag;  (** subsystem parameters **)
    end)
    (Xml.children xml)


let print_targets = fun outfile xml ->
  List.iter (fun tag ->
    if ExtXml.tag_is tag "target" then begin
      fprintf outfile "\n\n";
      fprintf outfile   "===target '%s'===\n" (Xml.attrib tag "name");
      fprintf outfile   "   Description: '%s'\n"       (Xml.attrib tag "description");
      print_params outfile tag;      (** target parameters **)
      print_subsystems outfile tag;  (** target subsystems **)
    end)
    (Xml.children xml)




let parse_firmware_xml = fun firmware_xml firmware_wiki ->
  let xml = ExtXml.parse_file firmware_xml in
  let f = open_out firmware_wiki in
  fprintf f " This file has been generated from %s\n" firmware_xml;
  if ExtXml.tag_is xml "firmware" then begin
    begin try
      fprintf f "\n=Firmware=\n";
      fprintf f " This is the '%s' firmware\n" (Xml.attrib xml "name");
      fprintf f "   Supported boards '%s'\n"   (Xml.attrib xml "boards");
      fprintf f "   Description: '%s'\n"       (Xml.attrib xml "description");
      fprintf f "==Parameters==\n";
      print_params f xml;      (** fimware parameters **)
      fprintf f "==Subsystems==\n";
      print_subsystems f xml;  (** firmware subsystems **)
      fprintf f "==Targets==\n";
      print_targets f xml;     (** targets **)
    with _ -> () end;
  end


(******************************* MAIN ****************************************)
(** call with wikigen firmware_desc.xml firmware_desc.wiki **)
let () =
  try
    if Array.length Sys.argv <> 3 then
      failwith (sprintf "Usage: %s firmware_desc.xml firmware_desc.wiki" Sys.argv.(0));
    let firmware_xml = Sys.argv.(1) in
    let firmware_wiki = Sys.argv.(2) in
    parse_firmware_xml firmware_xml firmware_wiki
  with
    Failure f ->
      prerr_endline f;
      exit 1
