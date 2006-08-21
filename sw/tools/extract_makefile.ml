open Printf 

let _ =
  let f = Sys.argv.(1) in
  let xml = Xml.parse_file f in
  printf "# This file has been generated from %s by sw/tools/extract_makefile\n" f;
  printf "# Please DO NOT EDIT\n";
  List.iter (fun x ->
    if ExtXml.tag_is x "makefile" then
      begin
	try
	  printf "\n# makefile target '%s'\n" (Xml.attrib x "target")
	with _ -> ()
      end;
    match Xml.children x with
      [Xml.PCData s] -> printf "%s\n" s
    | _ -> fprintf stderr "Warning: wrong makefile section in '%s': %s\n" f (Xml.to_string_fmt x))
    (Xml.children xml)
