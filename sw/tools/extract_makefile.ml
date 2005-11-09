open Printf 

let _ =
  let f = Sys.argv.(1) in
  let xml = Xml.parse_file f in
  printf "# This file has been generated from %s\n" f;
  printf "# Please DO NOT EDIT\n";
  try
    match Xml.children (ExtXml.child xml "makefile") with
      [Xml.PCData s] -> 
	printf "%s\n" s
    | _ -> ()
  with
    _ -> ()
