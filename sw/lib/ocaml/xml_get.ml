open Xml2h
let _ =
  if Array.length Sys.argv <> 4 then
    failwith "Usage: conf_get <xml_file> <path (with dots)> <attribute>";
  let xml_file = Sys.argv.(1)
  and path = Sys.argv.(2)
  and attribute = Sys.argv.(3) in
  let xml =
    try
      Xml.parse_file xml_file
    with
        Xml.Error e ->
          Printf.fprintf stderr "\nError in \"%s\": %s\n\n" xml_file (Xml.error e);
          exit 1
  in
  Printf.printf "%s\n" (ExtXml.get_attrib xml path attribute)
