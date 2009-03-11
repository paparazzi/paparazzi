open Printf 

let pipe_regexp = Str.regexp "|"
let targets_of_field = fun field ->
  try 
    Str.split pipe_regexp (Xml.attrib field "target")
  with
    _ -> []

let get_modules = fun dir m ->
  match Xml.tag m with
    "load" -> begin
      let name = ExtXml.attrib m "name" in
      let xml = Xml.parse_file (dir^name) in
      xml
		end
  | _ -> failwith ("Warning: tag load is undefined")

let _ =
  let f = Sys.argv.(1) in
  let modules_dir = Sys.argv.(2) in
  let xml = Xml.parse_file f in
  printf "# This file has been generated from %s by sw/tools/extract_makefile\n" f;
  printf "# Please DO NOT EDIT\n";
  List.iter (fun x ->
    if ExtXml.tag_is x "makefile" then begin
      begin try
        printf "\n# makefile target '%s'\n" (Xml.attrib x "target")
      with _ -> () end;
      match Xml.children x with
        [Xml.PCData s] -> printf "%s\n" s
        | _ -> fprintf stderr "Warning: wrong makefile section in '%s': %s\n" f (Xml.to_string_fmt x)
    end)
  (Xml.children xml);
  let modules_exist = ref [ "" ] in
  List.iter (fun x ->
    if ExtXml.tag_is x "modules" then begin
      let modules_list = List.map (get_modules modules_dir) (Xml.children x) in
      List.iter (fun m ->
        let name = ExtXml.attrib m "name" in
        let dir_name = (String.uppercase name)^"_DIR" in
        printf "\n# makefile for module %s\n" name;
        printf "%s = $(PAPARAZZI_SRC)/sw/airborne/modules/%s\n" dir_name name;
        List.iter (fun l ->
          if ExtXml.tag_is l "makefile" then begin
            let targets = targets_of_field l in
            List.iter (fun t ->
              if not (List.exists (fun s -> (String.compare s t) == 0) !modules_exist) then begin
                printf "%s.srcs += $(ACINCLUDE)/modules.c\n" t;
                modules_exist := !modules_exist @ [t];
              end;
              printf "%s.CFLAGS += -I $(%s)\n" t dir_name
            ) targets;
            List.iter (fun f ->
              match Xml.tag f with
                "flag" -> List.iter (fun t -> printf "%s.CFLAGS += -D%s%s\n" t
                  (Xml.attrib f "name")
                  (try "="^(Xml.attrib f "value") with _ -> "")
                  ) targets
              | "file" -> List.iter (fun t -> printf "%s.srcs += $(%s)/%s\n" t
                  dir_name
                  (Xml.attrib f "name")
                  ) targets
              | _ -> ()
            )
            (Xml.children l)
            end)
        (Xml.children m))
      modules_list
    end)
  (Xml.children xml)
