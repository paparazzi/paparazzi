
exception Failure of string

let file_of_url = fun ?dest url ->
  if String.sub url 0 7 = "file://" then
    String.sub url 7 (String.length url - 7)
  else
    let tmp_file =
      match dest with
	Some s -> s
      | None -> Filename.temp_file "fp" ".wget" in
    let c = Printf.sprintf "wget -nv --cache=off -O %s '%s'" tmp_file url in
    if Sys.command c = 0 then
      tmp_file
    else begin
      Sys.remove tmp_file;
      raise (Failure url)
    end
