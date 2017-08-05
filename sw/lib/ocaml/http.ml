exception Failure of string
exception Not_Found of string
exception Blocked of string

IFDEF NETCLIENT_V_4 THEN
module H = Nethttp_client
let () =
  Nettls_gnutls.init()
ELSE
module H = Http_client
END

let file_of_url = fun ?dest url ->
  if Compat.bytes_sub url 0 7 = "file://" then
    Compat.bytes_sub url 7 (Compat.bytes_length url - 7)
  else
    let tmp_file =
      match dest with
          Some s -> s
        | None -> Filename.temp_file "fp" ".wget" in
    let call = new H.get url in
    call#set_response_body_storage (`File (fun () -> tmp_file));
    let pipeline = new H.pipeline in
    IFDEF NETCLIENT_V_404 THEN
    pipeline # set_proxy_from_environment (~insecure:false) ()
    ELSE
    pipeline # set_proxy_from_environment ()
    END;
    pipeline # add call;
    pipeline # run ();
    match call#status with
      | `Successful ->
        (* prerr_endline (Printf.sprintf "file sucessfull: %s, '%s'" tmp_file url); *)
        tmp_file
      | `Client_error ->
        begin
          (* prerr_endline (Printf.sprintf "getting file '%s', client error: %d" url call#response_status_code); *)
          Sys.remove tmp_file;
          match call#response_status_code with
              404 -> raise (Not_Found url)
            | 403 ->
              begin
                (* prerr_endline (Printf.sprintf "Blocked!!!"); *)
                raise (Blocked url)
              end
            | _ -> raise (Failure url)
        end
      | _ ->
        Sys.remove tmp_file;
        raise (Failure url)
