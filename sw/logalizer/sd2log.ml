open Printf
module U = Unix
let (//) = Filename.concat
let logs_path = Env.paparazzi_home // "var" // "logs"


module Tm_Pprz = Pprz.Messages (struct let name = "telemetry" end)

module Parser = Serial.Transport(Logpprz.Transport)

let run_command = fun com ->
  if Sys.command com <> 0 then begin
    fprintf stderr "Command '%s' failed\n" com;
    exit 1;
  end


  
let convert_file = fun file ->
  let tmp_file = Filename.temp_file "tlm_from_sd" "data" in

  let f_in = open_in file
  and f_out = open_out tmp_file in

  let start_unix_time = ref None in

  let use_payload = fun payload ->
    let log_msg = Logpprz.parse payload in
    let (msg_id, ac_id, vs) = 
      Tm_Pprz.values_of_payload log_msg.Logpprz.pprz_data in
    let msg_descr = Tm_Pprz.message_of_id msg_id in
    let timestamp = Int32.to_float log_msg.Logpprz.timestamp /. 1e4 in
    fprintf f_out "%.3f %d %s\n" timestamp ac_id (Tm_Pprz.string_of_message msg_descr vs);

    (** Looking for a date from a GPS message *)
    if !start_unix_time = None
	&& msg_descr.Pprz.name = "GPS"
	&& Pprz.int_assoc "mode" vs = 3 then
      let itow = Pprz.int_assoc "itow" vs / 1000
      and week = Pprz.int_assoc "week" vs in
      printf "itow=%d week=%d\n%!" itow week;
      let unix_time = Latlong.unix_time_of_tow ~week itow in
      start_unix_time := Some (unix_time -. timestamp)
  in

  let parser = Parser.parse use_payload in
  let Serial.Closure reader = Serial.input parser in

  try
    while true do
      reader (U.descr_of_in_channel f_in)
    done
  with
    End_of_file ->
      close_in f_in;
      close_out f_out;

      (* Rename the file according to the GPS time *)
      let start_time =
	match !start_unix_time with
	  None -> U.gettimeofday () (* Not found, use now *)
	| Some u -> u in

      let d = U.localtime start_time in
      let basename = sprintf "%02d_%02d_%02d__%02d_%02d_%02d_SD" (d.U.tm_year mod 100) (d.U.tm_mon+1) (d.U.tm_mday) (d.U.tm_hour) (d.U.tm_min) (d.U.tm_sec) in
      let data_name = sprintf "%s.data" basename in

      (** Move the produced .data file *)
      let com = sprintf "mv %s %s" tmp_file (logs_path // data_name) in
      run_command com;

      (** Save the original binary file *)
      let com = sprintf "mv %s %s" tmp_file (logs_path // data_name) in
      run_command com

let () =
  if Array.length Sys.argv = 2 then
    convert_file Sys.argv.(1)
  else begin
    fprintf stderr "Usage: %s <telemetry airborne file>\n" Sys.argv.(0);
    exit 1;
  end
