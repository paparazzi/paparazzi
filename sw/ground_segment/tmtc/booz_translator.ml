open Printf

module Tm_Pprz = Pprz.Messages (struct let name = "telemetry" end)

let ac_id = "149"

let gps_status = ref 0

let get_status = fun _ values ->
  let ivalue = fun x -> try Pprz.int_assoc x values with Not_found ->
    failwith (sprintf "Error: field '%s' not found\n" x) in

  let ap_mode = ivalue "ap_mode" in
  if ap_mode = 0 then (); (* send kill *)
  let pprz_mode = ref 0 in
  gps_status := (ivalue "gps_status");
  (* let rc_mode = ivalue "rc_status" in *)
  let mode_values = [
    "ap_mode",       Pprz.Int !pprz_mode;
    "ap_gaz",        Pprz.Int 0;
    "ap_lateral",    Pprz.Int 0;
    "ap_horizontal", Pprz.Int 0;
    "if_calib_mode", Pprz.Int 0;
    "mcu1_status",   Pprz.Int 0] in
  Tm_Pprz.message_send ac_id "PPRZ_MODE" mode_values

let get_fp = fun _ values ->
  let i32value = fun x -> try Pprz.int32_assoc x values with Not_found ->
    failwith (sprintf "Error: field '%s' not found\n" x) in

  let lat    = Int32.to_float (i32value "lat") /. 1e7
  and lon    = Int32.to_float (i32value "lon") /. 1e7
  and alt    = Int32.to_float (i32value "alt") /. 100.
  and vnorth = Int32.to_float (i32value "vnorth")
  and veast  = Int32.to_float (i32value "veast")
  and phi    = Int32.to_int (i32value "phi")
  and theta  = Int32.to_int (i32value "theta")
  and psi    = Int32.to_int (i32value "psi") in
  let utm = Latlong.utm_of Latlong.WGS84 (Latlong.make_geo lat lon) in
  let gspeed = sqrt(vnorth*.vnorth +. veast*.veast) in
  (*let power_12 = Num.int_of_num (Num.power_num (Num.num_of_int 2)
   * (Num.num_of_int 12)) in*)
  let power_12 = 1 lsl 12 in

  let gps_values = [
    "mode",       Pprz.Int !gps_status;
    "utm_east",   Pprz.Int32 (Int32.of_float (utm.Latlong.utm_x *. 100.));
    "utm_north",  Pprz.Int32 (Int32.of_float (utm.Latlong.utm_y *. 100.));
    "course",     Pprz.Int (573 * psi / power_12);
    "alt",        Pprz.Int 0;
    "speed",      Pprz.Int (int_of_float gspeed);
    "climb",      Pprz.Int 0;
    "itow",       Pprz.Int32 (Int32.of_int 0);
    "utm_zone",   Pprz.Int 0;
    "gps_nb_err", Pprz.Int 0] in
  Tm_Pprz.message_send ac_id "GPS" gps_values;

  let est_values = [
    "z", Pprz.Float alt;
    "z_dot", Pprz.Float 0.] in
  Tm_Pprz.message_send ac_id "ESTIMATOR" est_values;

  let att_values = [
    "phi",   Pprz.Float (57.3 *. float_of_int (phi / power_12));
    "psi",   Pprz.Float (57.3 *. float_of_int (psi / power_12));
    "theta", Pprz.Float (57.3 *. float_of_int (theta / power_12))] in
  Tm_Pprz.message_send ac_id "ATTITUDE" att_values


(*********************** Main ************************************************)
let _ =
  let ivy_bus = ref "127.255.255.255:2010" in

  Arg.parse
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.255:2010"]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi BOOZ translator" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;


  ignore (Tm_Pprz.message_bind "BOOZ_STATUS" get_status);
  ignore (Tm_Pprz.message_bind "BOOZ2_FP" get_fp);

  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
