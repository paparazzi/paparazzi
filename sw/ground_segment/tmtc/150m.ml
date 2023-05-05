(** Code example on the Ivy bus.
    Agent which monitors the altitude from the ground and set the A/C in HOME mode
    if it reaches 150m. Also displays this altitude and a HOME button to allow the user
    to force the HOME mode *)


let (//) = Filename.concat

(* Hard coded id of the A/C *)
let ac_id = "1"

(* Hard coded index of the pprz_mode variable from the XML setting file *)
let index_pprz_mode = 5 (* From var/include/basic.h *)

let autopilot_HOME_mode_value = 3 (* From sw/airborne/autopilot.h *)

(* Build the module to listen telemetry *)
module Tele_Pprz = PprzLink.Messages(struct let name = "telemetry" end)

(* Build the module to send uplink messages *)
module Datalink_Pprz = PprzLink.Messages(struct let name = "datalink" end)



(******************************* Send the message to the A/C to set it in HOME mode *)
let set_to_HOME = fun () ->
  let vs = ["ac_id", PprzLink.String ac_id;
            "index", PprzLink.Int index_pprz_mode;
            "value", PprzLink.Float (float autopilot_HOME_mode_value)] in
  Datalink_Pprz.message_send "dl" "SETTING" vs


(******************************* Get GPS message, display the altitude from the SRTM
                                 model, and set to HOME if higher than 150m *)
let get_gps_message = fun label _sender vs ->
  (* Extract data from the message *)
  let alt_m = PprzLink.int_assoc "alt" vs / 100
  and utm_east = PprzLink.int_assoc "utm_east" vs / 100
  and utm_north = PprzLink.int_assoc "utm_north" vs / 100
  and utm_zone = PprzLink.int_assoc "utm_zone" vs in

  (* Build the geographic position *)
  let utm = { Latlong.utm_x    = float utm_east;
              Latlong.utm_y    = float utm_north;
              Latlong.utm_zone = utm_zone } in

  (* Get the ground altitude from the SRTM model *)
  let srtm_alt_m = Srtm.of_utm utm in

  let height = alt_m - srtm_alt_m in

  (* Display in the label *)
  label#set_text (Printf.sprintf "%d" height);

  (* Check if too high *)
  if height > 150 then
    set_to_HOME ()


(********************************* Main *********************************************)
let () =
  let ivy_bus = Defivybus.default_ivy_bus in

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi 150m" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (** Open the window and a horizontal box container *)
  let icon = GdkPixbuf.from_file Env.icon_file in
  let window = GWindow.window ~icon ~allow_shrink:true ~title:"GPWS" () in
  let hbox = GPack.hbox ~packing:window#add () in

  (* Add a label *)
  let label = GMisc.label ~text:"N/A" ~packing:hbox#add () in

  (* And a button *)
  let button = GButton.button ~label:"HOME" ~packing:hbox#add () in

  (* Listen GPS message *)
  ignore (Tele_Pprz.message_bind "GPS" (get_gps_message label));

  (* Connect the button to its action *)
  ignore (button#connect#clicked ~callback:set_to_HOME);

  (** Display the window and start the main loop *)
  window#show ();
  GMain.main ()
