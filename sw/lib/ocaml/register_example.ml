module Tm_Pprz = Pprz.Messages_of_type (struct let class_type = "downlink" end)

let to_binary = fun ac_id m ->
  let msg_id, vs = Tm_Pprz.values_of_string m in
	let cls_id = Pprz.class_id_of_msg_args m in
  let payload = Tm_Pprz.payload_of_values ac_id ~class_id:cls_id msg_id vs in
  let buf = Pprz.Transport.packet payload in
  Printf.printf "From caml: %s\n%!" (Debug.xprint buf);
  buf

let _ = Callback.register "to_binary" to_binary
