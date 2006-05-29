

module Protocol = struct
  let start_delimiter = Char.chr 0x7e
  let offset_length = 1
  let offset_payload = 3
  let size_packet = 4
  let index_start = fun s ->
    String.index s start_delimiter

  let length = fun s i ->
    if String.length s < i+offset_length+2 then
      raise Serial.Not_enough
    else
      (Char.code s.[i+offset_length] lsl 8) lor (Char.code s.[i+offset_length+1])

  let compute_checksum = fun s ->
    let cs = ref 0 in
    for i = offset_payload to String.length s - 2 do
      cs := (!cs + Char.code s.[i]) land 0xff
    done;
    0xff - !cs

  let checksum = fun s ->
    compute_checksum s + Char.code s.[String.length s-1] = 0xff

  let payload = fun s ->
    Serial.payload_of_string (String.sub s offset_payload (String.length s - size_packet))

  let packet = fun payload ->
    let payload = Serial.string_of_payload payload in
    let n = String.length payload in
    let msg_length = n + size_packet in
    let m = String.create msg_length in
    String.blit payload 0 m offset_payload n;
    m.[0] <- start_delimiter;
    m.[offset_length] <- Char.chr (msg_length lsr 8);
    m.[offset_length+1] <- Char.chr (msg_length land 0xff);
    let cs = compute_checksum m in
    m.[msg_length-1] <- Char.chr cs;
    m
end

let api_tx64_id = Char.chr 0x00
let api_tx16_id = Char.chr 0x01
let api_rx64_id = Char.chr 0x80
let api_rx16_id = Char.chr 0x81

let write_int64 = fun buf offset x ->
  for i = 0 to 7 do
    buf.[offset+7-i] <- Char.chr (Int64.to_int (Int64.shift_right x (8*i)) land 0xff)
  done

let write_int16 = fun buf offset x ->
  buf.[offset] <- Char.chr (x lsr 8);
  buf.[offset+1] <- Char.chr (x land 0xff)

let api_tx64 = fun ?(frame_id = 0) dest data ->
  assert (frame_id >=0 && frame_id < 0x100);
  let n = String.length data in
  assert (n <= 100);
  let l = 1 + 1 + 8 + 1 + n in
  let s = String.create l in
  s.[0] <- api_tx64_id;
  s.[1] <- Char.chr frame_id;
  write_int64 s 2 dest;
  s.[10] <- Char.chr 0;
  String.blit data 0 s 11 n;
  s
  
let api_tx16 = fun ?(frame_id = 0) dest data ->
  assert (frame_id >=0 && frame_id < 0x100);
  let n = String.length data in
  assert (n <= 100);
  let l = 1 + 1 + 2 + 1 + n in
  let s = String.create l in
  s.[0] <- api_tx16_id;
  s.[1] <- Char.chr frame_id;
  assert (dest >= 0 && dest < 0x10000);
  write_int16 s 2 dest;
  s.[4] <- Char.chr 0;
  String.blit data 0 s 5 n;
  s
  

let at_command_sequence = "+++"
let at_set_my = fun addr ->
  assert (addr >= 0 && addr < 0x10000);
  Printf.sprintf "ATMY%04x\n" addr
let at_exit = "ATCN\n"
let at_api_enable = "ATAP1\n"
