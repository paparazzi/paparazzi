open Types

  exception BadChecksum
  exception BadEndSequence

  let rec skip_until_start_sequence = fun gps ->
    while input_byte gps <> 0xA0 do () done;
    if input_byte gps <> 0xA2 then skip_until_start_sequence gps

  let send_start_sequence = fun gps ->
    output_byte gps 0xA0;
    output_byte gps 0xA2

  let get_end_sequence = fun gps ->
    if input_byte gps <> 0xB0 || input_byte gps <> 0xB3 then
      raise BadEndSequence

  let send_end_sequence = fun gps ->
    output_byte gps 0xB0;
    output_byte gps 0xB3

  let checksum = fun data ->
    let cs = ref 0 in
    String.iter (fun c -> cs := (!cs + Char.code c) land 0x7fff) data;
    !cs

  let receive = fun gps ->
    let gps = Unix.in_channel_of_descr gps in
    skip_until_start_sequence gps;
    let length_h = input_byte gps in
    let length_l = input_byte gps in
    let length = (length_h lsl 8) lor length_l in
    let payload = String.create length in
    for i = 0 to length - 1 do
      payload.[i] <- input_char gps;
    done;
    let checksum_h = input_byte gps in
    let checksum_l = input_byte gps in
    get_end_sequence gps;
    if checksum payload <> (checksum_h lsl 8) lor checksum_l then
      raise BadChecksum;
    payload

  let output_2bytes = fun gps x ->
    output_byte gps ((x land 0xff00) lsr 8);
    output_byte gps (x land 0xff)

  let send = fun gps payload ->
    let n = String.length payload in
    assert(n < 1023);
    send_start_sequence gps;
    output_2bytes gps n;
    String.iter (output_char gps) payload;
    output_2bytes gps (checksum payload);
    send_end_sequence gps;
    flush gps


let send1 = fun gps c ->
  Printf.printf "send 0x%2x\n" c; flush stdout

let send_int32 = fun gps x ->
  Printf.printf "send32 0x%8x\n" x; flush stdout


let get = fun gps n ->
  let buf = String.create n in
  buf.[0] <- Char.chr 0x79;
  assert(input gps buf 5 (n-5) = n-5);
  buf

let log_info = Bytes [ "MID", 1;
		       "S_First", 1; "S_Last", 1; 
		       "A_First", 4; "A_Last", 4; "A_Start", 4; 
		       "Size", 4 ]
let log_data = Bytes [ "MID", 1; "Start", 4; "Data", 256*2 ]

let extended_nav = Bytes [
  "MID", 1;
  "Latitude", 4; "Longitude", 4; "Altitude", 4;
  "Speed", 4; "ClimbRate", 4; "Course", 4;
  "Mode", 1;
  "Year", 2; "Month", 1; "Day", 1; "Hour", 1; "Minute", 1;
  "Second", 2;
  "GDOP", 1; "HDOP", 1; "PDOP", 1; "TDOP", 1; "VDOP", 1;]


let get_message = fun gps expected ->
  let s = size_of_message expected in
  let data = get gps s in
  (expected, data, 0)
  

let log_poll_info = fun gps ->
  send1 gps 0xbb;
  get_message gps log_info
  
let log_read = fun gps a ->
  send1 gps 0xb8;
  send_int32 gps a;
  get_message gps log_data
  
