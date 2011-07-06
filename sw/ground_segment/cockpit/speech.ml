let active = ref false

let current_os = ref "not_set"

(* These two functions are from sw/lib/defivybus.ml *)
let read_process_output command =
  let buffer_size = 2048 in
  let buffer = Buffer.create buffer_size in
  let string = String.create buffer_size in
  let in_channel = Unix.open_process_in command in
  let chars_read = ref 1 in
  while !chars_read <> 0 do
    chars_read := input in_channel string 0 buffer_size;
    Buffer.add_substring buffer string 0 !chars_read
  done;
  ignore (Unix.close_process_in in_channel);
  Buffer.contents buffer

let contains s substring =
  try ignore (Str.search_forward (Str.regexp_string substring) s 0); true
    with Not_found -> false
      
let say = fun s ->
  (
  if !active then (
   (* Checks if the os is known and gets the uname if not *)
   if contains !current_os "not_set" then (
      current_os := read_process_output "uname";
    );
    (* If the os is Darwin, then use "say" *)
    if contains !current_os "Darwin" then (
      ignore (Sys.command (Printf.sprintf "say '%s'&" s));
    )
    (* If the os is anything else, use "spd-say" (add additional cases here if necessary) *)
    else (
      ignore (Sys.command (Printf.sprintf "spd-say '%s'&" s));
    )
  ));;
