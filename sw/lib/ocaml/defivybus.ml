open Printf

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

let default_ivy_bus = 
 try ref (Sys.getenv "IVY_BUS" )
   with  Not_found -> ref  
   (if contains (read_process_output "uname") "Darwin" then       
      "224.255.255.255:2010" 
    else  
      "127.255.255.255:2010")


