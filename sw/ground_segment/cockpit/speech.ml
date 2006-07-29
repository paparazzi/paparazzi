let active = ref false

let say = fun s ->
  if !active then
    ignore (Sys.command (Printf.sprintf "spd-say '%s'&" s))
