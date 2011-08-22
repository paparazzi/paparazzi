(* $Id: ivy.ml,v 1.7 2004/09/11 17:06:59 poine Exp $ *)


type binding
type client
type client_event = Connected | Disconnected
type cb = client -> string array -> unit
type client_cb = client -> client_event -> unit

external send : string -> unit = "ivy_sendMsg"
external stop : unit -> unit = "ivy_stop"
external ext_init : string -> string -> string -> unit = "ivy_init"


let gensym = let n = ref 0 in fun p -> incr n; p ^ string_of_int !n
let cb_register = fun closure ->
  let s = gensym "callback_" in
  Callback.register s closure;
  s

let init = fun name ready ccb ->
  ext_init name ready (cb_register ccb)


external start : string -> unit = "ivy_start"
external ext_bind : string -> string -> binding = "ivy_bindMsg"

let bind = fun (cb:cb) regexp ->
  ext_bind (cb_register cb) regexp

external unbind : binding -> unit = "ivy_unbindMsg"


external name_of_client : client -> string = "ivy_name_of_client"
external host_of_client : client -> string = "ivy_host_of_client"



let marshal_tag = "MARSHAL"

let hexa_char = fun c ->
  assert(0 <= c && c < 16);
  if c < 10 then
    Char.chr (c + Char.code '0')
  else
    Char.chr (c + Char.code 'A' - 10)

let hexa_code = fun c ->
  if '0' <= c && c <= '9' then
    Char.code c - Char.code '0'
  else if 'A' <= c && c <= 'F' then
    Char.code c - Char.code 'A' + 10
  else failwith (Printf.sprintf "hexa_code: %c" c)


let hexa_of_string = fun s ->
  let n = String.length s in
  let h = String.create (n*2) in
  for i = 0 to n - 1 do
    let c = Char.code s.[i] in
    h.[2*i] <- hexa_char (c lsr 4);
    h.[2*i+1] <- hexa_char (c land 0xf)
  done;
  h

let string_of_hexa = fun h ->
  let n = String.length h / 2 in
  let s = String.create n in
  for i = 0 to n - 1 do
    s.[i] <- Char.chr (hexa_code h.[2*i] lsl 4 + hexa_code h.[2*i+1])
  done;
  s


let send_data = fun tag value ->
  let s = hexa_of_string (Marshal.to_string value []) in
  send (Printf.sprintf "%s %s %s" marshal_tag tag s)

let data_bind = fun cb tag ->
  let r = Printf.sprintf "%s %s (.*)" marshal_tag tag in
  bind (fun c a -> cb c (Marshal.from_string (string_of_hexa a.(0)) 0)) r
