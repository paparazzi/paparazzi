type channel
type delete_channel_cb = unit -> unit
type timer
type timer_cb = timer -> unit

external ext_timer : int -> int -> string -> timer = "ivy_timerRepeatafter"

let timer = fun n t cb ->
  let closure_name = Ivy.cb_register cb in
  ext_timer n t closure_name

external remove_timer : timer -> unit = "ivy_timerRemove"

external main : unit -> unit = "ivy_mainLoop"

external ext_channelSetUp : Unix.file_descr -> string -> channel = "ivy_channelSetUp"
external close_channel : channel -> unit = "ivy_channelClose"


type read_channel_cb = channel -> unit
let set_up_channel fd delete read =
  ext_channelSetUp fd (Ivy.cb_register read)
