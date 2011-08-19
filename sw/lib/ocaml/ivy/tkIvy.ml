
type channel
external main : unit -> unit = "ivy_TclmainLoop"
external ext_channelSetUp : Unix.file_descr -> string -> channel = "ivy_TclchannelSetUp"
let set_up_channel fd delete read =
  ext_channelSetUp fd (Ivy.cb_register read)
external close_channel : channel -> unit = "ivy_TclchannelClose"

