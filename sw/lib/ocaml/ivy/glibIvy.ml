
type channel
external main : unit -> unit = "ivy_GtkmainLoop"
external ext_channelSetUp : Unix.file_descr -> string -> channel = "ivy_GtkchannelSetUp"
let set_up_channel fd delete read =
  ext_channelSetUp fd (Ivy.cb_register read)
external close_channel : channel -> unit = "ivy_GtkchannelClose"

