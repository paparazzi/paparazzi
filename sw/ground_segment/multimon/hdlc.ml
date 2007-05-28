
external init_gen : string -> Unix.file_descr = "ml_init_gen_hdlc"
external init_dec : string -> Unix.file_descr = "ml_init_dec_hdlc"
external write_data : string -> unit = "ml_gen_hdlc"
external get_data : unit -> string = "ml_get_hdlc"
external write_to_dsp : unit -> unit = "ml_write_to_dsp"

let write_period = 90
