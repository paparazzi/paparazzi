val init_gen : string -> Unix.file_descr
val write_data : string -> unit
val write_to_dsp : unit -> unit

val write_period : int
(** (ms) Recommended period to call write_to_dsp *)

val init_dec : string -> Unix.file_descr
val get_data : unit -> string
