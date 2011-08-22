val main : unit -> unit
(** Glib main loop *)

type channel
(** Channel handled by the main loop *)

val set_up_channel : Unix.file_descr -> (unit -> unit) -> (channel -> unit) -> channel
(** [set_up_channel fd delete read] gives the opportunity to the main loop
to call [read] when data is available on [fd] and [delete] when [fd] is
closed *)

val close_channel : channel -> unit
(** Stops the handling of a channel by the main loop *)
