val main : unit -> unit
(** Starts the loop which handles asynchronous communications. The standard
version does not return until IVY is explictly stopped *)

type channel
(** Channel handled by the main loop *)

val set_up_channel : Unix.file_descr -> (unit -> unit) -> (channel -> unit) -> channel
(** [set_up_channel fd delete read] gives the opportunity to the main loop
to call [read] when data is available on [fd] and [delete] when [fd] is
closed *)

val close_channel : channel -> unit
(** Stops the handling of a channel by the main loop *)

type timer
(** Timer identifier *)

val timer : int -> int -> (timer -> unit) -> timer
(** [timer n ms cb] sets a timer which will call [n] times the callback [cb]
with a period of [ms] milliseconds *)

val remove_timer : timer -> unit
(** [remove_timer t] stops the timer [t] *)

