type cmd_name = string
type data = string
type cmd = cmd_name * data

val send :  Unix.file_descr -> cmd -> unit

val receive : ?ack:(unit -> unit) -> (cmd -> 'a) -> (Unix.file_descr -> unit)
