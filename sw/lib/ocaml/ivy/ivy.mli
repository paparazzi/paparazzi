(** $Id: ivy.mli,v 1.8 2004/09/11 22:23:32 brisset Exp $ *)

(** Interface for ivy-c (http://www.tls.cena.fr/products/ivy/) *)

type binding
(** Identification of bindings (callback/message) *)

type client
(** Identification of client applications *)

val name_of_client : client -> string
val host_of_client : client -> string
(** Access to client identification *)

type client_event = Connected | Disconnected
(** Status of (de)connecting applications *)

type cb = client -> string array -> unit
(** Profile of message binding callback *)

type client_cb = client -> client_event -> unit
(** Profile of callback for (de)connecting applications *)

val init : string -> string -> client_cb -> unit
(** [init name ready cb] initializes the application as an IVY client,
identifying itself with [name], first sending the [ready] message. [cb]
will be called each time a new application (de)connects to this IVY bus. *)

val start : string -> unit
(** [start bus] starts the connection to machine/network/port specified in
[bus]. Syntax for [bus] is ["IPaddress:port"] *)

val bind : cb -> string -> binding
(** [bind cb regexp] binds callback [cb] to messages matching the [regexp]
regular expression. [cb] will be called with the array of matching groups
defined in [regexp]. *)

val send : string -> unit
(** [send message] sends a message to the IVY initialized bus *)

val stop : unit -> unit
(** Exits the main loop *)

val unbind : binding -> unit
(** Removes a message binding *)

val send_data : string -> 'a -> unit
(** [send_data tag value] marshals [value] into a string and sends it with
[tag] over the IVY bus *)

val data_bind : (client -> 'a -> unit) -> string -> binding
(** [data_bind cb tag] binds [cb] to IVY messages sent with [send_data] and
tagged with [tag]. This operation IS NOT type safe.*)

(***)

val cb_register : ('a -> 'b) -> string
