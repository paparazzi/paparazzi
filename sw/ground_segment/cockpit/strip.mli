type t = <
    add_widget : GObj.widget -> unit;
    connect_shift_alt : (float -> unit) -> unit;
    connect_shift_lateral : (float -> unit) -> unit;
    connect_launch : (float -> unit) -> unit;
    connect_kill : (float -> unit) -> unit;
    connect_mode : (float -> unit) -> unit;
    connect_flight_time : (float -> unit) -> unit;
    connect_apt : (float -> unit) -> unit;
    set_agl : float -> unit;
    set_bat : float -> unit;
    set_throttle : ?kill:bool -> float -> unit;
    set_speed : float -> unit;
    set_airspeed : float -> unit;
    set_climb : float -> unit;
    set_color : string -> string -> unit;
    set_label : string -> string -> unit;
    hide_buttons : unit -> unit; 
    show_buttons : unit -> unit;
    connect : (unit -> unit) -> unit
>


val scrolled : GBin.scrolled_window

val add : Pprz.values -> string -> (unit -> unit) -> (unit -> unit) -> t



