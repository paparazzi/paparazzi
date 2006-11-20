type t = <
    add_widget : GObj.widget -> unit;
    connect_shift_alt : (float -> unit) -> unit;
    set_agl : float -> unit;
    set_bat : float -> unit;
    set_color : string -> string -> unit;
    set_label : string -> string -> unit;
    hide_buttons : unit -> unit; 
    show_buttons : unit -> unit;
    connect : (unit -> unit) -> unit
>


val scrolled : GBin.scrolled_window

val add : Pprz.values -> string -> (unit -> unit) -> (unit -> unit) -> t



