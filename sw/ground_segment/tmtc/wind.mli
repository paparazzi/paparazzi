(** Aircraft id type *)
type id = string

val new_ac : id -> int -> unit
(** [new_ac id max_nb_sample] *)

val clear : id -> unit

val update : id -> float -> float -> unit
(** [update id ground_speed course_rad] *)

val get : id -> float * float * float * float * int
(** [get id] Returns [(wind_direction_rad, wind_speed, ac_avg_airspeed, stdev, nb_samples)] *)
