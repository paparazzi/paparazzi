type message = {
    source : int;
    timestamp : int32;
    pprz_data : Serial.payload
  }

module Transport : Serial.PROTOCOL
(** Pprz frame stored by the logger *)

val parse : Serial.payload -> message


