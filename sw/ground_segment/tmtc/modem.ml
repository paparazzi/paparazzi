(*
 * Ground harware modem handling
 *
 * Copyright (C) 2004-2006 CENA/ENAC, Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *)


open Printf

module Protocol = struct
  (* Header: STX, length of (payload + checksum) *)
  (* Payload: tag, data *)
  (* Trailer : checksum, ETX *)

  let stx = Char.chr 0x02
  let etx = 0x03
  let index_start = fun buf ->
    Bytes.index buf stx

  let payload_length = fun buf start ->
    Char.code buf.[start+1] - 1

  let length = fun buf start ->
    let len = Bytes.length buf - start in
    if len >= 2 then
      Char.code buf.[start+1] + 3
    else
      raise Serial.Not_enough

  let checksum = fun msg ->
    let l = Bytes.length msg in
    let ck_a = ref 0 in
    for i = 1 to l - 3 do
      ck_a := Char.code msg.[i] lxor !ck_a
    done;
    !ck_a = Char.code msg.[l-2] && Char.code msg.[l-1] = etx

  let payload = fun msg ->
    let l = Bytes.length msg in
    assert(l >= 4);
    Serial.payload_of_string (Bytes.sub msg 2 (l-4))

  let packet = fun _payload ->
    failwith "Modem.Protocol.packet not implemented"
end

let msg_data = 0
let msg_error = 1
let msg_cd = 2
let msg_debug = 3
let msg_valim = 4

type status = {
  mutable valim : float;
  mutable cd : int;
  mutable error : int;
  mutable debug : int;
  mutable nb_byte : int;
  mutable nb_msg : int;
  mutable nb_err : int;
  mutable detected : int
}

let status = {
  valim = 0.;
  cd = 0;
  error = 0;
  debug = 0;
  nb_byte = 0;
  nb_msg = 0;
  nb_err = 0;
  detected = 0;
}
  (* FIXME *)
let valim = fun x -> float x *. 0.0162863 -. 1.17483
(* FIXME *)

let parse_payload = fun payload ->
  let payload = Serial.string_of_payload payload in
  status.detected <- 1;
  let len = Bytes.length payload in
  status.nb_byte <- status.nb_byte + len;
  status.nb_msg <- status.nb_msg + 1;
  let id = Char.code payload.[0] in
  if id = msg_data then
    Some (Bytes.sub payload 1 (len-1))
  else begin
    begin
      match id with
        | x when x = msg_error ->
          status.error <- (Char.code payload.[1])
        | x when x = msg_cd ->
          status.cd <- (Char.code payload.[1])
        | x when x = msg_debug ->
          status.debug <- (Char.code payload.[1])
        | x when x = msg_valim ->
          status.valim <- (valim (Char.code payload.[2] * 0x100 + Char.code payload.[1]));
        | _ -> (* Uncorrect id *)
          status.nb_err <- status.nb_err + 1
    end;
    None
  end
