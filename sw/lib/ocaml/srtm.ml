(*
 * Acces functions to SRTM data (http://edcftp.cr.usgs.gov/pub/data/srtm)
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec, Pascal Brisset, Antoine Drouin
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

open Latlong


let (//) = Filename.concat

type error = string
exception Tile_not_found of string

let srtm_url = "http://dds.cr.usgs.gov/srtm/version2_1/SRTM3"

let error = fun string ->
  Printf.sprintf "download %s/???/%s.hgt.zip in data/srtm/" srtm_url string

let tile_size = 1201

(* Previously opened tiles *)
let htiles = Hashtbl.create 13

(* Path to data files *)
let path = ref ["."]

let add_path = fun p -> path := p :: !path

let open_compressed = fun f ->
  Ocaml_tools.open_compress (Ocaml_tools.find_file !path f)

let find = fun tile ->
  try Hashtbl.find htiles tile with
      Not_found ->
        let (bottom, left) = tile in
        let tile_name =
          Printf.sprintf "%c%.0f%c%03.0f" (if bottom >= 0. then 'N' else 'S') (abs_float bottom) (if left >= 0. then 'E' else 'W') (abs_float left) in
        try
          let f = open_compressed (tile_name ^".hgt") in
          let n = tile_size*tile_size*2 in
          let buf = Bytes.create n in
          really_input f buf 0 n;
          Hashtbl.add htiles tile buf;
          buf
        with Not_found ->
          raise (Tile_not_found tile_name)


let get = fun tile y x ->
  let tile = find tile in
  let pos = (2*((tile_size-y)*tile_size+x)) in
  (((Char.code (Bytes.get tile pos) land 127) lsl 8) lor Char.code (Bytes.get tile (pos+1))) - ((Char.code (Bytes.get tile pos) lsr 7) * 256 * 128)

let of_wgs84 = fun geo ->
  let lat = (Rad>>Deg)geo.posn_lat
  and long = (Rad>>Deg)geo.posn_long in
  let bottom = floor lat and left = floor long in
  let tile = (bottom, left) in
  get tile (truncate ((lat-.bottom)*.1200.+.0.5)) (truncate ((long-.left)*.1200.+.0.5))

let of_utm = fun utm ->
  of_wgs84 (Latlong.of_utm WGS84 utm)

let available = fun geo ->
  try ignore(of_wgs84 geo); true with _ -> false

let area_of_tile = fun tile ->
  let area = open_compressed "srtm.data.bz2" in
  let rec _area_of_tile = fun () ->
    try
      let ib = Scanf.Scanning.from_channel area in
      Scanf.bscanf ib "%s %s\n" (fun t a ->
        if t = tile then a
        else _area_of_tile ())
    with
      | End_of_file -> raise (Tile_not_found tile)
      | _ -> _area_of_tile ()
  in
  _area_of_tile ()


(* field size in bytes *)
let field_size = 2
(* srtm file line size in bytes *)
let line_size = 1201 * field_size
let byte_shift_factor = 256
let sec3_steps_number = 1200

let delta_srtm3_meters = 90.0

let safety_height_meters = 20

let altitude_min_meters = -30000

let step_d = 100.

let step_alpha = Latlong.pi /. 10.

(** [find_horizon_slope geo alti_m ...] *)
let horizon_slope = fun geo r psi alpha d ->
  let { utm_zone = z; utm_x = utm_q; utm_y = utm_p} = Latlong.utm_of WGS84  geo in

  let alpha2 = 2.0 *. alpha in
  let heading_psi_2alpha = pi /. 2.0 -. psi -. alpha in

  let rec calc_horizon = fun sum_cos_alpha_i_slope_alpha_i  sum_cos_alpha_i alpha_i ->
    if alpha_i > alpha2 then atan ( sum_cos_alpha_i_slope_alpha_i /. sum_cos_alpha_i )
    else
      let max_slope = ref 0.0 in
      let dj = ref 0.0 in
      begin
        while !dj < d +. 1.0 do
          let s_utm = utm_q +. !dj *. cos( heading_psi_2alpha +. alpha_i) in
          let t_utm = utm_p +. !dj *. sin( heading_psi_2alpha +. alpha_i) in
          let h = of_utm { utm_zone = z; utm_x = s_utm; utm_y = t_utm} in
          begin
            begin
              let slope =  float_of_int (h-r ) /. !dj in
              if slope > !max_slope then max_slope := slope;
      (* Printf.printf " h %d dj %.2f \n" h !dj; *)
            end;
          end;
          dj := !dj +. step_d;
        done;
      (* Printf.printf "alpha_i %.2f max_slope %.2f \n" alpha_i !max_slope; *)
        calc_horizon (sum_cos_alpha_i_slope_alpha_i +. !max_slope *. (cos (alpha_i -. alpha))) (sum_cos_alpha_i +. (cos (alpha_i -. alpha))) (alpha_i +. step_alpha)
      end
  in
  begin
    (*  Printf.printf "debut calcul \n"; *)
    calc_horizon 0.0 0.0 0.0;
  end
