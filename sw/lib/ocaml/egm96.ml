(*
 * EGM96 geoid model
 *
 * Copyright (C) 2009 ENAC, Pascal Brisset
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

let ncols = 1440
let nrows = 721
let n = ncols * nrows * 2
let buf = Bytes.create n
let data =
  lazy (
    let path = [Env.paparazzi_home // "data" // "srtm"] in
    let f = Ocaml_tools.open_compress (Ocaml_tools.find_file path "WW15MGH.DAC") in
    really_input f buf 0 n;
    buf)


(* http://earth-info.nima.mil/GandG/wgs84/gravitymod/egm96/binary/binarygeoid.html *)
let of_wgs84 = fun geo ->
  let egm96_data = Lazy.force data in

  let lat = truncate ((Rad>>Deg) (norm_angle geo.posn_lat))
  and lon = truncate ((Rad>>Deg) (norm_angle geo.posn_long)) in
  let ilat = (90-lat)*4      (* 15' == 4 entries per degree *)
  and ilon = (lon+if lon < 0 then 360 else 0)*4 in

  let i = (2*(ilat*ncols+ilon)) in

  let x = Char.code (Bytes.get egm96_data i) lsl 8 + Char.code (Bytes.get egm96_data (i+1)) in

  float ((x lsl 16) asr 16) /. 100.
