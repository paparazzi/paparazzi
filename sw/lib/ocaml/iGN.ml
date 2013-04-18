(*
 * Handling IGN tiles
 *
 * Copyright (C) 2004-2006 ENAC, Pascal Brisset, Antoine Drouin
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

let (//) = Filename.concat

module LL = Latlong

type tile_t = {
  key : int * int; (* LambertIIe meters / size_m *)
  sw_corner : Latlong.geographic;
  ne_corner : Latlong.geographic
};;

let size_px = 250
let size_m = (size_px * 25) / 10

let tile_size = size_px , size_px

let cache_path = ref "/var/tmp"
let data_path = ref "/path_to_ign_files"

let tile_of_geo = fun wgs84 ->
  let lbt = LL.lambertIIe_of wgs84 in
  let kx = lbt.LL.lbt_x / size_m
  and ky = lbt.LL.lbt_y / size_m in
  let x0 = kx * size_m and y0 = ky * size_m in
  let x1 = x0 + size_m and y1 = y0 + size_m in
  let sw_corner= LL.of_lambertIIe {LL.lbt_x=x0; lbt_y=y0}
  and ne_corner =  LL.of_lambertIIe {LL.lbt_x=x1; lbt_y=y1} in
  { key = (kx, ky);
    sw_corner = sw_corner;
    ne_corner = ne_corner }

let get_tile = fun tile ->
  let (kx, ky) = tile.key in
  let dalle_x = (kx * size_m) / 10000
  and dalle_y = 267 - ((ky*size_m) / 10000) in
  let dalle_name = Printf.sprintf "F%03d_%03d" dalle_x dalle_y in
  let ix = (kx mod (10000 / size_m)) * size_px
  and iy = 4000 - size_px - ((ky mod (10000 / size_m)) * size_px) in
  let png_name = Printf.sprintf "%s_%dx%d+%d+%d.png" dalle_name size_px size_px ix iy in

  (* Look in the cache *)
  let cache_name = !cache_path // png_name in
  if Sys.file_exists cache_name then
    cache_name
  else (* Not there: crop it from the original IGN dalle *)
    let file = !data_path // dalle_name ^ ".png" in
    let png_file = !cache_path // png_name in
    let sub_tile = Printf.sprintf "convert -crop %dx%d+%d+%d %s %s" size_px size_px ix iy file png_file in
    let x = Sys.command sub_tile in
    if x <> 0 then failwith sub_tile;
    png_file


