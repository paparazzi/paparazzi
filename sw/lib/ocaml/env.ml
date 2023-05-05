(*
 * Configuration handling
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

open Printf


let (//) = Filename.concat
let space_regexp = Str.regexp "[ \t]+"
let make_element = fun t a c -> Xml.Element (t,a,c)

let paparazzi_src =
  try
    Sys.getenv "PAPARAZZI_SRC"
  with
      _ -> "/usr/share/paparazzi"

let paparazzi_home =
  try
    Sys.getenv "PAPARAZZI_HOME"
  with
      _ -> Filename.concat (Sys.getenv "HOME") "paparazzi"

let paparazzi_conf = paparazzi_home // "conf"

let flight_plans_path = paparazzi_conf // "flight_plans"
let flight_plan_dtd = flight_plans_path // "flight_plan.dtd"

let srtm_path = paparazzi_home // "data" // "srtm"
let srtm_pprzgcs_path = (Sys.getenv "HOME") // ".local" // "share" // "pprzgcs" // "srtm"

(** Returns the list of directories where to look for modules
 * Default PAPARAZZI_HOME/conf/modules is always returned
 * Extra directories can be added with PAPARAZZI_MODULES_PATH
 * where where items are ':' separated and modules are in subfolders
 * of a 'modules' folder
 * ex:
 *   PAPARAZZI_MODULES_PATH=/home/me/pprz_modules
 *   - pprz_modules/modules
 *   -- module1
 *   --- module1.xml
 *   --- module1.c
 *   --- module1.h
 *   -- module2
 *   --- module2.xml
 *   --- module2.c
 *   --- module2.h
 *)
let modules_paths =
  let default_path = paparazzi_conf // "modules" in
  try
    let path = Sys.getenv "PAPARAZZI_MODULES_PATH" in
    let dirs = Str.split (Str.regexp ":") path in
    let paths = List.fold_left (fun dl dir ->
      let sub_dirs = List.fold_left (fun sdl sdir ->
        let d = dir // "modules" // sdir in
        if Sys.is_directory d then d :: sdl else sdl
      ) [] (Array.to_list (Sys.readdir (dir // "modules"))) in
      dl @ sub_dirs) [] dirs
    in
    paths @ [default_path]
  with
  | Sys_error _ | Not_found -> [default_path]

(** Returns the list of directories in PAPARAZZI_MODULES_PATH *)
let modules_ext_paths =
  try
    let path = Sys.getenv "PAPARAZZI_MODULES_PATH" in
    Str.split (Str.regexp ":") path
  with _ -> []

let icon_file = paparazzi_home // "data/pictures/penguin_icon.png"
let icon_gcs_file = paparazzi_home // "data/pictures/penguin_icon_gcs.png"
let icon_mes_file = paparazzi_home // "data/pictures/penguin_icon_msg.png"
let icon_rep_file = paparazzi_home // "data/pictures/penguin_icon_rep.png"
let icon_rtp_file = paparazzi_home // "data/pictures/penguin_icon_rtp.png"
let icon_log_file = paparazzi_home // "data/pictures/penguin_icon_log.png"
let icon_sim_file = paparazzi_home // "data/pictures/penguin_icon_sim.png"

let gconf_file = paparazzi_home // "conf" // "%gconf.xml"

let gcs_icons_path = paparazzi_home // "data" // "pictures" // "gcs_icons"
let gcs_default_icons_theme = "."

let get_gcs_icon_path = fun theme icon ->
  if Sys.file_exists (gcs_icons_path // theme // icon) then
    (* test if file exists *)
    gcs_icons_path // theme // icon
  else if Sys.file_exists (gcs_icons_path // icon) then
    (* else try default path *)
    gcs_icons_path // icon
  else
    (* or raise not found *)
    raise Not_found

let dump_fp = paparazzi_src // "sw" // "tools" // "generators" // "dump_flight_plan.out"

let default_module_targets = "ap|sim|nps|hitl"

let filter_absolute_path = fun path ->
  Str.replace_first (Str.regexp (paparazzi_home // "conf/")) "" path


(* filter settings and keep the ones without brackets *)
let filter_settings = fun settings ->
  let sl = Str.split (Str.regexp "[ ]+") settings in
  let sl = List.filter (fun s -> not (s.[0] = '[' && s.[String.length s - 1] = ']')) sl in
  String.concat " " sl


(* Run a command and return its results as a string. *)
let read_process command =
  let buffer_size = 2048 in
  let buffer = Buffer.create buffer_size in
  let bytes_ = Bytes.create buffer_size in
  let in_channel = Unix.open_process_in command in
  let chars_read = ref 1 in
  while !chars_read <> 0 do
    chars_read := input in_channel bytes_ 0 buffer_size;
    Buffer.add_substring buffer (Bytes.to_string bytes_) 0 !chars_read
  done;
  ignore (Unix.close_process_in in_channel);
  Buffer.contents buffer

let get_paparazzi_version = fun () ->
  try
    Str.replace_first (Str.regexp "[ \n]+$") "" (read_process (paparazzi_src ^ "/paparazzi_version"))
  with _ -> "UNKNOWN"


let key_modifiers_of_string = fun key ->
  let key_split = Str.split (Str.regexp "\\+") key in
  let keys = List.map (fun k ->
    match k with
    | "Ctrl" -> "<Control>"
    | "Alt" -> "<Alt>"
    | "Shift" -> "<Shift>"
    | "Meta" -> "<Meta>"
    | x -> x
  ) key_split in
  String.concat "" keys
