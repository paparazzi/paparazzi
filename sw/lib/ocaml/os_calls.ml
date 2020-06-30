(*
 * Support for obtaining os specific information at runtime
 *
 * Copyright (C) 2011 Eric Parsonage and Stephen Dwyer
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


let current_os = ref "not_set"

let read_process_output command =
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
  try Buffer.sub buffer 0 ((Buffer.length buffer) - 1)
  with _ -> Buffer.contents buffer

let contains s substring =
  try ignore (Str.search_forward (Str.regexp_string substring) s 0); true
  with Not_found -> false

let os_name = (
  if contains !current_os "not_set" then (
    current_os := read_process_output "uname" );
  !current_os
)
