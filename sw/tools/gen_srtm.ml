(*
 * Call to Makefile.ac with the appropriate attributes from conf.xml
 *
 * Copyright (C) 2003-2010 ENAC
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


let (//) =  Filename.concat
let var_dir = Env.paparazzi_home // "var"
let srtm_tmp_dir = var_dir // "srtm"
let srtm_data = var_dir // "srtm.data.bz2"

let area_list = ["Africa"; "Australia"; "Eurasia"; "Islands"; "North_America"; "South_America"]

(* MAIN *)
let () =
  (* reading file function *)
  let read_file_and_print = fun out area ->
    try
      let xml = Xml.parse_file (srtm_tmp_dir // area) in
      prerr_endline (Printf.sprintf "parsing file %s" area);
      let body = ExtXml.child xml "body" in
      let ul = ExtXml.child body "ul" in
      List.iter (fun li ->
        let a = ExtXml.child li "a" in
        let tile = Xml.attrib a "href" in
        let name = String.sub tile 0 7 in
        Printf.fprintf out "%s %s\n" name area
      ) (List.tl (Xml.children ul)) (* skip first link "Parent directory" *)
    with _ -> prerr_endline (Printf.sprintf "error with %s: skipping" area) (* skip file if error (DTD ?) *)
  in

  (* Fetch srtm pages *)
  if not (Sys.file_exists srtm_tmp_dir) then
    Unix.mkdir srtm_tmp_dir 0o755;
  List.iter (fun area ->
    let _ = Http.file_of_url ~dest:(srtm_tmp_dir // area) (Srtm.srtm_url // area) in
    (* remove first line (generate xml parsing error *)
    let _ = Sys.command ("sed -i '1d' "^(srtm_tmp_dir // area)) in
    ()
  ) area_list;
  (* reading file names in dir *)
    let file_names = Sys.readdir srtm_tmp_dir in

  (* Open temporary file *)
    let file, out = Filename.open_temp_file ~temp_dir:var_dir "srtm" ".data" in

  (* Parse files for xml and read them *)
    Array.iter (read_file_and_print out) file_names;

  (* Close file *)
    close_out out;

  (* Compress file *)
    let _ = Sys.command ("bzip2 -z "^file) in

  (* Move to final name *)
    Unix.rename (file^".bz2") srtm_data;
    prerr_endline ("Srtm data: "^srtm_data)





