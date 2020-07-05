(*
 * Dump a flight plan XML file
 *
 * Copyright (C) 2010 Gautier Hattenberger
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

let () =

  let fp_xml, dump_out =
    try Sys.argv.(1), Sys.argv.(2)
    with _ ->
      failwith "Dump FP: please choose a flight plan XML and an output file"
  in

  let fp = Flight_plan.from_file fp_xml in
  Gen_flight_plan.generate fp ~dump:true fp.Flight_plan.filename dump_out

