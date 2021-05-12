(*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

(**
 * Radio control module for parsing XML config files
 *)

type channel = {
  cname: string; (* a.k.a. function in DTD *)
  min: int;
  max: int;
  neutral: int;
  average: bool; (* average data *)
  reverse: bool; (* reverse min/max parameters *)
}

let parse_channel = function
  | Xml.Element ("channel", attribs, []) as xml ->
    let bget = fun attrib ->
      try Xml.attrib xml attrib <> "0"
      with Xml.No_attribute _ -> false in
    let reverse = bget "reverse" in
    let min = ExtXml.int_attrib xml "min"
    and max = ExtXml.int_attrib xml "max" in
    { cname = Xml.attrib xml "function";
      min = if reverse then max else min;
      max = if reverse then min else max;
      neutral = ExtXml.int_attrib xml "neutral";
      average = bget "average";
      reverse;
      }
  | _ -> failwith "Radio.parse_channel: unreachable"


type pulse = PositivePulse | NegativePulse

type t = {
  filename: string;
  name: string;
  data_min: int;
  data_max: int;
  sync_min: int;
  sync_max: int;
  pulse_type: pulse;
  channels: channel list;
  xml: Xml.xml;
}

let from_xml = function
  | Xml.Element ("radio", attribs, channels) as xml ->
    let name = Xml.attrib xml "name" in
    { filename = "";
      name;
      data_min = ExtXml.int_attrib xml "data_min";
      data_max = ExtXml.int_attrib xml "data_max";
      sync_min = ExtXml.int_attrib xml "sync_min";
      sync_max = ExtXml.int_attrib xml "sync_max";
      pulse_type = begin match Xml.attrib xml "pulse_type" with
        | "POSITIVE" -> PositivePulse
        | "NEGATIVE" -> NegativePulse
        | _ -> failwith "Radio.from_xml: unknown pulse type"
      end;
      channels = List.map parse_channel channels;
      xml;
    }
  | _ -> failwith "Radio.from_xml: unreachable"

let from_file = fun filename ->
  let r = from_xml (Xml.parse_file filename) in
  { r with filename }
