(*
 *  $Id$
 *
 * Servos calibration
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

let servo_update_period = 500 (* ms *)

let default_max = 2000
let default_min = 1000
let max_max = 2500.
let min_min = 500.
let default_neutral = 1500

let soi = string_of_int
let ios = int_of_string
let fos = float_of_string

let output_2bytes = fun tty x ->
  prerr_endline (string_of_int x);
  Tty.write_byte tty ((x land 0xff00) asr 8);
  Tty.write_byte tty (x land 0xff)

type selected = { widget : Widget.label Widget.widget; channel : int }

type channel = { frame : Widget.frame Widget.widget; print : out_channel -> unit }

let one_channel = fun _i parent slider selected servo_xml ->
  let frame = Frame.create parent in
  let no = Textvariable.get (VarXml.attrib servo_xml "no") in
  let n = Label.create frame ~text:no ~background:`Green in

  let create_label = fun id ->
    let v = VarXml.attrib servo_xml id in
    let l = Label.create frame ~textvariable:v in

    let button_action = fun _event ->
      Label.configure l ~relief:`Sunken;
      begin
	match !selected with
	  None -> () 
	| Some s -> Label.configure s.widget ~relief:`Flat
      end;
      selected := Some { widget=l; channel=int_of_string no };
      
      Scale.set slider (fos (Textvariable.get v));
      Scale.configure ~command:(fun value -> Textvariable.set v (string_of_int (truncate value))) slider in
    
    Tk.bind ~events:[`ButtonPress] ~action:button_action l;
    l in

  let maximum = create_label "max"
  and neutral = create_label "neutral"
  and minimum = create_label "min" in

  Tk.pack [n; maximum; neutral; minimum] ~side:`Top;
  frame


let rec send_selected_value = fun selected sending_state tty ->
  if Textvariable.get sending_state = "1" then begin
    Timer.set servo_update_period (fun () -> send_selected_value selected sending_state tty);
    match !selected with
      Some s ->
	Tty.write_byte tty 0;
	Tty.write_byte tty s.channel;
	output_2bytes tty (ios (Tk.cget s.widget `Text));
	Tty.write tty "\n";
	Tty.flush tty
    | None -> ()
  end

let create_sheet = fun sheets airframe_xml ->
  let top = Frame.create sheets in
  Notebook.create_sheet sheets "Servos" top;

  let tf = Frame.create ~relief:`Ridge ~borderwidth:1  top in

  let channels = Frame.create top in

  let sending_state = Textvariable.create ()
  and selected = ref None in

  let sending = fun () ->
    if Textvariable.get sending_state = "1" then begin
      let fbw_tty = Textvariable.get Hardware.Fbw.tty in
      Console.write (Printf.sprintf "Sending to %s\n" fbw_tty);
      Tty.connect fbw_tty;
      send_selected_value selected sending_state fbw_tty
    end else
      Tty.deconnect (Textvariable.get Hardware.Fbw.tty)
  in

  let program = fun () ->
    Flasher.make Flasher.Fbw (Printf.sprintf "%s/test" Env_conf.fbw_dir) "TARGET=setup_servos load" in

  let legend = Frame.create channels
  and slider = Scale.create tf ~orient:`Horizontal ~min:min_min ~max:max_max ~digits:4 ~label:"micro-seconds" ~resolution:1. ~length:300
  and pb = Button.create ~text:"Program board to send" ~command:program tf
  and rb = Checkbutton.create ~text:"Sending" ~variable:sending_state ~command:sending tf in
  let label = fun s -> Label.create legend ~text:s in
  Tk.pack [label "Channel: "; 
	label "Max (click to select): "; 
	label "Neutral (click to select): "; 
	label "Min (click to select): "] ~side:`Top;


  let servos_xml = Array.of_list (VarXml.children (VarXml.child airframe_xml "servos")) in
	
  let cs = Array.mapi (fun i s -> one_channel i channels slider selected s) servos_xml in

  Tk.pack (legend::Array.to_list cs) ~side:`Left;
  Tk.pack [tf];
  Tk.pack [pb]; Tk.pack [rb];
  Tk.pack [slider]; 
  Tk.pack [channels]
