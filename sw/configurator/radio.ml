(*
 *  $Id$
 *
 * Radio Control transmitter calibration
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

open Printf

let xml_radio = ref VarXml.empty


let zero = '\000'
let get_2bytes = fun buf i ->
  (Char.code buf.[i] lsl 8) lor (Char.code buf.[i+1])

type channel = {
    frame : Widget.frame Widget.widget;
    update : int -> unit
  }

let soi = string_of_int
let ios = int_of_string

let nb_neutral = 10

let ctl_values = [|"A";"B";"C";"D";"E";"F";"G";"H";"I"|]
let functions = [|"POWER";"ROLL";"PITCH";"DIRECTION";"MODE";"GAIN1";"GAIN2";"LLS";"CALIB"|]
let ctl_values = [|"A";"B";"C";"D";"E";"F";"G"|]
let functions = [|"POWER";"ROLL";"PITCH";"DIRECTION";"MODE";"GAIN1";"GAIN2"|]

let nb_channels = Array.length ctl_values


let popup_entry = fun parent set ->
  let t = Toplevel.create parent in
  let e = Entry.create ~takefocus:true t in
  let action = fun _ -> set (Entry.get e); Tk.destroy t in
  Tk.bind e ~events:[`KeyPressDetail "Return"] ~action:action;
  let b = Button.create ~text:"ok" ~command:action t in
  Tk.pack [e];
  Tk.pack [b]

let list_button = fun parent values tv ->
  let lb = Menubutton.create parent (* ~indicatoron:true *) ~textvariable:tv in
  let lb_menu = Menu.create lb in
  Menubutton.configure lb ~menu:lb_menu;
  Array.iter
    (fun v ->
      Menu.add_command lb_menu ~label:v ~command:(fun _ -> Textvariable.set tv v))
    values;
  Menu.add_command lb_menu ~label:"..." ~command:(fun _ -> popup_entry lb (fun x -> Textvariable.set tv x));
  lb



let one_channel = fun parent i xml_channel ->
  let frame = Frame.create ~relief:`Ridge ~borderwidth:1 ~width:5 parent in
  let n = Label.create frame ~text:(string_of_int i) ~background:`Green in

  let xml_get = VarXml.attrib xml_channel in
  let maximum = xml_get "max"
  and neutral = xml_get "neutral"
  and minimum = xml_get "min" in

  let rev = Textvariable.create () in

  let ctl = list_button frame ctl_values (xml_get "ctl")
  and function_name = list_button frame functions (xml_get "function") in
  let current = Label.create frame ~text:"1234" ~relief:`Sunken ~borderwidth:2;
  and maxi = Entry.create ~width:4 frame ~textvariable:maximum;
  and neutr = Entry.create ~width:4  frame ~textvariable:neutral;
  and mini = Entry.create ~width:4 frame ~textvariable:minimum
  and reverse = Checkbutton.create ~variable:rev frame
  and average = Entry.create ~width:2 ~textvariable:(xml_get "average") frame in

  let max_or_min = fun a b ->
    if Textvariable.get rev = "1" then min a b else max a b
  and min_or_max = fun a b ->
    if Textvariable.get rev = "1" then max a b else min a b in


  Tk.pack [n] ~side:`Top;
  Tk.pack [ctl; function_name] ~side:`Top;
  Tk.pack [current] ~side:`Top;
  Tk.pack [maxi; neutr; mini] ~side:`Top;
  Tk.pack [reverse] ~side:`Top;
  Tk.pack [average] ~side:`Top;

  let get_ma = fun () -> ios (Textvariable.get maximum)
  and get_mi = fun () -> ios (Textvariable.get minimum) in

  let update_max = fun v -> 
    let ma = max_or_min (get_ma ()) v in
    Textvariable.set maximum (soi ma)
  and update_min = fun v ->
    let mi = min_or_max (get_mi ()) v in
    Textvariable.set minimum (soi mi) in

  let sum_neutral = ref 0 and cpt_neutral = ref 1 in

  Tk.bind ~events:[`ButtonPress] ~action:(fun _ -> Textvariable.set maximum "00000") maxi;
  Tk.bind ~events:[`ButtonPress] ~action:(fun _ -> Textvariable.set minimum "99999") mini;
  Tk.bind ~events:[`ButtonPress] ~action:(fun _ -> cpt_neutral := 0; sum_neutral := 0) neutr;


  let update = fun value ->
    Label.configure current ~text:(string_of_int value);
    update_max value;
    update_min value;
    if !cpt_neutral < nb_neutral then begin
      incr cpt_neutral;
      sum_neutral := !sum_neutral + value;
      Textvariable.set neutral (soi (!sum_neutral/ !cpt_neutral));
    end
 in

  { frame = frame; update = update};;



let program_board = fun w cs () ->
  Flasher.make Flasher.Fbw (Printf.sprintf "%s/test" Env.fbw_dir) "TARGET=rc_transmitter load";
  
  let tty = Hardware.Fbw.get_tty () in
  Tty.connect tty;
  Tty.add_formatted_input tty "\000\000" (2*(nb_channels+1)+1)
    (fun input ->
      for i = 0 to String.length input -1 do
	printf "0x%X " (Char.code input.[i]);
      done;
      print_newline ();
      let channel_values = Array.init nb_channels (fun s -> get_2bytes input (2+2*s)) in
      for i = 0 to nb_channels - 1 do
	cs.(i).update channel_values.(i)
      done);
  Tk.bind ~events:[`Destroy] ~action:(fun _ -> Tty.deconnect tty) w
    


let create_display_frame = fun nf xml_radio ->
  let xml_channels = Array.of_list (VarXml.children xml_radio) in
  let cs = Array.mapi (fun i xml -> one_channel nf i xml) xml_channels in

  let p = Button.create ~text:"Get Radio-Control Values" ~command:(program_board nf cs) nf in

  Tk.pack [p];

  let legend = Frame.create nf in
  let label = fun s -> Label.create legend ~text:s in
  Tk.pack ~anchor:`E [label "Channel: "; 
	label "Control name: "; 
	label "Function name: "; 
	label "Current value: "; 
	label "Max (click to reset): "; 
	label "Neutral (click to reset): "; 
	label "Min (click to reset): ";
	label "Reverse";
	label "Averaged"] ~side:`Top;
	

  Tk.pack (legend::List.map (fun c -> c.frame) (Array.to_list cs)) ~side:`Left


let create_sheet = fun sheets ->
  let top = Frame.create sheets in
  Notebook.create_sheet sheets "Radio" top;

  TkXml.create top create_display_frame
