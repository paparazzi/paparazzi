(*
 *  $Id$
 *
 * Microcotrollers connection checking
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

module FbwMcu = struct
  let name = "Fly by wire"
  let sort = Flasher.Fbw
  let path = Env.fbw_dir
  let default_tty = Env.fbw_tty
end

module ApMcu = struct
  let name = "Autopilot"
  let sort = Flasher.Ap
  let path = Env.ap_dir
  let default_tty = Env.ap_tty
end

module ModemMcu = struct
  let name = "Ground Modem"
  let sort = Flasher.Modem
  let path = Env.modem_dir
  let default_tty = Env.modem_tty
end

module type MCU = sig
  val name : string
  val sort : Flasher.mcu
  val path  : string
  val default_tty : string
end

let nb_adc = 8

let get_2bytes = fun buf i ->
  (Char.code buf.[i] lsl 8) lor (Char.code buf.[i+1])

let (+.=) r v = r := !r +. v

let lls = fun values l ()->
  let sum_x = ref 0.
  and sum_y = ref 0.
  and sum_xy = ref 0.
  and sum_x2 = ref 0.
  and n = ref 0 in
  Hashtbl.iter
    (fun x y ->
      let x = float_of_string x
      and y = float_of_string y in
      incr n;
      sum_x +.= x;  
      sum_x2 +.= x*.x;  
      sum_y +.= y;
      sum_xy +.= x*.y)
    values;
  let n = float !n in
  let mx = !sum_x /. n
  and my = !sum_y /. n in
  let c_xy = mx *. my +. (!sum_xy -. mx *. !sum_y -. my *. !sum_x) /. n
  and s2_x = mx *. mx +. (!sum_x2 -. 2.*. mx *. !sum_x) /. n in
  let a = c_xy /. s2_x in
  let b = my -. a *. mx in

  Label.configure ~text:(Printf.sprintf "a=%f b=%f" a b) l

let rec check_adcs = fun button w tty make () ->
  let text = Tk.cget button `Text in
  make "TARGET=tx_adcs load" ();

  let f = Frame.create w in
  let destroy = fun () ->
    Tk.destroy f;
    Button.configure ~text button;
    Button.configure ~command:(check_adcs button w tty make) button in

  Button.configure ~text:"Close" button;
  Button.configure ~command:destroy button;

  let create_channel = fun i ->
    let fc = Frame.create f in

    let value = Textvariable.create () in
    
    let l = Label.create ~text:(string_of_int i) fc
    and v = Label.create  ~textvariable:value ~width:4 fc in

    let e = Entry.create ~width:4fc in
    let lb = Menubutton.create ~text:"Registered" fc in
    let lb_menu = Menu.create lb in
    Menubutton.configure lb ~menu:lb_menu;

    let values = Hashtbl.create 97 in (* I would prefer to find the values in the Menu but I do not know how to do it ! *)
    let register_values = fun ev ->
      let adc_value = Textvariable.get value
      and entry_value = Entry.get e in
      Hashtbl.add values adc_value entry_value;
      Menu.add_command lb_menu
	~label:(adc_value^":"^entry_value)
	~command:(fun () -> Hashtbl.remove values adc_value; Menu.delete ~first:`Active ~last:`Active lb_menu)
    in
    Tk.bind ~events:[`KeyPressDetail "Return"] ~action:register_values e;

    let lb' = Menubutton.create ~text:"Fit" fc in
    let lb_menu' = Menu.create lb' in
    Menubutton.configure lb' ~menu:lb_menu';
    let result = Label.create ~width:20 fc in
    Menu.add_command lb_menu' ~label:"Linear" ~command:(lls values result);



    Tk.pack [l; v] ~side:`Left;
    Tk.pack [e] ~side:`Left;
    Tk.pack [lb;lb'] ~side:`Left;
    Tk.pack [result] ~side:`Left;
    (fc, value)
  in

  let channels = Array.init nb_adc create_channel in

  (* Listen to tty input *)
  Tty.connect tty;
  Tty.add_formatted_input tty "\000\000" (2*(nb_adc+1)+1)
    (fun input ->
      for i = 0 to nb_adc - 1 do
	let vi = get_2bytes input (2+2*i) in
	Textvariable.set (snd channels.(i)) (string_of_int vi)
      done
    );
  Tk.bind ~events:[`Destroy] ~action:(fun _ -> Tty.deconnect tty) f;

  (***)
  let x = Button.create ~text:"Random"
      ~command:(fun () ->
	for i = 0 to nb_adc - 1 do
	  Textvariable.set (snd channels.(i)) (string_of_int (Random.int 1024))
	done
	       ) f in
  (***)


  Tk.pack [x];
  Tk.pack (List.map fst (Array.to_list channels)) ~side:`Top;

  Tk.pack [f]


module Make(Mcu : MCU) = struct
  let make = fun target () -> Flasher.make Mcu.sort Mcu.path target
  let make_test = fun target () -> Flasher.make Mcu.sort (Mcu.path^"/test") target
      
  let tty = Textvariable.create ()

  let connected = Textvariable.create ()

  let get_tty = fun () -> Textvariable.get tty

  let uart = ref Unix.stdin

  let connect_tty = fun () ->
    let tty = get_tty ()in
    if Textvariable.get connected = "1" then
      try
	Tty.connect tty
      with _ ->
	Console.write ~tags:["red"] (Printf.sprintf "Cannot open '%s'\n" tty)
    else
      try
	Tty.deconnect tty
      with
	Not_found -> 
	  Console.write ~tags:["red"] (Printf.sprintf "Device '%s' not opened\n" tty)

  let log_tty = fun () -> Tty.add_ttyinput (get_tty ()) Console.write

  let check_adcs_b = ref (Button.create Widget.default_toplevel)
      
  let create = fun parent ->
    let f = Frame.create ~borderwidth:4 ~relief:`Sunken parent in
    let f' = Frame.create f in

    let l = Label.create ~text:Mcu.name f'
    and b0 = Button.create ~text:"Erase" ~command:(make "erase") f'
    and b1 = Button.create ~text:"Check link" ~command:(make "check_arch") f'
    and b2 = Button.create ~text:"Write Fuses" ~command:(make "wr_fuses") f'
    and b3 = Button.create ~text:"Check UART" ~command:(fun () -> make_test (Printf.sprintf "TARGET=check_uart TTY=\"%s\" load" (Textvariable.get tty)) ()) f'
    and e = Entry.create ~textvariable:tty ~width:10 f'
    and c = Checkbutton.create ~text:"Connect tty" ~variable:connected ~command:connect_tty f'
    and c' = Checkbutton.create ~text:"Log tty" ~command:log_tty f' in
    
    Button.configure  b3;
    
    Entry.insert e ~index:`End ~text:Mcu.default_tty;

    check_adcs_b := Button.create ~text:"Check ADCS" f;
    Button.configure !check_adcs_b ~command:(fun () -> check_adcs !check_adcs_b f (get_tty ()) make_test ());
    
    
    Tk.pack [l];
    Tk.pack [b0;b1;b2] ~side:`Left;
    Tk.pack [e] ~side:`Left;
    Tk.pack [c;c'] ~side:`Left;
    Tk.pack [b3] ~side:`Left;
    Tk.pack [f'];
    Tk.pack [!check_adcs_b] ~side:`Left;
    f
end

module Fbw = Make(FbwMcu)
module Ap = Make(ApMcu)
module Modem = struct
  include Make(ModemMcu)

  let bat_a = Textvariable.create ()
  let bat_b = Textvariable.create ()
end

let create_sheet = fun sheets ->
  let f = Frame.create sheets in
  Notebook.create_sheet sheets "Hardware" f;

  let fbw = Fbw.create f
  and ap = Ap.create f
  and modem = Modem.create f in

  Button.configure ~state:`Disabled !Modem.check_adcs_b;

  let check_spi = Button.create ~text:"Check SPI" ~state:`Disabled ap in


  let check_downlink = Button.create ~text:"Check Downlink" ~state:`Disabled modem in

  let label = Label.create ~text:"Check the connectivity of your micro-controllers" f in

  Tk.pack ~anchor:`W [check_spi] ~side:`Left;
  Tk.pack ~anchor:`W [check_downlink];
  Tk.pack ~anchor:`W [label];
  Tk.pack ~anchor:`W [fbw; ap; modem]

