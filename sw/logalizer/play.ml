(*
 * $Id$
 *
 * Log player
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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

module Ground_Pprz = Pprz.Messages(struct let name = "ground" end)

let log = ref [||]

let load_log = fun window (adj:GData.adjustment) name ->
  let f = Ocaml_tools.open_compress name  in
  let lines = ref [] in
  try
    while true do
      let l = input_line f in
      try
	Scanf.sscanf l "%f %[^\n]" (fun t m -> lines := (t,m):: !lines)
      with
	_ -> ()
    done
  with
    End_of_file ->
      close_in f;
      log := Array.of_list (List.rev !lines);
      let start = fst !log.(0) in
      let end_ = fst !log.(Array.length !log - 1) in
      adj#set_bounds ~lower:start ~upper:end_ ();
      window#set_title (Filename.basename name)


let timer = ref None
let was_running = ref false

let stop = fun () ->
  match !timer with
    None -> ()
  | Some t -> GMain.Timeout.remove t; timer := None


let file_dialog ~title ~callback () =
  let sel = GWindow.file_selection ~title ~filename:"*.data[.*]" ~modal:true () in
  ignore (sel#cancel_button#connect#clicked ~callback:sel#destroy);
  ignore
    (sel#ok_button#connect#clicked
       ~callback:(fun () ->
	 let name = sel#filename in
	 sel#destroy ();
	 callback name));
  sel#show ()

let open_log = fun window adj () ->
  stop ();
  ignore (file_dialog ~title:"Open Log" ~callback:(fun name -> load_log window adj name) ())

let index_of_time log t =
  let t = t in
  let rec loop = fun a b ->
    if a >= b then a else
    let c = (a+b)/ 2 in
    if t <= fst log.(c) then loop a c else loop (c+1) b in
  loop 0 (Array.length log - 1)

let rec run log adj i speed =
  let (t, m) = log.(i) in
  Ivy.send (Printf.sprintf "%s" m);
  adj#set_value t;
  if i + 1 < Array.length log then
    let dt = fst log.(i+1) -. t in
    timer := Some (GMain.Timeout.add (truncate (1000. *. dt /. speed#value)) (fun () -> run log adj (i+1) speed; false))
      
let play adj speed =
  stop ();
  if Array.length !log > 1 then
    run !log adj (index_of_time !log adj#value) speed
  
  

let _ =
  let window = GWindow.dialog ~title:"Paparazzi Replay" ~width:300 () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let adj = GData.adjustment 
      ~value:0. ~lower:0. ~upper:1000. 
    ~step_incr:0.5 ~page_incr:1.0 ~page_size:1.0 () in

  let speed = object
    val mutable v = 1. method value = v method set_value x = v <- x
  end in

  let bus = ref "127.255.255.255:3333" in
  Arg.parse 
    [ "-b", Arg.String (fun x -> bus := x), "Bus\tDefault is 127.255.255.25:2010"]
    (fun x -> load_log window adj x)
    "Usage: ";
      

  let menubar = GMenu.menu_bar ~packing:window#vbox#pack () in
  let factory = new GMenu.factory menubar in
  let accel_group = factory#accel_group in
  let file_menu = factory#add_submenu "File" in
  let file_menu_fact = new GMenu.factory file_menu ~accel_group in
  
  ignore (file_menu_fact#add_item "Open Log" ~key:GdkKeysyms._O ~callback:(open_log window adj));  
  ignore (file_menu_fact#add_item "Play" ~key:GdkKeysyms._X ~callback:(fun () -> play adj speed));  
  ignore (file_menu_fact#add_item "Stop" ~key:GdkKeysyms._S ~callback:(fun () -> stop ()));  
  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);  


  let timescale = GRange.scale `HORIZONTAL ~adjustment:adj ~packing:window#vbox#pack () in

  window#add_accel_group accel_group;
  window#show ();

  Ivy.init "Paparazzi replay" "READY" (fun _ _ -> ());
  Ivy.start !bus;

  let world_update_time = fun _ vs ->
      speed#set_value (Pprz.float_assoc "time_scale" vs)
    in

    ignore (Ground_Pprz.message_bind "WORLD_ENV" world_update_time);

  GMain.Main.main ()
