let log = ref [||]

let load_log = fun window (adj:GData.adjustment) name ->
  let f =
    if Filename.check_suffix name ".gz"
    then Unix.open_process_in (Printf.sprintf "zcat %s" name)
    else open_in name in
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
      let end_ = fst !log.(Array.length !log - 1) -. start in
      adj#set_bounds ~upper:end_ ();
      window#set_title (Filename.basename name)


let timer = ref None
let was_running = ref false

let stop = fun () ->
  match !timer with
    None -> ()
  | Some t -> GMain.Timeout.remove t; timer := None


let file_dialog ~title ~callback () =
  let sel = GWindow.file_selection ~title ~filename:"*.xml" ~modal:true () in
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
  let t = t +. fst log.(0) in
  let rec loop = fun a b ->
    if a >= b then a else
    let c = (a+b)/ 2 in
    if t <= fst log.(c) then loop a c else loop (c+1) b in
  loop 0 (Array.length log - 1)

let rec run log adj i speed =
  let (t, m) = log.(i) in
  Ivy.send (Printf.sprintf "%.2f %s" t m);
  adj#set_value (t -. fst log.(0));
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

  let speed = GData.adjustment ~value:1. ~lower:0.05 ~upper:10. 
    ~step_incr:0.25 ~page_incr:1.0 () in

  let bus = ref "127.255.255.255:2010" in
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
  let speed_button = GEdit.spin_button ~adjustment:speed ~rate:0. ~digits:2 ~width:50 ~packing:window#vbox#add () in

  (** #move_slider is not working ??? **) ignore (timescale#event#connect#button_release ~callback:(fun _ -> if !was_running then play adj speed; false));
  ignore (timescale#event#connect#button_press ~callback:(fun _ -> was_running := !timer <> None; stop (); false));

  window#add_accel_group accel_group;
  window#show ();

  Ivy.init "Paparazzi replay" "READY" (fun _ _ -> ());
  Ivy.start !bus;

  GMain.Main.main ()
