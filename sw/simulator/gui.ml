(* ocamlc -w a -I +lablgtk2 lablgtk.cma lablgnomecanvas.cma gtkInit.cmo gui.ml -o gui.out *)
 
let main () =
  
  (* window *)
  let window = GWindow.window ~title: "Paparazzi simulator"
      ~border_width: 5 ~width: 400 ~height: 200 () in
  ignore (window#connect#destroy ~callback:GMain.quit);
  
 
  let hb = GPack.hbox ~border_width:5  ~spacing:5 ~packing:window#add () in
 
 
  (* wind *)
  let frame_w = GBin.frame ~label:"Wind" ~shadow_type:`IN ~packing:hb#pack () in
   
  let vb_w = GPack.vbox ~packing:frame_w#add () in
 
  let adj_d =
    GData.adjustment ~lower:0. ~upper:100. ~step_incr:1. ~page_incr:10. () in
  let sc_d = GRange.scale `HORIZONTAL ~adjustment:adj_d ~draw_value:false
      ~packing:vb_w#pack () in
 
  let adj_s =
    GData.adjustment ~lower:0. ~upper:100. ~step_incr:1. ~page_incr:10. () in
  let sc_s = GRange.scale `HORIZONTAL ~adjustment:adj_s ~draw_value:false
      ~packing:vb_w#pack () in
 
  (* infrared *)
  let frame_i = GBin.frame ~label:"Infrared" ~shadow_type:`IN ~packing:hb#pack () in
   
  let vb_i = GPack.vbox ~packing:frame_i#add () in
 
  let adj_i =
    GData.adjustment ~lower:0. ~upper:100. ~step_incr:1. ~page_incr:10. () in
  let sc_i = GRange.scale `HORIZONTAL ~adjustment:adj_i ~draw_value:false
      ~packing:vb_i#pack () in
  
  let button = GButton.button ~use_mnemonic:true ~label:"_Coucou" ~packing:(hb#pack ~padding:5) () in
  ignore(button#connect#clicked ~callback:
    (fun () -> prerr_endline "Coucou"));

 
 
  window#show ();
  GMain.Main.main ()
     
let _ = main ()

