let (//) = Filename.concat

let dnd_targets = [ { Gtk.target = "STRING"; flags = []; info = 0} ]
let parse_dnd =
  let sep = Str.regexp ":" in
  fun s ->
    match Str.split sep s with
      [s; c; m; f] -> (s, c, m, f)
    | _ -> failwith (Printf.sprintf "parse_dnd: %s" s)


let colors = [|"red"; "blue"; "green"; "orange"; "purple"|]

type plot = { da: GMisc.drawing_area ; mutable min: float; mutable max: float; size : int}
let create_plot = fun da ->
  { da = da;
    min = max_float;
    max = -. max_float;
    size = 100
  }
type values = { array: float option array; mutable index: int; color : string }
let create_values = fun size color_index ->
  let color = colors.(!color_index) in
  color_index := (!color_index + 1) mod Array.length colors;
  { array = Array.create size None; index = 0; color = color }
let add_value = fun plot v a ->
  a.array.(a.index) <- Some v;
  plot.min <- min plot.min v;
  plot.max <- max plot.max v

let dt = 0.5

let update_curves = fun plot curves () ->
  try
let {Gtk.width=width; height=height} = plot.da#misc#allocation in
  let dr = GDraw.pixmap ~width ~height ~window:plot.da () in
  dr#set_foreground (`NAME "white");
  dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();
  let margin = height / 10 in

  (* Time Graduations *)
  let context = plot.da#misc#create_pango_context in
  context#set_font_by_name ("sans " ^ string_of_int (margin/2));
  let layout = context#create_layout in

  let f = fun x s ->
    Pango.Layout.set_text layout s;
    let (w, h) = Pango.Layout.get_pixel_size layout in
    dr#put_layout ~x ~y:(height-h) ~fore:`BLACK layout in

  let t = dt *. float plot.size in
  f (width-width/plot.size) "0";
  f (width/2) (Printf.sprintf "-%.1f" (t/.2.));
  f 0 (Printf.sprintf "-%.1f" t);
	
  Hashtbl.iter
    (fun _ a ->
      (* Shift *)
      a.index <- (a.index + 1) mod (Array.length a.array);

      (* Draw *)
      let curve = ref [] in
      assert (plot.size = Array.length a.array);
      let dy = float (height-2*margin) /. (plot.max -. plot.min) in
      for i = 0 to plot.size - 1 do
	let i' = (i+a.index) mod plot.size in
	match a.array.(i') with
	  None -> ()
	| Some v ->
	    curve := ((i * width) / plot.size, height - margin - truncate ((v-.plot.min)*.dy)) :: !curve;
      done;
      if !curve <> [] then begin
	dr#set_foreground (`NAME a.color);
	dr#lines !curve;
      end;
      (new GDraw.drawable plot.da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap)
    curves;
  true
with exc -> prerr_endline (Printexc.to_string exc); true;;

let plot_window = fun () ->
  let plotter = GWindow.window ~title:"Plotter" () in
  plotter#show ();
  let width = 400 and height = 100 in
  let da = GMisc.drawing_area ~width ~height ~show:true ~packing:plotter#add () in
  let plot = create_plot da in
  let values = Hashtbl.create 3 in
  let color_index = ref 0 in

  let data_received = fun context ~x ~y data ~info ~time ->
    try
      let (sender, class_name, msg_name, field_name) = parse_dnd  data#data in
      let cb = fun a _sender values ->
	let v = float_of_string (Pprz.string_assoc field_name values) in
	add_value plot v a in
      
      let module P = Pprz.Messages (struct let name = class_name end) in
      let a = create_values plot.size color_index in
      Hashtbl.add values (class_name, msg_name, field_name) a;
      ignore (P.message_bind ~sender msg_name (cb a))
      with
	exc -> prerr_endline (Printexc.to_string exc)
  in
  da#drag#dest_set dnd_targets ~actions:[`COPY];
  ignore (da#drag#connect#data_received ~callback:data_received);
  ignore (GMain.Timeout.add (truncate (dt*.1000.)) (update_curves plot values))


let _ =
  let ivy_bus = ref "127.255.255.255:2010" in

  Arg.parse
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.255:2010"]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi messages" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  plot_window ();

  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done
