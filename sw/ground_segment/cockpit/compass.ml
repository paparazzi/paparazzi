open Printf
open Latlong

module Tm_Pprz = Pprz.Messages (struct let name = "telemetry" end)

let width = 200
let height = width
let background = `NAME "grey"
let fore = `BLACK

let arrow = [1,1; 2,1; 0,3; -2,1; -1,1; -1,-3; 1,-3; 1,1]

let rot = fun angle ->
  let angle = (Deg>>Rad)(-. angle) in
  let ca = cos angle and sa = sin angle in
  fun (x,y) -> 
    let x = float x and y = float y in
    (truncate (ca*.x-.sa*.y), truncate (sa*.x+.ca*.y))

let n = 100
let circle = fun (dr:GDraw.pixmap) (x,y) r ->
  let r = float r in
  let points = Array.init n
      (fun i ->
	let a = float i /. float n *. 2.*.pi in
	(x + truncate (r*.cos a), x + truncate (r*.sin a))) in
  dr#polygon (Array.to_list points)
      
let draw = fun da desired_course course distance ->
  let {Gtk.width=width; height=height} = da#misc#allocation in
  let dr = GDraw.pixmap ~width ~height ~window:da () in
  dr#set_foreground background;
  dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();
  let s = (min width height) / 8 in

  (* Arrow *)
  let rotation = rot (desired_course -. course) in
  let translate = fun (x, y) -> (4*s+x, 4*s-y) in
  let points = List.map (fun (x, y) -> translate (rotation (x*s/2,y*s/2))) arrow in
  dr#set_foreground fore;
  dr#polygon ~filled:true points;
  circle dr (4*s,4*s) (2*s);
  circle dr (4*s,4*s) (3*s);
  
  (* Text *)
  let context = da#misc#create_pango_context in
  context#set_font_by_name (sprintf "sans %d" (s/3));
  let print_string = fun x y string ->
    let layout = context#create_layout in
    let fd = Pango.Context.get_font_description (Pango.Layout.get_context layout) in
    Pango.Font.modify fd ~weight:`BOLD ();
    context#set_font_description fd;
    let from_codeset = "ISO-8859-15"
    and to_codeset = "UTF-8" in
    Pango.Layout.set_text layout (Glib.Convert.convert ~from_codeset ~to_codeset string);
    let (w,h) = Pango.Layout.get_pixel_size layout in
    dr#put_layout ~x:(x-w/2) ~y:(y-h/2) ~fore layout in
  print_string (7*s) s (sprintf "%.0f m" distance);
  print_string (7*s) (s/2) "Dist.";
  print_string s s (sprintf "%.0f°" desired_course);
  print_string s (s/2) "Brg";
  print_string s (7*s) (sprintf "%.0f°" course);
  print_string s (7*s+s/2) "Track";

  (* Cardinal points *)
  let rotation = rot (-. course) in
  let cards = [(0, 10, "N"); (0, -10, "S"); (10, 0, "E"); (-10, 0, "W");
	       (7, 7, "NE"); (7, -7, "SE");(-7,-7,"SW");(-7,7,"NW")] in
  List.iter (fun (x,y,string)->
    let (x,y) = translate (rotation ((x*5*s)/20, (y*5*s)/20)) in
    print_string x y string)
    cards;  
  
  (new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap

(*********************** Main ************************************************)
let _ =
  let ivy_bus = ref "127.255.255.255:2010" in

  Arg.parse
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.255:2010"]
    (fun x -> prerr_endline ("WARNING: don't do anything with "^x))
    "Usage: ";

  (** Connect to the Ivy bus *)
  Ivy.init "Paparazzi UGV Compass" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (** Open the window *)
  let icon = GdkPixbuf.from_file Env.icon_file in
  let window = GWindow.window ~icon ~title:"UGV Compass" () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let da = GMisc.drawing_area ~width ~height ~packing:window#add () in
  da#misc#realize ();

  (* Listening messages *)
  let course = ref 0. in
  let get_navigation = fun _ values ->
    let desired_course = float (Pprz.int_assoc "desired_course" values) /. 10.
    and distance = sqrt (Pprz.float_assoc "dist2_wp" values) in
    draw da desired_course !course distance in
  ignore (Tm_Pprz.message_bind "NAVIGATION" get_navigation);
  let get_gps = fun _ values ->
    course := float (Pprz.int_assoc "course" values) /. 10. in
  ignore (Tm_Pprz.message_bind "GPS" get_gps);

  (** Start the main loop *)
  window#show ();
  GMain.main () 
  
