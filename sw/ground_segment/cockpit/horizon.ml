open Latlong

let pitch_scale = fun pitch radius -> truncate (pitch *. float radius)

let rotate = fun (x, y) t -> 
  let (nx,ny) =  ((x*. cos t -. y *. sin t), (x*. sin t +. y *. cos t)) 
  in
    (nx, ny)

let rotate_with_center = fun (x, y) (cx, cy) rad ->
  let (tx, ty) = rotate (x-.cx , y-. cy) rad in
    (tx+. cx, ty+. cy)

let int_rotate_with_center = fun (x, y) (cx, cy) rad ->
  let (nx, ny) = rotate_with_center 
    (float x, float y) (float cx, float cy) rad in
    (int_of_float nx, int_of_float ny)


class h = fun ?packing width  ->
  let height = width in
  let da = GMisc.drawing_area ~width ~height ?packing () in
  object (self)
    val drawing_area = da
    method draw = fun roll pitch ->
      let {Gtk.width=width; height=height} = da#misc#allocation in
      let dr = GDraw.pixmap ~width ~height ~window:da () in
	
      let draw_arc = fun ~color ~cx ~cy radius start stop ->
	dr#set_foreground  color;
	let start = (Rad>>Deg)start
	and stop = (Rad>>Deg)stop in
	let x = cx - radius 
	and y = cy - radius
	and width = radius * 2 in
	  dr#arc ~x ~y ~width ~height:width ~filled:true ~start ~angle:(stop -. start) () in
	  
      let draw_grad_with_l = fun color cx cy roll d l ->
	let dx = l/2 in
	let (ax, ay) = int_rotate_with_center ((cx-dx),(cy-d)) (cx, cy) (-.roll) in
	let (bx, by) = int_rotate_with_center ((cx+dx),(cy-d)) (cx, cy) (-.roll) in
	  dr#set_foreground color;
	  (*	dr#line (cx-dx) (cy-d) (cx+dx) (cy-d)*)
	  dr#line ax ay bx by;
      in
	
      let draw_small_grad = fun color cx cy roll d -> 
	draw_grad_with_l color cx cy roll d 10 in
	
      let draw_big_grad = fun color cx cy roll d ->
	draw_grad_with_l color cx cy roll d 40 in
	
	dr#set_background (`NAME "gray");
	
	let radius = min width height / 3 in
	let cx = width / 2 
	and cy = height/2 + pitch_scale pitch radius in
	  
	let center_x = width /2
	and center_y = height /2  in

	let top_clip = fun cx cy radius ->
	  let point_list = ref [] in
	    for i=(-180) to 0 do 
	      let x = float cx +. float radius *. cos ((float i)*.pi/. 180.) in
	      let y = float cy +. float radius *. sin ((float i)*.pi/. 180.) in
		point_list := (int_of_float x, int_of_float y)::!point_list;
	    done;
	    dr#set_foreground (`NAME "gray");
	    point_list := (width, cy)::!point_list;
	    point_list := (width, 0)::!point_list;
	    point_list := (0, 0)::!point_list;
	    point_list := (0, cy)::!point_list;
	    (*for i = 0 to List.length !point_list -2 do
	      let ax, ay = (List.nth !point_list i)
	      in
	      let bx, by = (List.nth !point_list i)
	      in
		dr#line ax ay bx by;
	    done*)
	    dr#polygon ~filled: true !point_list
	in


	let bottom_clip = fun cx cy radius ->
	  let point_list = ref [] in 
	    for i=(-180) to 0 do 
	      let x = float cx +. float radius *. cos ((float i)*.pi/. 180.) in
	      let y = float cy +. float cy -. (float cy +. float radius *. sin ((float i)*.pi/. 180.)) in
		point_list := (int_of_float x, int_of_float y)::!point_list;
	    done;
	    dr#set_foreground (`NAME "gray");
	    point_list := (width, cy)::!point_list;
	    point_list := (width, height)::!point_list;
	    point_list := (0, height)::!point_list;
	    point_list := (0, cy)::!point_list;
	    (*for i = 0 to List.length !point_list -2 do
	      let ax, ay = (List.nth !point_list i)
	      in
	      let bx, by = (List.nth !point_list i)
	      in
		dr#line ax ay bx by;
	    done*)
	    dr#polygon ~filled: true !point_list
	in
	  
	let top_arc = fun cx cy radius start stop -> 
	  let point_list = ref [] in 
	    for i=(start*10) to (stop*10) do 
	      let x = float cx +. float radius *. cos ((float i)*.pi/. 1800.) in
	      let y = float cy +. float cy -. (float cy +. float radius *. sin ((float i)*.pi/. 1800.)) in
		point_list := (int_of_float x, int_of_float y)::!point_list;
	    done;
	    dr#set_foreground `WHITE;
	    for i = 0 to List.length !point_list -2 do
	      let ax, ay = (List.nth !point_list i)
	      in
	      let bx, by = (List.nth !point_list i)
	      in
		dr#line ax ay bx by;
	    done in
	    

	let rx1 = center_x-radius in
	let rx2 = center_x+radius-(width/20) in
	let ry= center_y-2 in

	draw_arc ~color:(`NAME "blue") ~cx ~cy (2*radius) roll (roll+.pi);
	draw_arc ~color:(`NAME "slategray") ~cx ~cy (2*radius) (roll+.pi) (roll+.2.*.pi);

	let clip_horizon = fun () ->
	  (*dr#set_foreground `BLACK;
	  dr#rectangle
	    ~x: 0 ~y: 0 ~width: rx1 ~height: width ~filled: true ();
	  dr#rectangle
	    ~x: (cx+radius) ~y: 0 ~width: rx1 ~height: width ~filled: true ();
	  dr#rectangle
	    ~x: 0 ~y: 0 ~width: width ~height: (cy-radius) ~filled: true ();
	  dr#rectangle
	    ~x: 0 ~y: (cy+radius) ~width: width ~height: (cy-radius) ~filled: true ();*)
	  top_clip center_x center_y radius;
	  bottom_clip center_x center_y radius
	in

	let graduate = fun () ->
	  for i=(-16) to 16 do
	    dr#set_foreground `WHITE;
	    draw_small_grad `WHITE cx cy roll (3*i*radius/32);
	  done;
	  for i=(-4) to 4 do
	    dr#set_foreground `WHITE;
	    draw_big_grad `WHITE cx cy roll (12*i*radius/32);
	  done 
	in

	  graduate ();
	  clip_horizon ();
	  top_arc center_x center_y radius 60 120;
	  top_arc center_x center_y (radius+1) 70 110;

	  dr#set_foreground `WHITE;
	dr#rectangle ~x:rx1 ~y:(ry-1) ~width: (width/20) ~height:6 ~filled: true ();
	dr#set_foreground `BLACK;
	dr#rectangle ~x:rx1 ~y:ry ~width: (width/20) ~height:4 ~filled: true ();

	dr#set_foreground `WHITE;
	dr#rectangle ~x:rx2 ~y:(ry-1) ~width: (width/20) ~height:6 ~filled: true ();
	dr#rectangle ~x:(center_x-3) ~y:(center_y-3) ~width: 6 ~height:6 ~filled: true ();

	dr#set_foreground `BLACK;

	dr#rectangle ~x:rx2 ~y:ry ~width: (width/20) ~height:4 ~filled: true ();
	dr#rectangle ~x:(center_x-2) ~y:(center_y-2) ~width: 4 ~height:4 ~filled: true ();
	dr#set_foreground `WHITE;
	dr#arc ~x: center_x ~y: center_y ~width: (width) ~height: (width) ~filled: true ~start: (pi/. 3.) ~angle: (pi/. 3.) ();

	
	(new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap
  end
