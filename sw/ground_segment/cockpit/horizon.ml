open Latlong

let affine_pos_and_angle xw yw angle =
  let cos_a = cos angle in
  let sin_a = sin angle in
  [| cos_a ; sin_a ; ~-. sin_a; cos_a; xw ; yw |]


let arc = fun n r start stop ->
  let s = (stop -. start) /. float (n-1) in
  Array.init n (fun i -> let a = start+.float i*.s in (r*.cos a, r*.sin a))

let floats_of_points = fun ps ->
  let n = Array.length ps in
  let a = Array.create (2*n) 0. in
  for i = 0 to n - 1 do
    let (x, y) = ps.(i) in
    a.(2*i)<-x;
    a.(2*i+1)<-y
  done;
  a

class h = fun ?packing size  ->
  let canvas = GnoCanvas.canvas ~aa:true ~width:size ~height:size ?packing () in
  let size = float size in
  let size2 = size /. 2. in

  let pitch_scale = fun pitch -> pitch *. size2 *. 2. in

  let disc = GnoCanvas.group canvas#root in
  let top = GnoCanvas.rect ~x1:(-.size2) ~y1:(-.size2*.5.) ~x2:size2 ~y2:0. ~fill_color:"#0099cb" disc
  and bottom = GnoCanvas.rect ~x1:(-.size2) ~y1:0. ~x2:size2 ~y2:(size2*.5.) ~fill_color:"#986701" disc
  and line = GnoCanvas.line ~props:[`WIDTH_PIXELS 4] ~points:[|-.size2;0.;size2;0.|] ~fill_color:"white" disc in
  let grads = fun ?(text=false) n s a b ->
    for i = 0 to n do
      let a = float i *. a in
      let y = pitch_scale ((Deg>>Rad)(a+.b)) in
      ignore (GnoCanvas.line ~points:[|-.s; y; s; y|] ~fill_color:"white" disc);
      ignore (GnoCanvas.line ~points:[|-.s; -.y; s; -.y|] ~fill_color:"white" disc);
      if text then
	let text = Printf.sprintf "-%d" (truncate a)
	and x = 2.*.s in
	ignore (GnoCanvas.text ~props:[`ANCHOR `CENTER; `FILL_COLOR "white"] ~text ~y ~x disc);
	ignore (GnoCanvas.text ~props:[`ANCHOR `CENTER; `FILL_COLOR "white"] ~text ~y ~x:(-.x) disc);
	let y = -. y
	and text = Printf.sprintf "-%d" (truncate a) in
	ignore (GnoCanvas.text ~props:[`ANCHOR `CENTER; `FILL_COLOR "white"] ~text ~y ~x disc);
	ignore (GnoCanvas.text ~props:[`ANCHOR `CENTER; `FILL_COLOR "white"] ~text ~y ~x:(-.x) disc);
    done in
  let _ = 
    grads 10 (size2/.10.) 5. 2.5;
    grads 5 (size2/.7.) 10. 5.;
    grads ~text:true 5 (size2/.5.) 10. 10. in

  let mask = GnoCanvas.group canvas#root in
  let center = GnoCanvas.ellipse ~x1:(-3.) ~y1:(-.3.) ~x2:3. ~y2:3. ~fill_color:"black" mask in
  let pi6 = pi/.6. in
  let n = 20 in
  let arc_above = arc n size2 pi6 (5.*.pi6) in
  let (x,y) = arc_above.(n-1) in
  let rest = [|(x, 0.);(size, 0.); (size, size); (-.size,size);(-.size,0.);(-.x,0.)|] in
  let points = floats_of_points (Array.append arc_above rest) in
  let _ = 
    ignore (GnoCanvas.polygon ~fill_color:"black" ~points mask);
    for i = 0 to Array.length points / 2 - 1 do
      points.(2*i+1) <- -. points.(2*i+1) 
    done;
    ignore (GnoCanvas.polygon ~fill_color:"black" ~points mask);
    let s = size2/. 5. in
    ignore (GnoCanvas.line  ~props:[`WIDTH_PIXELS 4] ~points:[|-.x;0.;-.x-.s;0.;-.x-.s;s|] ~fill_color:"black" mask);
    ignore (GnoCanvas.line  ~props:[`WIDTH_PIXELS 4] ~points:[|x;0.;x+.s;0.;x+.s;s|] ~fill_color:"black" mask) in
  
  object (self)
      method draw = fun roll pitch ->
	disc#affine_absolute (affine_pos_and_angle 0. (pitch_scale pitch) (-.roll))
  end
