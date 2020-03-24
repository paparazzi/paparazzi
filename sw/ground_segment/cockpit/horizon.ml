(*
 * Multi aircrafts map display and flight plan editor
 *
 * Copyright (C) 2004-2006 ENAC, Pascal Brisset, Antoine Drouin / 2011 Tobias Muench, Rolf Noellenburg
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
open Latlong

let affine_pos_and_angle xw yw angle =
  let cos_a = cos angle in
  let sin_a = sin angle in
  [| cos_a ; sin_a ; ~-. sin_a; cos_a; xw; yw |]


let affine_pos xw yw = affine_pos_and_angle xw yw 0.

let arc = fun n r start stop ->
  let s = (stop -. start) /. float (n-1) in
  Array.init n (fun i -> let a = start+.float i*.s in (r*.cos a, r*.sin a))

let floats_of_points = fun ps ->
  let n = Array.length ps in
  let a = Array.make (2*n) 0. in
  for i = 0 to n - 1 do
    let (x, y) = ps.(i) in
    a.(2*i)<-x;
    a.(2*i+1)<-y
  done;
  a

let ruler = fun ?(index_on_right=false) ~text_props ~max_value ~scale ~w ~index_width ~step ~h root ->
  let r = GnoCanvas.group root in
  let height = scale *. float max_value in

  (* Grey background *)
  let _ = GnoCanvas.rect ~x1:0. ~y1:(-.height) ~x2:w ~y2:height ~fill_color:"#808080" r in
  let props = (text_props@[`ANCHOR `EAST]) in

  (* One step drawer *)
  let tab = Array.make (max_value/step) false in
  let draw = fun i ->
    let i = i * step in
    let y = -. scale *. float i in
    let text = Printf.sprintf "%d" i in
    let _ = GnoCanvas.text ~text ~props ~y ~x:(w*.0.75) r in
    let _ = GnoCanvas.line ~points:[|w*.0.8;y;w;y|] ~fill_color:"white" r in
    let y = y -. float step /. 2. *. scale in
    let _ = GnoCanvas.line ~points:[|w*.0.8;y;w;y|] ~fill_color:"white" r in
    () in

  let lazy_drawer = fun v ->
    let v = truncate v / step in
    for i = max 0 (v - 5) to min (v + 5) (Array.length tab - 1) do (* FIXME *)
      if not tab.(i) then begin
        tab.(i) <- true;
        draw i
      end
    done in

  (** Yellow index *)
  let _ = GnoCanvas.line ~points:[|0.;0.;w;0.|] ~props:[`WIDTH_PIXELS 2] ~fill_color:"yellow" root in
  let s = index_width in
  let idx = GnoCanvas.polygon ~points:[|0.;0.;-.s;s/.2.;-.s;-.s/.2.|] ~fill_color:"yellow" root in
  if index_on_right then
    idx#affine_absolute (affine_pos_and_angle w 0. pi);

  (** Mask (bottom & top) *)
  let _ = GnoCanvas.rect ~x1:0. ~y1:(-.height) ~x2:w ~y2:(-.h) ~fill_color:"black" root in
  let _ = GnoCanvas.rect ~x1:0. ~y1:height ~x2:w ~y2:h ~fill_color:"black" root in
  r, lazy_drawer


class h = fun ?packing size  ->
  let canvas = GnoCanvas.canvas ~aa:true ~width:size ~height:size ?packing () in
  let _ =
    canvas#set_center_scroll_region false;
  in

  let size = float size in
  let size2 = size /. 2. in

  let left_margin = size2 /. 10. in
  let pitch_scale = fun pitch -> pitch *. size2 *. 2. in
  let speed_scale = size2 /. 10. in
  let alt_scale = size2 /. 50. in
  let speed_width = size2/.3. in
  let alt_width = size2/.2.25 in
  let index_width = size2 /. 10. in

  let xc = left_margin +. speed_width +. size2
  and yc = size2*.1.25 in

  let text_props = [`FONT "Sans 8"; `ANCHOR `CENTER; `FILL_COLOR "white"] in

  let disc = GnoCanvas.group canvas#root in
  let _top = GnoCanvas.rect ~x1:(-.size) ~y1:(-.size2*.5.) ~x2:size ~y2:0. ~fill_color:"#0099cb" disc
  and _bottom = GnoCanvas.rect ~x1:(-.size) ~y1:0. ~x2:size ~y2:(size2*.5.) ~fill_color:"#986701" disc
  and _line = GnoCanvas.line ~props:[`WIDTH_PIXELS 4] ~points:[|-.size;0.;size;0.|] ~fill_color:"white" disc
  and _ = GnoCanvas.line ~points:[|0.;-.size;0.;size|] ~fill_color:"white" disc
  in
  let grads = fun ?(text=false) n s a b ->
    for i = 0 to n do
      let deg = float i *. a +. b in
      let y = pitch_scale ((Deg>>Rad)deg) in
      ignore (GnoCanvas.line ~points:[|-.s; y; s; y|] ~fill_color:"white" disc);
      ignore (GnoCanvas.line ~points:[|-.s; -.y; s; -.y|] ~fill_color:"white" disc);
      if text then
        let text = Printf.sprintf "%d" (truncate deg)
        and x = 2.*.s in
        ignore (GnoCanvas.text ~props:text_props ~text ~y:(-.y) ~x disc);
        ignore (GnoCanvas.text ~props:text_props ~text ~y:(-.y) ~x:(-.x) disc);
        let text = "-"^text in
        ignore (GnoCanvas.text ~props:text_props ~text ~y ~x disc);
        ignore (GnoCanvas.text ~props:text_props ~text ~y ~x:(-.x) disc);
    done in

  let _ =
    grads 10 (size2/.10.) 5. 2.5;
    grads 5 (size2/.7.) 10. 5.;
    grads ~text:true 5 (size2/.5.) 10. 10. in

  let mask = GnoCanvas.group ~x:xc ~y:yc canvas#root in
  let _center = GnoCanvas.ellipse ~x1:(-3.) ~y1:(-.3.) ~x2:3. ~y2:3. ~fill_color:"black" mask in
  let pi6 = pi/.6. in
  let n = 20 in
  let arc_above = arc n size2 pi6 (5.*.pi6) in
  let (x, _y) = arc_above.(n-1) in
  let rest = [|(x, 0.);(10.*.size, 0.); (10.*.size, 10.*.size); (-.size, 10.*.size);(-.size,0.);(-.x,0.)|] in
  let points = floats_of_points (Array.append arc_above rest) in
  let _ =
    ignore (GnoCanvas.polygon ~fill_color:"black" ~points mask);
    for i = 0 to Array.length points / 2 - 1 do
      points.(2*i+1) <- -. points.(2*i+1)
    done;
    ignore (GnoCanvas.polygon ~fill_color:"black" ~points mask);
    let s = size2/. 5. in
    ignore (GnoCanvas.line  ~props:[`WIDTH_PIXELS 4] ~points:[|-.x;0.;-.x-.s;0.;-.x-.s;s|] ~fill_color:"black" mask);
    ignore (GnoCanvas.line  ~props:[`WIDTH_PIXELS 4] ~points:[|x;0.;x+.s;0.;x+.s;s|] ~fill_color:"black" mask);

  (* Top and bottom graduations *)
    let g = fun a ->
      let l = GnoCanvas.line~props:[`WIDTH_PIXELS 1]  ~fill_color:"white" ~points:[|0.;-.size2;0.;-.1.07*.size2|] mask in
      l#affine_relative (affine_pos_and_angle 0. 0. ((Deg>>Rad)a)) in
    for i = 1 to 5 do
      let a = float (i*10) in
      g a; g (-.a)
    done;

    let gg = fun a  ->
      let l = GnoCanvas.line~props:[`WIDTH_PIXELS 2]  ~fill_color:"white" ~points:[|0.;-.size2;0.;-.1.15*.size2|] mask in
      l#affine_relative (affine_pos_and_angle 0. 0. ((Deg>>Rad)a)) in
    gg 30.; gg (-30.);
    gg 0.; gg 0.;

    let _30 = fun a  ->
      let t = GnoCanvas.text ~text:"30" ~props:text_props ~x:0. ~y:(-1.28*.size2) mask in
      t#affine_relative (affine_pos_and_angle 0. 0. ((Deg>>Rad)a)) in
    _30  30.; _30 (-30.)
  in


  (* Speedometer on the left side *)
  let speed, mi, mx, lazy_speed =
    let g = GnoCanvas.group ~x:left_margin ~y:yc canvas#root in
    let r, lazy_ruler = ruler ~text_props ~index_on_right:true ~max_value:50 ~scale:speed_scale ~w:speed_width ~step:2 ~index_width ~h:(0.75*.size2) g in
    let mx =
      GnoCanvas.text ~x:(speed_width/.2.) ~y:(-0.88*.size2) ~props:text_props g
    and mi =
      GnoCanvas.text ~x:(speed_width/.2.) ~y:(0.875*.size2) ~props:text_props g in
    mx#set [`FILL_COLOR "yellow"];
    mi#set [`FILL_COLOR "yellow"];
    lazy_ruler 0.;
    r, mi, mx, lazy_ruler

  (* Altimeter on the right side *)
  and alt, lazy_alt =
    let g = GnoCanvas.group ~x:(xc+.size2) ~y:yc canvas#root in
    ruler ~text_props ~max_value:3000 ~scale:alt_scale ~w:alt_width ~step:10 ~index_width ~h:(0.75*.size2) g
  in

object
  method set_attitude = fun roll pitch ->
    disc#affine_absolute (affine_pos_and_angle (xc+.((sin roll)*.(pitch_scale pitch))) (yc+.pitch_scale pitch*.(cos roll)) (-.roll))
  val mutable max_speed = 0.
  val mutable min_speed = max_float
  method set_speed = fun (s:float) ->
    speed#affine_absolute (affine_pos 0. 0.);
    lazy_speed s;
    speed#affine_absolute (affine_pos 0. (speed_scale*.s));
    min_speed <- min min_speed s;
    max_speed <- max max_speed s;
    mi#set [`TEXT (sprintf "%.1f" min_speed)];
    mx#set [`TEXT (sprintf "%.1f" max_speed)]
  initializer
    ignore (speed#connect#event (function
    `BUTTON_PRESS _ev ->
      max_speed <- 0.; min_speed <- max_float; true
      | _ -> false))

  method set_alt = fun (s:float) ->
    alt#affine_absolute (affine_pos 0. 0.);
    lazy_alt s;
    alt#affine_absolute (affine_pos 0. (alt_scale*.s))

end

(*****************************************************************************)
(* pfd page                                                                  *)
(*****************************************************************************)
class pfd ?(visible = fun _ -> true) (widget: GBin.frame) =
  let horizon = new h ~packing: widget#add 150 in
  let _lazy = fun f x -> if visible widget then f x in

object
  method set_attitude roll pitch =
    _lazy (horizon#set_attitude  ((Deg>>Rad)roll)) ((Deg>>Rad)pitch)
  method set_alt (a:float) = _lazy horizon#set_alt a
  method set_climb (_c:float) = ()
  method set_speed (c:float) = _lazy horizon#set_speed c
end
