(*
 * 2D Geometry
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec
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

(* Distance from a point to a line, a segment, a set of segments *)
(* Projection of a point on a line, a segment, a set of segments *)

(* Local Modules *)

let epsilon = 0.0001

(* Type containing a 2D point *)
type pt_2D = {x2D : float; y2D : float}

(* Zero vector 2D *)
let null_vector = {x2D=0.; y2D=0.}

(* Polygon 2D, not-closed by default *)
type poly_2D = pt_2D list

(* Types of intersections :                                                     *)
(* T_IN_SEGx     : intersection point x in the segment (exluding ends)          *)
(* T_ON_PTx      : intersection at point x                                      *)
(* T_OUT_SEG_PTx : intersection out of a segment. The point of intersection is  *)
(*                 located on the side of the point x                           *)
type t_crossing = T_IN_SEG1 | T_IN_SEG2 | T_ON_PT1 | T_ON_PT2 | T_ON_PT3
                  | T_ON_PT4 | T_OUT_SEG_PT1 | T_OUT_SEG_PT2 | T_OUT_SEG_PT3 | T_OUT_SEG_PT4

(* Polygon type : convex, concave or undefined *)
type t_conv = CONVEX | CONCAVE | CONV_UNDEFINED

(* Direction of a polygon: clockwise, counterclockwise and undefined *)
type t_ccw  = CW | CCW | CCW_UNDEFINED

(* Square function *)
let cc x = x*.x

(* Type of points used for triangulation *)
type vertex = {pos : pt_2D;
               num : int ;
               mutable prev : int ;
               mutable next : int ;
               mutable ear  : bool}

(* ============================================================================= *)
(* = Manipulations with angles                                                 = *)
(* ============================================================================= *)
let m_pi = 3.1415926535897932384626433832795
let deg2rad d = (d*.m_pi)/.180.
let rad2deg d = (d*.180.)/.m_pi

(* ============================================================================= *)
(* = Comparison of points / vectors                                            = *)
(* ============================================================================= *)
let point_same pt1 pt2 = (pt1.x2D = pt2.x2D) && (pt1.y2D = pt2.y2D)

(* ============================================================================= *)
(* = Creation of a vector                                                      = *)
(* ============================================================================= *)
let vect_make pt1 pt2 = {x2D=pt2.x2D -. pt1.x2D; y2D=pt2.y2D -. pt1.y2D}

(* ============================================================================= *)
(* = Vector norm                                                               = *)
(* ============================================================================= *)
let vect_norm v = sqrt((cc v.x2D) +. (cc v.y2D))

(* ============================================================================= *)
(* = Vector normalization                                                      = *)
(* ============================================================================= *)
let vect_normalize v = let n = vect_norm v in {x2D=v.x2D/.n; y2D=v.y2D/.n}

(* ============================================================================= *)
(* = Force norm of a vector                                                    = *)
(* ============================================================================= *)
let vect_set_norm v norme =
  let n = norme /. (vect_norm v) in {x2D=v.x2D*.n; y2D=v.y2D*.n}

(* ============================================================================= *)
(* = Distance between two points                                               = *)
(* ============================================================================= *)
let distance pt1 pt2 = vect_norm (vect_make pt1 pt2)

(* ============================================================================= *)
(* = Rotation of a vector by an angle alpha in radians                         = *)
(* ============================================================================= *)
let vect_rotate_rad v alpha =
  let c = cos alpha and s = sin alpha in
  {x2D= v.x2D *. c -. v.y2D *. s; y2D= v.x2D *. s +. v.y2D *. c}

(* ============================================================================= *)
(* = Rotation of a vector by an angle alpha in degrees                         = *)
(* ============================================================================= *)
let vect_rotate v alpha = vect_rotate_rad v (deg2rad alpha)

(* ============================================================================= *)
(* = Creation of a normal vector to vector v (rotated 90 degrees positive)     = *)
(* ============================================================================= *)
let vect_rotate_90 v = {x2D= -.v.y2D; y2D=v.x2D}

(* ============================================================================= *)
(* = Adds two vectors (or a point and a vector)				                   = *)
(* ============================================================================= *)
let vect_add u v = {x2D=u.x2D+.v.x2D; y2D=u.y2D+.v.y2D}

(* ============================================================================= *)
(* = Subtract two vectors (or a point and a vector)                            = *)
(* ============================================================================= *)
let vect_sub u v = {x2D=u.x2D-.v.x2D; y2D=u.y2D-.v.y2D}

(* ============================================================================= *)
(* = Multiplication of a vector by a scalar (float)                            = *)
(* ============================================================================= *)
let vect_mul_scal v m = {x2D=m*.v.x2D; y2D=m*.v.y2D}

(* ============================================================================= *)
(* = Operation B=lamba.v+A                                                     = *)
(* ============================================================================= *)
let vect_add_mul_scal lambda a v = vect_add a (vect_mul_scal v lambda)

(* ============================================================================= *)
(* = Vector inversion                                                          = *)
(* ============================================================================= *)
let vect_inverse v = vect_mul_scal v (-1.)

(* ============================================================================= *)
(* = Middle of a segment                                                       = *)
(* ============================================================================= *)
let point_middle p1 p2 = {x2D=(p1.x2D+.p2.x2D)/.2.; y2D= (p1.y2D+.p2.y2D)/.2.}

(* ============================================================================= *)
(* = Centroid of a list of points with or without coefficients                 = *)
(* ============================================================================= *)
let barycenter lst_pts =
  let v = List.fold_left (fun p pt -> vect_add p pt) null_vector lst_pts in
  vect_mul_scal v (1.0/.(float_of_int (List.length lst_pts)))

let weighted_barycenter lst_pts lst_coeffs =
  let (v, somme_coeffs) =
    List.fold_left2 (fun (p, s) pt c -> (vect_add_mul_scal c p pt, s+.c))
      (null_vector, 0.0) lst_pts lst_coeffs in
  vect_mul_scal v (1.0/.somme_coeffs)

(* ============================================================================= *)
(* = Scalar product                                                            = *)
(* ============================================================================= *)
let dot_product u v = u.x2D*.v.x2D +. u.y2D*.v.y2D

(* ============================================================================= *)
(* = Cross product (vector product)                                            = *)
(* ============================================================================= *)
let cross_product u v = u.x2D*.v.y2D -. u.y2D*.v.x2D


(* ============================================================================= *)
(* =                                                                           = *)
(* =                             Projections                                   = *)
(* =                                                                           = *)
(* ============================================================================= *)

(* ============================================================================= *)
(* = Projecting a point pt on a line (a, u)                                    = *)
(* ============================================================================= *)
let point_project_on_line pt a u =
  let v = vect_make a pt in
  let n = vect_norm u in
  let lambda = ((dot_product u v)/.(cc n)) in
  vect_add_mul_scal lambda a u

(* ============================================================================= *)
(* = Projection of a point on a segment, if possible                           = *)
(* ============================================================================= *)
let point_project_on_segment pt a b =
  let v = vect_make a pt and u = vect_make a b in
  let n = vect_norm u in
  let lambda = ((dot_product u v)/.(cc n)) in
  if lambda>=0. && lambda <=1. then Some (vect_add_mul_scal lambda a u)
  else None

(* ============================================================================= *)
(* = Projection of a point on a set of segments, if possible                   = *)
(* ============================================================================= *)
let point_project_on_segments_list pt lst_points =
  let proj = ref None and dist = ref 0. in
  let rec f l =
    match l with
        a::b::reste ->
          (match point_project_on_segment pt a b with
              None   -> ()
            | Some p ->
              (* The point projects on the segment ab, we test if the distance  *)
              (* of pt to the segment ab is less than the current distance, if  *)
              (* yes then p is the point we are searching.                      *)
              let d = distance p pt in
              (match !proj with
                  None   -> dist:= d; proj:=Some p
                | Some _ -> if d < !dist then begin dist:= d; proj:=Some p end
              )) ;
          f (b::reste)
      | _ -> !proj
  in
  f lst_points


(* ============================================================================= *)
(* =                                                                           = *)
(* =                              Distances                                    = *)
(* =                                                                           = *)
(* ============================================================================= *)


(* ============================================================================= *)
(* = Distance of a point from a line                                           = *)
(* ============================================================================= *)
let distance_point_line pt a u =
  let v = vect_make pt a in abs_float ((cross_product u v)/.(vect_norm u))

(* ============================================================================= *)
(* = Distance from a point to a set of segments                                = *)
(* ============================================================================= *)
let distance_point_segments_list pt lst_points =
  match point_project_on_segments_list pt lst_points with
      None   -> None
    | Some p -> Some (distance p pt)


(* ============================================================================= *)
(* =                                                                           = *)
(* =                             Intersections                                 = *)
(* =                                                                           = *)
(* ============================================================================= *)


(* ============================================================================= *)
(* = Intersection routine                                                      = *)
(* = a = point + u = vector definting the first line                           = *)
(* = b = point + v = vector defining the second line                           = *)
(* ============================================================================= *)
let crossing_point a u c v =
  let x = vect_make c a in
  let num1 = cross_product x v and num2 = cross_product x u
  and denom = -.(cross_product u v) in

  if denom = 0. then
    (* The two vectors are parallel *)
    None
  else begin
    let r = num1 /. denom and s = num2 /. denom in
    let type_intersection_seg1 =
      if abs_float r < epsilon then T_ON_PT1
      else if abs_float (r-.1.0) < epsilon then T_ON_PT2
      else if r<0.0 then T_OUT_SEG_PT1
      else if r>1.0 then T_OUT_SEG_PT2
      else T_IN_SEG1

    and type_intersection_seg2 =
      if abs_float s < epsilon then T_ON_PT3
      else if abs_float (s-.1.0) < epsilon then T_ON_PT4
      else if s<0.0 then T_OUT_SEG_PT3
      else if s>1.0 then T_OUT_SEG_PT4
      else T_IN_SEG2

    and pt_intersection = vect_add_mul_scal r a u in

    Some (type_intersection_seg1, type_intersection_seg2, pt_intersection)
  end

(* ============================================================================= *)
(* = Test the type of interscetion                                             = *)
(* ============================================================================= *)
let test_in_segment t =
  (t=T_IN_SEG1)||(t=T_ON_PT1)||(t=T_ON_PT2)||
    (t=T_IN_SEG2)||(t=T_ON_PT3)||(t=T_ON_PT4)
let test_on_hl t = (test_in_segment t)||(t=T_OUT_SEG_PT4)||(t=T_OUT_SEG_PT2)

(* ============================================================================= *)
(* = Tests the intersection of two segments (a,b) and (c,d)                    = *)
(* ============================================================================= *)
let crossing_seg_seg a b c d =
  match crossing_point a (vect_make a b) c (vect_make c d) with
      None -> false
    | Some (type1, type2, _pt) -> (test_in_segment type1) && (test_in_segment type2)

(* ============================================================================= *)
(* = Tests the intersection of a segment (a,b) and a half-line (c,v)           = *)
(* ============================================================================= *)
let crossing_seg_hl a b c v =
  match crossing_point a (vect_make a b) c v with
      None -> false
    | Some (type1, type2, _pt) ->
      (* OK si intersection sur la demi-droite *)
      (test_in_segment type1) && (test_on_hl type2)

(* ============================================================================= *)
(* = Tests the intersection of two half-lines                                  = *)
(* ============================================================================= *)
let crossing_hl_hl a u c v =
  let inter = crossing_point a u c v in
  match inter with
      None -> false
    | Some (type1, type2, _pt) -> (test_on_hl type1) && (test_on_hl type2)

(* ============================================================================= *)
(* = Tests the intersection of two lines and returns the point if it exists    = *)
(* ============================================================================= *)
let crossing_lines a u c v =
  match crossing_point a u c v with
      None -> (false, null_vector)
    | Some (_type1, _type2, pt) -> (true, pt)


(* ============================================================================= *)
(* =                                                                           = *)
(* =                              Polygons                                     = *)
(* =                by default they are considered open                        = *)
(* =                                                                           = *)
(* ============================================================================= *)


(* ============================================================================= *)
(* = Test if a polygon is closed                                               = *)
(* ============================================================================= *)
let poly_is_closed poly =
  if poly=[] then false else point_same (List.hd poly) (List.hd (List.rev poly))

(* ============================================================================= *)
(* = Close a polygon [A; B; C; D] -> [A; B; C; D; A]                           = *)
(* ============================================================================= *)
let poly_close poly =
  if poly = [] || poly_is_closed poly then poly else poly@[List.hd poly]

(* ============================================================================= *)
(* = Close a polygon [A; B; C; D] -> [|A; B; C; D; A; B|]                      = *)
(* ============================================================================= *)
let poly_close2 poly =
  if List.length poly < 2 then Array.of_list poly else begin
    let poly = if poly_is_closed poly then poly else poly@[List.hd poly] in
    Array.of_list (poly@[List.hd (List.tl poly)])
  end

(* ============================================================================= *)
(* = Indicates whether a point is in a polygon                                 = *)
(* ============================================================================= *)
let point_in_poly pt poly =
  let p = Array.of_list poly in

  let do_func {x2D=xi; y2D=yi} {x2D=xj; y2D=yj} {x2D=x; y2D=y} c =
    if (((yi<=y) && (y<yj)) || ((yj<=y) && (y<yi))) &&
      (x<((xj-.xi)*.(y-.yi)/.(yj-.yi)+.xi)) then
      c := not !c in

  let is_in = ref false and j = ref (List.length poly -1) in
  Array.iteri (fun i p0 -> do_func p0 p.(!j) pt is_in ; j := i) p ;

  (* Result *)
  !is_in

(* ============================================================================= *)
(* = Indicates whether a point is in a circle                                  = *)
(* ============================================================================= *)
let point_in_circle pt (center, r) = distance pt center <= r

(* ============================================================================= *)
(* = Calculation of the convex hull of a polygon                               = *)
(* ============================================================================= *)
let convex_hull poly =
  let det a b =
    match cross_product a b with 0.0 -> 0.0 | n when n>0.0 -> 1.0 | _ -> -1.0
  in

  let du_meme_cote a b c d =
    let u = vect_make a b and v = vect_make a c and w = vect_make a d in
    (det u v)*.(det u w)>0.0
  in

  let plus_proche a b c d =
    (d=a) || ((not(c=a)) &&
                 ((du_meme_cote b c d a) || (
                   let u = vect_make b c and v = vect_make b d in
                   (det u v)=0.0 && (abs_float(u.x2D)+.abs_float(u.y2D)>
                                      abs_float(v.x2D)+.abs_float(v.y2D)))))
  in

  let extract_mini p l =
    let rec aux reste vu mini =
      match reste with
          t::q ->
            if (try(p mini t) with _ -> false) then aux q (t::vu) mini
            else aux q (mini::vu) t
        | [] -> mini,vu
    in match l with
        t::q -> aux q [] t
      | [] -> raise Exit
  in

  let p a b = a.x2D<b.x2D || (a.x2D=b.x2D && a.y2D>b.y2D) in
  let l2=poly in
  let debut,_=extract_mini p l2 in
  let rec itere a o  liste sol =
    let p = plus_proche a o in
    let u,v=extract_mini p liste in
    if (u=debut) then (List.rev((u::sol))) else (itere o u v  (u::sol))
  in
  itere {x2D=debut.x2D+.1.0;y2D=debut.y2D} debut l2 [debut]

(* ============================================================================= *)
(* = Intersection of a segment and a polygon (not closed)                      = *)
(* ============================================================================= *)
let crossing_seg_poly a b poly =
  (* Remove duplicates in a sorted list *)
  let supprime_doublons_points l =
    let (_p, new_l) = List.fold_left (fun (old, lst) pt ->
      match old with
          None ->   (Some pt, [pt])
        | Some p -> if point_same p pt then (old, lst) else (Some pt, pt :: lst)
    ) (None, []) l in
    List.rev new_l
  in

  let u = vect_make a b and pol = Array.of_list poly
  and lst_pts_inter = ref [] in

  for i = 0 to (Array.length pol-1) do
    let c = pol.(i) and
        (* Reminder: the polygon is not closed... *)
        d = if i < (Array.length pol) -1 then pol.(i+1) else pol.(0) in
    let inter = crossing_point a u c (vect_make c d) in
    match inter with
        None -> () (* No intersection between the segment and the vertice *)
      | Some (type1, type2, pt) ->
        if (test_in_segment type1) && (test_in_segment type2) then
          (* The intersection is on the 2 segments *)
          lst_pts_inter := pt :: !lst_pts_inter
  done ;

  (* Remove duplicates in the list of intersections.                      *)
  (* There is a duplicate if the intersection is on a edge of the polygon *)
  supprime_doublons_points !lst_pts_inter

(* ============================================================================= *)
(* = Intersection without considering the vertices                             = *)
(* ============================================================================= *)
let crossing_seg_poly_exclusive a b poly =
  let u = vect_make a b and
      pol = Array.of_list poly and
      lst_pts_inter = ref [] in

  for i = 0 to (Array.length pol-1) do
    let c = pol.(i) and
        (* Reminder: the polygon is not closed... *)
        d = if i < (Array.length pol) -1 then pol.(i+1) else pol.(0) in
    let inter = crossing_point a u c (vect_make c d) in
    match inter with
        None -> () (* No intersection between the segment and the vertice *)
      | Some (type1, type2, pt) ->
        if (type1=T_IN_SEG1) && (type2=T_IN_SEG2) then
          (* The intersection is on the 2 segments *)
          lst_pts_inter := pt :: !lst_pts_inter ;
  done ;

  !lst_pts_inter

(* ============================================================================= *)
(* = Circumscribed circle in a triangle                                        = *)
(* ============================================================================= *)
let circumcircle {x2D=x1; y2D=y1} {x2D=x2; y2D=y2} {x2D=x3; y2D=y3} =
  (* 3x3 matrix determinant *)
  let eval_det a1 a2 a3 b1 b2 b3 c1 c2 c3 =
    a1*.b2*.c3-.a1*.b3*.c2-.a2*.b1*.c3+.a2*.b3*.c1+.a3*.b1*.c2-.a3*.b2*.c1 in
  let eval_det1 a1 a2 b1 b2 c1 c2 = eval_det a1 a2 1. b1 b2 1. c1 c2 1. in

  let a = eval_det1 x1 y1 x2 y2 x3 y3 in
  let s1 = (cc x1)+.(cc y1) and s2 = (cc x2)+.(cc y2) and s3 = (cc x3)+.(cc y3) in
  let bx = eval_det1 s1 y1 s2 y2 s3 y3 in
  let by = -.(eval_det1 s1 x1 s2 x2 s3 x3) in
  let c = -.(eval_det s1 x1 y1 s2 x2 y2 s3 x3 y3) in
  let a = 2.*.a in
  let xc = bx/.a and yc = by/.a in
  let r = abs_float ((sqrt(bx*.bx+.by*.by-.2.*.a*.c))/.a) in

  (* Position of the centre and the radius *)
  ({x2D=xc; y2D=yc}, r)

(* ============================================================================= *)
(* = Test direction of an angle                                                = *)
(* ============================================================================= *)
let ccw_angle p0 p1 p2 =
  let p = cross_product (vect_make p0 p1) (vect_make p1 p2) in
  if p > 0. then CCW else if p < 0. then CW else CCW_UNDEFINED

(* ============================================================================= *)
(* = Test if a polygon is concave or convex                                    = *)
(* = Is convex if all consecutive angles are in the same direction             = *)
(* ============================================================================= *)
let poly_test_convex l =
  if List.length l > 2 then begin
    (* l = [A; B; C; D] -> t = [|A; B; C; D; A; B|] *)
    let t = poly_close2 l in
    let n = Array.length t in
    let sign = ccw_angle t.(0) t.(1) t.(2) and i = ref 1 in
    while !i<n-2 && ccw_angle t.(!i) t.(!i+1) t.(!i+2) = sign do incr i done ;
    if !i<n-2 then CONCAVE else CONVEX
  end else CONV_UNDEFINED

(* ============================================================================= *)
(* = Test if a polygon is ordered in a clockwise direction or not              = *)
(* ============================================================================= *)
let poly_test_ccw l =
  if List.length l > 2 then begin
    let t = Array.of_list (poly_close l) in
    if poly_test_convex l = CONVEX then ccw_angle t.(0) t.(1) t.(2)
    else begin
      let s = ref 0. in
      for i = 0 to (Array.length t-2) do
        s:= !s+.cross_product t.(i) t.(i+1)
      done ;
      if !s>0. then CCW else if !s<0. then CW else CCW_UNDEFINED
    end
  end else CCW_UNDEFINED

(* ============================================================================= *)
(* = Area of a polygon (signed)                                                = *)
(* ============================================================================= *)
let poly_signed_area poly =
  (* This can be done with vector products but the following way is *)
  (* more efficient and precise *)

  if List.length poly < 2 then 0. else begin
    let poly = poly_close2 poly in
    let n = Array.length poly -2 and area = ref 0. in
    for i = 1 to n do
      area:=!area+.poly.(i).x2D*.(poly.(i+1).y2D-.poly.(i-1).y2D)
    done ;
    !area/.2.
  end

(* ============================================================================= *)
(* = Area of a polygon (not signed)                                            = *)
(* ============================================================================= *)
let poly_area poly = abs_float (poly_signed_area poly)

(* ============================================================================= *)
(* = Centroid of a polygon                                                     = *)
(* ============================================================================= *)
let poly_centroid poly =
  (* We could triangulate and weight the center of each triangle by its  *)
  (* surface, but it can be more efficient. Here, we take a point of the *)
  (* polygon (the first for instance) and we weight the (signed) area of *)
  (* the triangles built from this point.                                *)

  (* Centroid of a triangle *)
  let centroid_triangle p1 p2 p3 =
    {x2D=(p1.x2D+.p2.x2D+.p3.x2D)/.3.; y2D=(p1.y2D+.p2.y2D+.p3.y2D)/.3.} in

  (* Signed area of a triangle, no need for poly_area... *)
  let area_triangle p1 p2 p3 =
    (cross_product (vect_make p1 p2) (vect_make p1 p3))/.2.
  in

  let rec f p0 l centroid =
    match l with
        p1::p2::reste ->
          let new_centroid = vect_add_mul_scal (area_triangle p0 p1 p2) centroid
            (centroid_triangle p0 p1 p2) in
          f p0 (p2::reste) new_centroid
      | _ ->
        let area = poly_signed_area poly in
        vect_mul_scal centroid (1./.area)
  in

  match poly with
      []         -> null_vector
    | p::[]      -> p
    | p1::p2::[] -> point_middle p1 p2
    | _          -> f (List.hd poly) (List.tl poly) null_vector

(* ============================================================================= *)
(* =                                                                           = *)
(* =                        Triangulation of polygons                          = *)
(* =                Default they are considered open                           = *)
(* =                                                                           = *)
(* ============================================================================= *)


(* ============================================================================= *)
(* = Triangulation of a polygon                                                = *)
(* ============================================================================= *)
let in_tesselation poly =
  (* It tests whether the polygon is CCW, if tt is not then it is reverse *)
  let (switched, l)=
    match poly_test_ccw poly with
        CW -> (true, List.rev poly)
      | _  -> (false, poly)
  in

  (* Creation of the points table in the necessary form for a triangulation *)
  let vertices =
    let t = Array.of_list l and n = List.length l and vertices = ref [] in
    Array.iteri (fun i pt ->
      vertices:={pos=pt; num=i; ear=false;
                 prev=if i=0 then n-1 else i-1;
                 next=if i=n-1 then 0 else i+1}::!vertices) t ;
    Array.of_list (List.rev !vertices)
  in

  (* Function testing whether points of indices n1 and n2 form a diagonal *)
  (* completely contained within the polygon *)
  let is_diagonal n1 n2 =
    let lefton a b c = cross_product (vect_make a b) (vect_make a c) >= 0. in
    let left a b c   = cross_product (vect_make a b) (vect_make a c) > 0. in

    let is_in_cone a b =
      let a1=vertices.(a.next) and a0=vertices.(a.prev) in

      (* Is A a convex point?  *)
      if lefton a.pos a1.pos a0.pos then
        (left a.pos b.pos a0.pos) && (left b.pos a.pos a1.pos)
      else not ((lefton a.pos b.pos a1.pos) && (lefton b.pos a.pos a0.pos))
    in

    let a = vertices.(n1) and b = vertices.(n2) in

    (* AAA  if is_in_cone a b && is_in_cone b a then begin *)
    if is_in_cone a b || is_in_cone b a then begin
      let rec f l =
        match l with
            c::reste ->
              let c1 = vertices.(c.next) in
              if c.num<>a.num && c1.num<>a.num && c.num<>b.num && c1.num<>b.num &&
                crossing_seg_seg a.pos b.pos c.pos c1.pos then false
              else f reste
          | [] -> true
      in
      f (Array.to_list vertices)
    end else false
  in

  (* Initialize ears *)
  Array.iter (fun v1 -> v1.ear <- is_diagonal v1.prev v1.next) vertices ;

  (* Triangulation *)
  let current_idx = ref 0 and earfound = ref false and lst_triangles = ref []
  and n = ref (Array.length vertices) in
  while !n>=3 do
    earfound:=false ;
    let v2 = ref !current_idx and finished = ref false in
    while not !finished && not !earfound do
      if vertices.(!v2).ear then begin
        (* The current point has an ear, we will delete it *)
        earfound:=true ;

        (* 5 consecutive points, v2 is the 'middle' of 5 *)
        let v1=vertices.(!v2).prev and v3=vertices.(!v2).next in
        let v0=vertices.(v1).prev  and v4=vertices.(v3).next in

        (* Saving the triangle. Not in the form v1, v2, v3 otherwise *)
        (* it is not CCW and therefore outside normal is not correct *)
        lst_triangles := (v3, !v2, v1)::!lst_triangles ;

        (* Update ears *)
        vertices.(v1).ear <- is_diagonal v0 v3 ;
        vertices.(v3).ear <- is_diagonal v1 v4 ;

        (* Remove the point v2 *)
        vertices.(v1).next <- v3 ;
        vertices.(v3).prev <- v1 ;
        current_idx:=v3 ;

        (* One riangle less to search *)
        decr n
      end else v2:=vertices.(!v2).next ;

      (* It's over when it returns to the initial point *)
      finished:= !v2 = !current_idx
    done ;
  done ;

  if not !earfound then begin Printf.printf "No ear !\n"; flush stdout end ;

  (* If the polygon was departing CW, it has be reversed and the points'   *)
  (* number are then not in the same sense that the polygon passed         *)
  (* by the user. Numbers are put back in the original order as when the   *)
  (* triangulation function was called.                                    *)
  if switched then begin
    let n = List.length poly in
    lst_triangles:=List.map (fun (p1, p2, p3) -> (n-1-p1, n-1-p2, n-1-p3)
    ) !lst_triangles
  end ;

  !lst_triangles

(* ============================================================================= *)
(* = Triangulation of a polygon                                                = *)
(* ============================================================================= *)
let tesselation l =
  let t = Array.of_list l in
  List.map (fun (p1, p2, p3) -> [t.(p1); t.(p2); t.(p3)]) (in_tesselation l)

(* ============================================================================= *)
(* = Search fan triangles in a list of triangles                               = *)
(* ============================================================================= *)
let in_tesselation_fans l =
  let t = Array.of_list l in
  let l = in_tesselation l in
  let tt = Array.mapi (fun i _x -> (i, 0)) t in
  let add_val x = let (p, n) = tt.(x) in tt.(x) <- (p, n+1) in
  List.iter (fun (p1, p2, p3) ->
    add_val p1; add_val p2; add_val p3) l ;
  let lst = List.fast_sort (fun (_, n1) (_, n2) -> n2-n1) (Array.to_list tt) in

  let tt2 = Array.make (Array.length tt) (0, []) in
  let i = ref 0 in
  List.iter (fun (x, _) -> tt2.(x) <- (!i, []); incr i) lst ;
  List.iter (fun (p1, p2, p3) ->
    let (t1, l1) = tt2.(p1) and (t2, l2) = tt2.(p2) and (t3, l3) = tt2.(p3) in
    if t1<t2&&t1<t3 then tt2.(p1) <- (t1, (p2, p3)::l1)
    else if t2<t1&&t2<t3 then tt2.(p2) <- (t2, (p3, p1)::l2)
    else tt2.(p3) <- (t3, (p1, p2)::l3)) l ;

  let lst_fans = ref [] in
  Array.iteri (fun i (_, l) ->
    let l0 = ref [] in
    let add_element (a, b) =
      let rec f deb fin =
        match fin with
            [] -> (a, b, [a; b])::!l0
          | (c, d, lst)::reste ->
            if b=c then begin
            (* Insert before *)
              (List.rev ((a, d, a::lst)::deb))@reste
            end else if a=d then begin
            (* Insert after *)
              (List.rev ((c, b, lst@[b])::deb))@reste
            end else f ((c, d, lst)::deb) reste
      in
      l0:=f [] !l0
    in
    let merge_lists () =
      let rec in_merge (a, b, l1) ll0 ll =
        match ll with
            (c, d, l2)::reste ->
              if b=c then
                (true, ((a, d, l1@(List.tl l2))::ll0)@reste)
              else if d=a then
                (true, ((c, b, l2@(List.tl l1))::ll0)@reste)
              else in_merge (a, b, l1) ((c, d, l2)::ll0) reste
          | [] -> (false, ll0)
      in
      let rec f l ll =
        match l with
            l1::reste ->
              let (merged, newl) = in_merge l1 [] reste in
              if merged then f newl ll
              else f reste (l1::ll)
          | [] -> ll
      in
      l0:=f !l0 []
    in

    if l<>[] then begin
      List.iter (fun x -> add_element x; merge_lists ()) l ;
      List.iter (fun (_, _, l) ->
        lst_fans := (i::l)::!lst_fans) !l0
    end) tt2 ;

  (t, !lst_fans)

(* ============================================================================= *)
(* = Triangulation in triangles_fan                                            = *)
(* ============================================================================= *)
(* Performs triangulation of the polygon
   triangle_fan OpenGL. Output is a list containing lists of
   points. Each point lists contains either 3 points (triangle)
   or more than 3 points (for a triangle_fan)
*)

let tesselation_fans l =
  let (t, l) = in_tesselation_fans l in
  List.map (fun l -> List.map (fun x -> t.(x)) l) l






type pt_2D_polar = { r2D : float; theta2D : float; }

let cart2polar p = {r2D = vect_norm p; theta2D = atan2 p.y2D p.x2D}

let polar2cart p = {x2D = p.r2D *. cos p.theta2D; y2D = p.r2D *. sin p.theta2D}


(* Aviation related crap (e.g. angle/heading normalization) *)

let two_m_pi = 2. *. m_pi
let m_pi_two = m_pi /. 2.

let wind_dir_from_angle_rad rad =
  let w = ref (3. *. m_pi_two -. rad) in
  while !w > two_m_pi do
    w := !w -. two_m_pi done;
  !w

let heading_of_to_angle_rad angle =
  let a =  ref (5. *. m_pi_two -. angle) in
  while !a >= two_m_pi do a := !a -. two_m_pi done;
  !a

let norm_angle_rad a =
  let a = ref a in
  while !a < -. m_pi do a := !a +. two_m_pi done;
  while !a > m_pi do a := !a -. two_m_pi done;
  !a

let norm_heading_rad a =
  let a = ref a in
  while !a < 0. do a := !a +. two_m_pi done;
  while !a > two_m_pi do a := !a -. two_m_pi done;
  !a

let oposite_heading_rad rad =
  norm_heading_rad (rad +. m_pi)


let sign = fun f -> f /. abs_float f

let arc_segment = fun p0 p1 p2 radius ->
  (* C: center of the arc *)
  let p0p1 = vect_make p0 p1
  and p0p2 = vect_make p0 p2 in
  let u = vect_normalize p0p1 in
  let v = vect_rotate_90 u in
  let s = sign (cross_product p0p1 p0p2) in
  let c = vect_add p1 (vect_mul_scal v (s*.radius)) in

  (* F first point of the segment *)
  let d_c2 = distance c p2 in
  if radius > d_c2 then
    (** Arc is empty *)
    (c, p1, s)
  else
    let alpha_2cf = -. s *. acos (radius /. d_c2)
    and alpha_c2 = (cart2polar (vect_make c p2)).theta2D in
    let alpha_cf = alpha_c2 +. alpha_2cf in
    let f = vect_add c (polar2cart {theta2D=alpha_cf;r2D=radius}) in

    (c, f, s)


let arc = fun ?(nb_points=5) c r a1 a2 ->
  let a2 = if a2 < a1 then a2 +. 2. *. m_pi else a2 in
  let da = (a2 -. a1) /. float (nb_points-1) in
  Array.init nb_points
    (fun i ->
      let a = a1 +. float i *. da in
      vect_add c (polar2cart { r2D = r; theta2D = a }))


type slice = { top : float; left_side : float * float; right_side : float * float }

let slice_polygon = fun poly ->
  let n = Array.length poly in
  assert (n >= 3);
  let next i = (i+1) mod n
  and prev i = (i-1+n) mod n in

  let bottom = ref 0 in
  for i = 1 to n -1  do
    if poly.(i).y2D < poly.(!bottom).y2D then
      bottom := i
  done;
  let l = ref [] in

  let slope = fun i j ->
    (poly.(j).x2D -. poly.(i).x2D) /. (poly.(j).y2D -. poly.(i).y2D) in

  let i = ref 1 in
  let g = ref !bottom
  and d = ref !bottom in
  let alpha_g = ref 0.
  and alpha_d = ref 0.
  and last_y = ref (poly.(!bottom).y2D) in
  begin
    try
      while !i <= n do
        while poly.(!d).y2D = !last_y && !i <= n do
          let d' = prev !d in
          if (d' = !g && poly.(!d).y2D = poly.(!g).y2D) then raise Exit;
          alpha_d := slope !d d';
          d := d';
          incr i
        done;
        while poly.(!g).y2D = !last_y && !i <= n do
          let g' = next !g in
          alpha_g := slope !g g';
          g := g';
          incr i
        done;
        let yd = poly.(!d).y2D
        and yg = poly.(!g).y2D in
        let ym = min yd yg in
        let xd = poly.(!d).x2D -. !alpha_d *. (yd -. ym) in
        let xg = poly.(!g).x2D -. !alpha_g *. (yg -. ym) in
        l :=  {top = ym; left_side=(xg, !alpha_g); right_side=(xd, !alpha_d)} :: !l;
        last_y := ym
      done
    with Exit -> () (* Flat polygon *)
  end;
  let under = {top=poly.(!bottom).y2D; left_side= (max_float, 0.); right_side=(min_float, 0.)}
  and over = {top=max_float; left_side=(max_float, 0.); right_side= (min_float, 0.)} in
  Array.of_list (under :: List.rev (over :: !l));;



(* =============================== END ========================================= *)

