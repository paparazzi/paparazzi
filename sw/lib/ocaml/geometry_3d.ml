(*
 * 3D Geometry
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

open Geometry_2d

(* Type contenant un point 2D *)
type pt_3D = {x3D : float; y3D : float; z3D : float}

(* Polygone 3D, non ferme par defaut *)
type poly_3D = pt_3D list

(* Volume 3D *)
type volume_3D = poly_3D list

(* Vecteurs nuls en 2D *)
let null_vector = {x3D=0.; y3D=0.; z3D=0.}

(* Carre *)
let cc x = x*.x

let epsilon = 0.0001
type t_crossing3d = T_IN_SEG1 | T_IN_SEG2 | T_ON_PT1 | T_ON_PT2 | T_ON_PT3
                    | T_ON_PT4 | T_OUT_SEG_PT1 | T_OUT_SEG_PT2 | T_OUT_SEG_PT3 | T_OUT_SEG_PT4

(* Type pour les differents axes *)
type t_axis3d = T_X3D | T_Y3D | T_Z3D

(* ============================================================================= *)
(* = Determinant d'une matrice 3x3                                             = *)
(* ============================================================================= *)
let eval_det3 a1 a2 a3 b1 b2 b3 c1 c2 c3 =
  a1*.b2*.c3-.a1*.b3*.c2-.a2*.b1*.c3+.a2*.b3*.c1+.a3*.b1*.c2-.a3*.b2*.c1

(* ============================================================================= *)
(* = Determinant d'une matrice 4x4 (par developpement en matrice 3x3           = *)
(* ============================================================================= *)
let eval_det4 a1 a2 a3 a4 b1 b2 b3 b4 c1 c2 c3 c4 d1 d2 d3 d4 =
  let det1 = eval_det3 b2 b3 b4 c2 c3 c4 d2 d3 d4
  and det2 = eval_det3 b1 b3 b4 c1 c3 c4 d1 d3 d4
  and det3 = eval_det3 b1 b2 b4 c1 c2 c4 d1 d2 d4
  and det4 = eval_det3 b1 b2 b3 c1 c2 c3 d1 d2 d3 in

  a1*.det1-.a2*.det2+.a3*.det3-.a4*.det4

(* ============================================================================= *)
(* = Comparaison de points/vecteurs                                            = *)
(* ============================================================================= *)
let point_same pt1 pt2 =
  (pt1.x3D = pt2.x3D) && (pt1.y3D = pt2.y3D) && (pt1.z3D = pt2.z3D)

(* ============================================================================= *)
(* = Creation d'un vecteur                                                     = *)
(* ============================================================================= *)
let vect_make pt1 pt2 = {x3D=pt2.x3D -. pt1.x3D; y3D=pt2.y3D -. pt1.y3D;
                         z3D=pt2.z3D -. pt1.z3D}

(* ============================================================================= *)
(* = Norme d'un vecteur                                                        = *)
(* ============================================================================= *)
let vect_norm v = sqrt((cc v.x3D) +. (cc v.y3D) +. (cc v.z3D))

(* ============================================================================= *)
(* = Normalisation d'un vecteur                                                = *)
(* ============================================================================= *)
let vect_normalize v =
  let n = vect_norm v in {x3D=v.x3D/.n; y3D=v.y3D/.n; z3D=v.z3D/.n}

(* ============================================================================= *)
(* = Force la norme d'un vecteur                                               = *)
(* ============================================================================= *)
let vect_set_norm v norme =
  let n = norme /. (vect_norm v) in {x3D=v.x3D*.n; y3D=v.y3D*.n; z3D=v.z3D*.n}

(* ============================================================================= *)
(* = Distance entre deux points                                                = *)
(* ============================================================================= *)
let distance pt1 pt2 = vect_norm (vect_make pt1 pt2)

(* ============================================================================= *)
(* = Ajoute deux vecteurs (ou d'un point et d'un vecteur)                      = *)
(* ============================================================================= *)
let vect_add u v = {x3D=u.x3D+.v.x3D; y3D=u.y3D+.v.y3D; z3D=u.z3D+.v.z3D}

(* ============================================================================= *)
(* = Soustraction de deux vecteurs (ou d'un point et d'un vecteur)             = *)
(* ============================================================================= *)
let vect_sub u v = {x3D=u.x3D-.v.x3D; y3D=u.y3D-.v.y3D; z3D=u.z3D-.v.z3D}

(* ============================================================================= *)
(* = Multiplication d'un vecteur par un flottant                               = *)
(* ============================================================================= *)
let vect_mul_scal v m = {x3D=m*.v.x3D; y3D=m*.v.y3D; z3D=m*.v.z3D}

(* ============================================================================= *)
(* = Operation B=lamba.v+A                                                     = *)
(* ============================================================================= *)
let vect_add_mul_scal lambda a v = vect_add a (vect_mul_scal v lambda)

(* ============================================================================= *)
(* = Vecteur oppose                                                            = *)
(* ============================================================================= *)
let vect_inverse v = vect_mul_scal v (-1.)

(* ============================================================================= *)
(* = Milieu d'un segment                                                       = *)
(* ============================================================================= *)
let point_middle p1 p2 = {x3D=(p1.x3D+.p2.x3D)/.2.; y3D=(p1.y3D+.p2.y3D)/.2.;
                          z3D=(p1.z3D+.p2.z3D)/.2.}

(* ============================================================================= *)
(* = Barycentre d'une liste de points avec ou sans coefficients                = *)
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
(* = Produit scalaire                                                          = *)
(* ============================================================================= *)
let dot_product u v = u.x3D*.v.x3D +. u.y3D*.v.y3D +. u.z3D*.v.z3D

(* ============================================================================= *)
(* = Produit vectoriel                                                         = *)
(* ============================================================================= *)
let cross_product u v = {x3D=u.y3D*.v.z3D -. u.z3D*.v.y3D;
                         y3D=u.z3D*.v.x3D -. u.x3D*.v.z3D;
                         z3D=u.x3D*.v.y3D -. u.y3D*.v.x3D}

(* ============================================================================= *)
(* = Normale unitaire a un deux vecteurs                                       = *)
(* ============================================================================= *)
let normal u v = vect_normalize (cross_product u v)

(* ============================================================================= *)
(* = Test d'un point sur un segment                                            = *)
(* ============================================================================= *)
let point_on_segment p p1 p2 =
  (* P est sur le segment P1 P2 si le produit vectoriel P1P^P1P2 *)
  (* est nul (P sur la droite P1 P2) et que le produit scalaire  *)
  (* P1P.P1P2 est compris entre 0 et la norme de P1P2 au carre   *)
  let v = cross_product (vect_make p1 p) (vect_make p1 p2) in
  let scal = dot_product (vect_make p1 p) (vect_make p1 p2)
  and n = distance p1 p2 in
  v=null_vector && scal>=0. && scal<=cc n


(* ============================================================================= *)
(* =                                                                           = *)
(* =                             Intersections                                 = *)
(* =                                                                           = *)
(* ============================================================================= *)


(* ============================================================================= *)
(* = Intersection de deux droites 3D                                           = *)
(* ============================================================================= *)
let crossing_point a u c v =
  let w = vect_make a c in
  let n = cross_product u v in
  let n2 = vect_norm n in let n2 = cc n2 in

  (* Si s<>0 alors les vecteurs ne sont pas coplanaires *)
                          let s = dot_product n w in

  (* Si n est nul alors les vecteurs sont paralleles *)
                          if n2 = 0.0 or s <> 0. then None else begin
                            let r = (dot_product (cross_product w v) n)/.n2
                            and s = (dot_product (cross_product w u) n)/.n2 in
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
(* = Test du type d'intersection                                               = *)
(* ============================================================================= *)
let test_in_segment t =
  (t=T_IN_SEG1)||(t=T_ON_PT1)||(t=T_ON_PT2)||
    (t=T_IN_SEG2)||(t=T_ON_PT3)||(t=T_ON_PT4)
let test_on_hl t = (test_in_segment t)||(t=T_OUT_SEG_PT4)||(t=T_OUT_SEG_PT2)

(* ============================================================================= *)
(* = Teste l'intersection de deux segments (a,b) et (c,d)                      = *)
(* ============================================================================= *)
let crossing_seg_seg a b c d =
  match crossing_point a (vect_make a b) c (vect_make c d) with
      None -> false
    | Some (type1, type2, _pt) -> (test_in_segment type1)&&(test_in_segment type2)

(* ============================================================================= *)
(* = Teste l'intersection d'un segment (a,b) et d'une demi-droite (c,v)        = *)
(* ============================================================================= *)
let crossing_seg_hl a b c v =
  match crossing_point a (vect_make a b) c v with
      None -> false
    | Some (type1, type2, _pt) ->
      (* OK si intersection sur la demi-droite *)
      (test_in_segment type1) && (test_on_hl type2)

(* ============================================================================= *)
(* = Teste l'intersection de deux demi-droites                                 = *)
(* ============================================================================= *)
let crossing_hl_hl a u c v =
  let inter = crossing_point a u c v in
  match inter with
      None -> false
    | Some (type1, type2, _pt) -> (test_on_hl type1) && (test_on_hl type2)

(* ============================================================================= *)
(* = Teste l'intersection de deux droites et renvoie le point s'il existe      = *)
(* ============================================================================= *)
let crossing_lines a u c v =
  match crossing_point a u c v with
      None -> (false, null_vector)
    | Some (_type1, _type2, pt) -> (true, pt)

(* ============================================================================= *)
(* = Intersection d'une droite (a, u) et d'un plan (c, d, e)                   = *)
(* ============================================================================= *)
let crossing_line_plane a u c d e =
  let num = eval_det4
    1.    1.    1.    1.
    c.x3D d.x3D e.x3D a.x3D
    c.y3D d.y3D e.y3D a.y3D
    c.z3D d.z3D e.z3D a.z3D
  and denom = eval_det4
    1.    1.    1.    0.
    c.x3D d.x3D e.x3D u.x3D
    c.y3D d.y3D e.y3D u.y3D
    c.z3D d.z3D e.z3D u.z3D
  in
  if denom=0. then None
  else Some (vect_add_mul_scal (num/.denom) a u)

(* ============================================================================= *)
(* = Intersection d'une demi-droite (a, u) et d'un plan (c, d, e)              = *)
(* ============================================================================= *)
let crossing_hline_plane a u c d e =
  let num = eval_det4
    1.    1.    1.    1.
    c.x3D d.x3D e.x3D a.x3D
    c.y3D d.y3D e.y3D a.y3D
    c.z3D d.z3D e.z3D a.z3D
  and denom = eval_det4
    1.    1.    1.    0.
    c.x3D d.x3D e.x3D u.x3D
    c.y3D d.y3D e.y3D u.y3D
    c.z3D d.z3D e.z3D u.z3D
  in

  if denom=0. then None else begin
    let s = (-.num)/.denom in
    if s >= 0. then Some (vect_add_mul_scal s a u)
    else None
  end


(* ============================================================================= *)
(* =                                                                           = *)
(* =                               Polygones                                   = *)
(* =                                                                           = *)
(* =            Ils sont consideres comme etant ouverts et plans               = *)
(* =                                                                           = *)
(* ============================================================================= *)


(* ============================================================================= *)
(* = Teste si un polygone est ferme                                            = *)
(* ============================================================================= *)
let poly_is_closed poly =
  if poly=[] then false else point_same (List.hd poly) (List.hd (List.rev poly))

(* ============================================================================= *)
(* = Ferme un polygone [A; B; C; D] -> [A; B; C; D; A]                         = *)
(* ============================================================================= *)
let poly_close poly =
  if poly = [] or poly_is_closed poly then poly else poly@[List.hd poly]

(* ============================================================================= *)
(* = Ferme un polygone [A; B; C; D] -> [|A; B; C; D; A; B|]                    = *)
(* ============================================================================= *)
let poly_close2 poly =
  if List.length poly < 2 then Array.of_list poly else begin
    let poly = if poly_is_closed poly then poly else poly@[List.hd poly] in
    Array.of_list (poly@[List.hd (List.tl poly)])
  end

(* ============================================================================= *)
(* = Transformation d'un point 3D en 2D en supprimant la coordonnee indiquee   = *)
(* ============================================================================= *)
let pt_3d_to_pt_2d axis pt =
  match axis with
      T_X3D -> {x2D=pt.y3D; y2D=pt.z3D}
    | T_Y3D -> {x2D=pt.x3D; y2D=pt.z3D}
    | T_Z3D -> {x2D=pt.x3D; y2D=pt.y3D}

(* ============================================================================= *)
(* = Transformation d'un polygone 3D en polygone 2D en supprimant une coord    = *)
(* ============================================================================= *)
let poly_3d_to_poly_2d axis poly = List.map (pt_3d_to_pt_2d axis) poly

(* ============================================================================= *)
(* = Test d'un point dans un plan defini par 3 points                          = *)
(* ============================================================================= *)
let point_in_plane pt a b c =
  let det = eval_det4
    pt.x3D pt.y3D pt.z3D 1.
    a.x3D  a.y3D  a.z3D  1.
    b.x3D  b.y3D  b.z3D  1.
    c.x3D  c.y3D  c.z3D  1.
  in
  abs_float det < epsilon

(* ============================================================================= *)
(* = Determination des etendues du polygone sur chaque axe pour choisir le     = *)
(* = de projection -> limite les pbs d'instabilite numerique                   = *)
(* ============================================================================= *)
let span_poly poly =
  let set_min_max valeur min max =
    if valeur < !min then min:=valeur else if valeur > !max then max:=valeur
  in

  let p = List.hd (poly) in
  let minx = ref p.x3D and maxx = ref p.y3D and miny = ref p.y3D
  and maxy = ref p.y3D and minz = ref p.z3D and maxz = ref p.z3D in
  List.iter (fun p ->
    set_min_max p.x3D minx maxx ;
    set_min_max p.y3D miny maxy ;
    set_min_max p.z3D minz maxz) (List.tl poly) ;
  (* Renvoie les etendues sur chaque axe *)
  (!maxx-. !minx, !maxy-. !miny, !maxz-. !minz)

(* ============================================================================= *)
(* = Projection d'un polygone 3D sur le plan ayant les plus grandes etendues   = *)
(* ============================================================================= *)
let poly_3d_to_poly_2d_smallest_span poly =
  let (dx, dy, dz) = span_poly poly in
  let axis =
    if dx<dy then if dx<dz then T_X3D else T_Z3D
    else if dy<dz then T_Y3D else T_Z3D
  in
  (* On renvoie le polygone simplifie et l'axe utilise pour la simplification *)
  (poly_3d_to_poly_2d axis poly, axis)

(* ============================================================================= *)
(* = Test si le point est dans le polygone quand les 2 sont projetes en 2D     = *)
(* ============================================================================= *)
let point_in_poly_2D pt poly =
  if List.length poly >= 3 then begin
    (* Simplification du polygone *)
    let (new_poly, axis) = poly_3d_to_poly_2d_smallest_span poly in
    (* Simplification du point a tester suivant le meme axe *)
    let new_pt = pt_3d_to_pt_2d axis pt in
    (* Utilisation de la fonction 2D pour tester l'inclusion *)
    Geometry_2d.point_in_poly new_pt new_poly
  end else false

(* ============================================================================= *)
(* = Normale unitaire au plan contenant un polygone                            = *)
(* ============================================================================= *)
let poly_normal poly =
  if List.length poly >= 3 then begin
    (* 3 points du polygone qui definissent le plan le contenant *)
    let a = List.hd poly and b = List.hd (List.tl poly)
    and c = List.hd (List.tl (List.tl poly)) in
    (* Normale unitaire au plan contenant le polygone *)
    normal (vect_make a b) (vect_make b c)
  end else null_vector

(* ============================================================================= *)
(* = Test d'un point dans un polygone 3D                                       = *)
(* ============================================================================= *)
let point_in_poly pt poly =
  if List.length poly >= 3 then begin
    (* 3 points du polygone qui definissent le plan le contenant *)
    let a = List.hd poly and b = List.hd (List.tl poly)
    and c = List.hd (List.tl (List.tl poly)) in

    (* Le point est-il dans le plan contenant le polygone ? *)
    if point_in_plane pt a b c then
      (* Oui, on teste alors en projetant en 2D *)
      point_in_poly_2D pt poly
    else false
  end else false

(* ============================================================================= *)
(* = Aire signee d'un polygone 3D                                              = *)
(* ============================================================================= *)
let poly_signed_area poly =
  if List.length poly >= 3 then begin
    let poly_closed = poly_close2 poly and vect = ref null_vector in
    for i = 0 to (List.length poly)-1 do
      vect := vect_add !vect (cross_product poly_closed.(i) poly_closed.(i+1))
    done ;

    (dot_product (poly_normal poly) !vect)/.2.
  end else 0.

(* ============================================================================= *)
(* = Aire d'un polygone 3D                                                     = *)
(* ============================================================================= *)
let poly_area poly = abs_float (poly_signed_area poly)

(* ============================================================================= *)
(* = Evaluation du centroide d'un polygone 3D                                  = *)
(* ============================================================================= *)
let poly_centroid poly =
  (* On peut trianguler et ponderer le centre de chaque triangle par sa  *)
  (* surface mais on peut faire plus efficace. Ici, on prend un point du *)
  (* polygone (le premier par ex.) et on pondere l'aire (signee) des     *)
  (* triangles construits a partir de ce point.                          *)

  (* Centroide d'un triangle *)
  let centroid_triangle p1 p2 p3 =
    {x3D=(p1.x3D+.p2.x3D+.p3.x3D)/.3.;
     y3D=(p1.y3D+.p2.y3D+.p3.y3D)/.3.;
     z3D=(p1.z3D+.p2.z3D+.p3.z3D)/.3.} in

  (* Normale au plan contenant le polygone *)
  let n = poly_normal poly in

  (* Aire signee d'un triangle, pas besoin de poly_area... *)
  let area_triangle p1 p2 p3 =
    (dot_product n (cross_product (vect_make p1 p2) (vect_make p1 p3)))/.2. in

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
(* = Teste si un point est contenu dans un volume                              = *)
(* ============================================================================= *)
let point_in_volume pt vol =
  let t = Hashtbl.create 11 in

  (* Ajout des points a une hashtable pour compter les points en double/triple *)
  let add_point pt =
    try let nb = Hashtbl.find t pt in Hashtbl.replace t pt (nb+1)
    with Not_found -> Hashtbl.add t pt 1
  in

  (* Teste si le point d'intersection est sur une des aretes du volume *)
  let rec point_on_one_segment pt l =
    match l with
        poly::reste ->
          let rec f l =
            match l with
                p1::p2::reste ->
                  if point_on_segment pt p1 p2 then true else f (p2::reste)
              | _ -> false
          in
          if f (poly_close poly) then true else point_on_one_segment pt reste
      | [] -> false
  in

  (* Test supplementaire pour voir si les points d'intersections se trouvent sur *)
  (* un des sommets ou une des aretes *)
  let traite_pts_inter is_in =
    Hashtbl.iter (fun pt n ->
      (* n=2 -> arete, n=3 -> sommet *)
      if (n=2 or n=3) && point_on_one_segment pt vol then is_in:=not !is_in) t
  in

  let rec find_direction lst_faces =
    match lst_faces with
        poly::reste ->
        (* On essaie avec la direction entre le point et le centroide *)
        (* de la face courante *)
          let centroid = poly_centroid poly in
          let dir = vect_normalize (vect_make pt centroid) in

        (* Normale au plan du polygone pour avoir l'angle *)
          let n = poly_normal poly in

        (* Rappel : les deux vecteurs dir et n sont normalises donc pas besoin *)
        (* de diviser par le produit des normes pour avoir l'angle *)
          let s = dot_product dir n in

        (* On conserve cette direction si l'angle est inferieur a ~85 degres *)
          if abs_float s >=0.1 then dir
          else find_direction reste
      | [] -> {x3D=1.; y3D=0.; z3D=0.}
  in

  (* Choix d'une 'bonne' direction *)
  let dir = find_direction vol in

  (* On compte le nombre d'intersections entre la demi-droite issue du point *)
  (* a tester de vecteur directeur dir avec le volume *)

  let is_in = ref false in
  List.iter (fun poly_face ->
    if List.length poly_face>=3 then begin
      (* 3 points definissant le plan contenant la face *)
      let a = List.hd poly_face and b = List.hd (List.tl poly_face)
      and c = List.hd (List.tl (List.tl poly_face)) in

      (* Evaluation du point P' projete de P, suivant dir, sur le plan contenant *)
      (* la face poly_face *)
      match crossing_hline_plane pt dir a b c with
          None   -> () (* Pas d'intersection *)
        | Some p ->
          add_point p ;

          (* Le point projete est-il dans le polygone constituant la face ? *)
          if point_in_poly_2D p poly_face then
            (* Oui -> une intersection de plus *)
            is_in:=not !is_in
    end) vol ;

  (* Test supplementaires pour les sommets et les aretes *)
  traite_pts_inter is_in ;

  (* Nombre impair d'intersections -> le point est dans le volume *)
  !is_in


(* ============================================================================= *)
(* =                                                                           = *)
(* =                        Triangulation de polygones 3D                      = *)
(* =                par defaut ils sont consideres comme ouverts               = *)
(* =                                                                           = *)
(* ============================================================================= *)


(* ============================================================================= *)
(* = Triangulation d'un polygone                                               = *)
(* ============================================================================= *)
let tesselation poly =
  (* Projection en 2D pour utiliser la routine de tesselation 2D *)
  let (l, _) = poly_3d_to_poly_2d_smallest_span poly in

  (* Tesselation 2D *)
  let indices = Geometry_2d.in_tesselation l in

  (* On remet le polygone en 3D en utilisant les indices des points *)
  let t = Array.of_list poly in
  List.map (fun (p1, p2, p3) -> [t.(p1); t.(p2); t.(p3)]) indices

(* ============================================================================= *)
(* = Triangulation en triangles_fan                                            = *)
(* ============================================================================= *)
let tesselation_fans poly =
  (* Projection en 2D pour utiliser la routine de tesselation 2D *)
  let (l, _) = poly_3d_to_poly_2d_smallest_span poly in

  (* Tesselation 2D *)
  let (_, indices) = Geometry_2d.in_tesselation_fans l in

  (* On remet le polygone en 3D en utilisant les indices des points *)
  let t = Array.of_list poly in
  List.map (fun l -> List.map (fun x -> t.(x)) l) indices

(* =============================== FIN ========================================= *)
