(*
 * $Id$
 *
 * 3D OpenGL visualisation
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec, Pascal Brisset, Antoine Drouin
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

open Ocaml_tools
open Geometry_3d

open Gtk_3d
open Latlong

let fos = fun x -> 
  try
    float_of_string x
  with
    Failure("float_of_string") -> failwith ("float_of_string: "^ x)
let float_attrib = fun xml a -> fos (ExtXml.attrib xml a)

(* Version de l'appli *)
let version = "0.1"

(* 1 point tous les 50m *)
let dx = 50 and dy = 50

(* Facteur d'echelle pour les altitudes *)
let fact_alti = 3

(* Taille de la fenetre d'affichage *)
let width = 800 and height = 600

let tolerance_alti = 100.

let default_color_trajs = ref (`NAME "red")

let home = Env.paparazzi_home
let (//) = Filename.concat
let default_path_SRTM = home // "data" // "SRTM"
let default_path_maps = home // "data" // ""
let default_path_traj = home // "var" // ""
let default_path_missions = home // "conf"

let color_pixmaps = Hashtbl.create 101

let limits = ref ((min_float,min_float), (max_float, max_float))

let track_filter = fun points ->
  let ((minx,miny), (maxx, maxy)) = !limits in
  let rec loop = function
      [] -> []
    | (t,x,y,a)::ps ->
	if minx < x && x < maxx && miny < y && y < maxy
	then (t,x,y,a)::loop ps
	else loop ps in
  loop points

(* Fichier d'aide contenant les touches clavier utilisees *)
let filename_help_keys = Env.paparazzi_src // "conf" // "Help_Keys.txt"

(* ============================================================================= *)
(* = Passage de couleur GTK vers GL                                            = *)
(* =                                                                           = *)
(* = color = couleur GTK (`NAME ou `RGB) a transformer                         = *)
(* ============================================================================= *)
let gtk_to_gl_color color =
  let t = GDraw.color color in
  ((float_of_int (Gdk.Color.red t))/.65535.0,
   (float_of_int (Gdk.Color.green t))/.65535.0,
   (float_of_int (Gdk.Color.blue t))/.65535.0)

(* ============================================================================= *)
(* = Passage de couleur GL vers GTK                                            = *)
(* =                                                                           = *)
(* = (r, g, b) = couleur GL a transformer en equivalent GTK                    = *)
(* ============================================================================= *)
let gl_to_gtk_color (r, g, b) =
  `RGB(int_of_float (r*.65535.0), int_of_float (g*.65535.0),
       int_of_float (b*.65535.0))

(* ============================================================================= *)
(* = Lecture d'un fichier de trajectoire avec correction des points etranges   = *)
(* ============================================================================= *)
let read_traj_file filename =
  let traj = ref [] and prev_alti = ref None in
  let corrige_alt alt =
    let x =
      match !prev_alti with
	None -> alt
      | Some prev_alti ->
	  if abs_float(alt-.prev_alti)<tolerance_alti then alt
	  else prev_alti
    in
    prev_alti:=Some x;
    x
  in

  let match_func l error_func =
    match l with
      time::"GPS"::mode::utm_x::utm_y::_::alt::_ ->
	(try
	  if mode = "3" then (* Else no pertinent info available *)
	    let t = fos time
	    and utm_x = (fos utm_x)/.100.
	    and utm_y = (fos utm_y)/.100.
	    and alt = (fos alt) in
	    (* Filtrage des altitudes incorrectes *)
	    let alt = corrige_alt alt in
	    traj:=(t, utm_x, utm_y, alt)::!traj
	with _ -> error_func ())
    | _ -> ()
  in
  do_read_file filename match_func (fun () -> ()) ;
  let traj' = track_filter (List.rev !traj) in
  (traj', !default_color_trajs)

(* ============================================================================= *)
(* = Ajout de la surface                                                       = *)
(* ============================================================================= *)
let add_surface view3d texture_file (min_x, min_y) (max_x, max_y) utm_zone =
  (* Creation de la texture a partir d'une image *)
  Printf.printf "Lecture texture..."; flush stdout ;
  let texture_id = Gtk_3d.create_texture_from_image texture_file in
  Printf.printf " OK\n"; flush stdout ;

  (* Creation d'une matrice contenant les elevations *)
  let nx = (max_x-min_x)/dx+1 and ny = (max_y-min_y)/dy+1 in
  let tab = Array.make_matrix ny nx {x3D=0.; y3D=0.; z3D=0.} in

  let y = ref max_y and i = ref 0 in
  try
    for i = 0 to ny - 1 do
      let x = ref min_x in
      for j = 0 to nx - 1 do
	let alt = (Srtm.of_utm {utm_x = float !x; utm_y = float !y; utm_zone = utm_zone})*fact_alti in
	tab.(i).(j) <- {x3D = float !x; y3D= float !y; z3D = float alt} ;
	x:=!x+dx
      done ;
      y:=!y-dy
    done ;
    
    (* Ajout de cette matrice a la vue 3D *)
    view3d#add_object_surface_with_texture tab texture_id
  with
    Srtm.Tile_not_found s ->
      failwith (Printf.sprintf "SRTM tile '%s' not found,  you can download it with %s" s (Srtm.error s))

(* ============================================================================= *)
(* = Ajout d'une trajectoire                                                   = *)
(* ============================================================================= *)
let point3D = fun (_, utm_x, utm_y, alt) -> {x3D=utm_x; y3D=utm_y; z3D=alt*. float fact_alti}
let add_traj view3d (points, id) =
  let l = List.map point3D points in

  let color = gtk_to_gl_color id in
  view3d#add_object_line l color 2 false false

(* Adding one more point to a track *)
let last_points = Hashtbl.create 11
let add_point (view3d:Gtk_3d.widget_3d) (point, id) =
  let p = point3D point in
  try
    let last = Hashtbl.find last_points id in
    let color = gtk_to_gl_color id in
    view3d#display (view3d#add_object_line [last;p] color 2 false false);
    Hashtbl.replace last_points id p
  with
    Not_found ->
      Hashtbl.add last_points id p
 
(* ============================================================================= *)
(* = Load a map. Use SRTM elevation data to produce a 3d surface               = *)
(* ============================================================================= *) 
let load_surface view3d id_sol xml_map_file =
  let min_x = ref max_int and min_y = ref max_int
  and max_x = ref min_int and max_y = ref min_int  
  and texture_file = ref "" in 
  let xml = Xml.parse_file xml_map_file in
  let texture_file = Xml.attrib xml "file" in
  let texture_file = Filename.concat (Filename.dirname xml_map_file) texture_file in
  let (_format, header) = Images.file_format texture_file in
  let int_attrib x a = int_of_string (Xml.attrib x a) in
  begin
    match Xml.children xml with
      p::_ ->
	let utm_x = float_attrib p "utm_x"
	and utm_y = float_attrib p "utm_y"
	and x = float_attrib p "x"
	and y = float_attrib p "y"
	and scale = float_attrib xml "scale" in
	min_x := truncate (utm_x -. scale *. x);
	min_y := truncate (utm_y -. scale *. (float header.Images.header_height -. y));
	max_x := truncate (utm_x +. scale *. (float header.Images.header_width -. x));
	max_y := truncate (utm_y +. scale *. y)
    | _ -> failwith "load_surface"
  end;
  begin
    match !id_sol with
      Some x -> view3d#delete_object x
    | None -> ()
  end;
  let utm_zone = try int_of_string (Xml.attrib xml "utm_zone") with _ -> Printf.fprintf stderr "Warning: utm_zone attribute not specified in '%s'; default is 31\n" xml_map_file; flush stderr; 31 in
  id_sol:= Some (add_surface view3d texture_file (!min_x, !min_y) (!max_x, !max_y) utm_zone);
  limits := ((float !min_x, float !min_y), (float !max_x, float !max_y));
  view3d#display_func


let load_mission = fun (view3d:Gtk_3d.widget_3d) xml ->
  let wps = ExtXml.child xml "waypoints" in
  let utm_x0 = float_attrib wps "utm_x0"
  and utm_y0 = float_attrib wps "utm_y0" in
  let display_waypoint = fun wp ->
    let utm_x = float_attrib wp "x" +. utm_x0
    and utm_y = float_attrib wp "y" +. utm_y0
    and alt = float_attrib wp "alt" in
    let p3d = point3D (0., utm_x, utm_y, alt)
    and p3d_label = point3D (0., utm_x+.10., utm_y+.10., alt) in
    view3d#display (view3d#add_object_point p3d p3d_label (ExtXml.attrib wp "name") (gtk_to_gl_color (`NAME "red")) true) in
  List.iter display_waypoint (Xml.children wps)

(* ============================================================================= *)
(* = Map loading callback                                                      = *)
(* ============================================================================= *)
let on_load_surface win view3d id_sol () =
  let priv_load_surf xml_map_file =
    load_surface view3d id_sol xml_map_file 
  in
  try
    Gtk_tools.open_file_dlg "Map calibration file" priv_load_surf None default_path_maps false
  with x ->
    Gtk_tools.error_box win "Read error" (Printexc.to_string x)


(* ============================================================================= *)
(* = Chargement d'une trajectoire                                              = *)
(* ============================================================================= *)
let load_trajectory view3d lst_ids_trajs () =
  let read_data f =
    let new_traj = read_traj_file f in
    lst_ids_trajs:=(add_traj view3d new_traj)::!lst_ids_trajs ;
    (* Force la mise a jour de l'affichage *)
    view3d#display_func
  in

  Gtk_tools.open_file_dlg "Track" read_data None default_path_traj false

(* ============================================================================= *)
(* = Recherche/Ajout d'une pixmap de couleur dans la table                     = *)
(* ============================================================================= *)
let get_color_pixmap win color =
  let taille_x = 20 and taille_y = 8 in
  try Hashtbl.find color_pixmaps color
  with Not_found ->
    let pm = Gtk_tools.rectangle_pixmap win color taille_x taille_y in
    Hashtbl.add color_pixmaps color pm ;
    pm

(* ============================================================================= *)
(* = Selection de trajectoires                                                 = *)
(* ============================================================================= *)
let build_lst_traj tooltips view3d lst_ids_trajs () =
  let get_id idx = try List.nth !lst_ids_trajs idx with _ -> (-1) in

  let (window,boite) = Gtk_tools.create_window "Liste des trajectoires" 450 300 in
  let lst = Gtk_tools.create_managed_list
      [("Id", 40); ("Sel.", 40); ("Color", 60)] boite#add
  in
  let buts = Gtk_tools.create_buttons
      [("Hide", "Hide the track");
       ("Display", "Display the track");
       ("Color", "Change the color");
       ("Delete", "Delete the track") ;
       ("Close", "Close the window")] tooltips boite#pack
  in
  let but_masque  = List.nth buts 0 and but_aff = List.nth buts 1
  and but_couleur = List.nth buts 2 and but_del = List.nth buts 3 in
  Gtk_tools.set_sensitive_list
    [but_couleur; but_del; but_masque; but_aff] false ;

  let current_selection = ref (-1) and current_idx = ref "" in
  let callback_traj index _ selection =
    if selection then begin
      current_selection:=get_id (int_of_string index) ;
      current_idx:=index ;
      let masquable = view3d#object_get_visibility !current_selection in
      Gtk_tools.set_sensitive but_masque masquable ;
      Gtk_tools.set_sensitive but_aff (not masquable) ;
      Gtk_tools.set_sensitive_list [but_couleur; but_del] true ;
    end else begin
      current_selection:=(-1); current_idx:="" ;
      Gtk_tools.set_sensitive_list
	[but_couleur; but_del; but_masque; but_aff] false
    end
  in
  let fill_list_traj = Gtk_tools.connect_managed_list
      lst 0 callback_traj ("track", true, true)
  in
  let fill_list () =
    let to_select = !current_idx in
    current_selection:=(-1); current_idx:="" ;
    let n = ref (-1) in
    let l = List.map (fun id ->
      incr n ;
      [string_of_int !n;
       (if view3d#object_get_visibility id then " x " else ""); ""]
		     ) !lst_ids_trajs in
    fill_list_traj to_select l ;
    let row = ref 0 in
    List.iter (fun id ->
      let c = gl_to_gtk_color (view3d#object_get_color id) in
      (fst lst)#set_cell !row 2 ~pixmap:(get_color_pixmap window c) ;
      incr row) !lst_ids_trajs
  in
  fill_list () ;

  let func_masque_aff affiche =
    if !current_selection<>(-1) then begin
      view3d#object_set_visibility !current_selection affiche ;
      view3d#display_func ;
      fill_list ()
    end
  in
  let change_color () =
    if !current_selection<>(-1) then begin
      Gtk_tools.select_color (fun color ->
	view3d#object_set_color !current_selection (gtk_to_gl_color color) ;
	view3d#display_func ;
	fill_list ()) ;
    end	
  in
  let delete_traj () =
    if !current_selection<>(-1) then begin
      view3d#delete_object !current_selection; view3d#display_func ;
      lst_ids_trajs:=
	List.filter (fun id -> id <> !current_selection) !lst_ids_trajs ;
      fill_list ()
    end
  in

  Gtk_tools.create_buttons_connect buts
    [(fun () -> func_masque_aff false); (fun () -> func_masque_aff true);
     change_color; delete_traj;
     (fun () -> window#destroy (); view3d#display_func)] ;
  window#show ()

(* ============================================================================= *)
(* = Fenetre About                                                             = *)
(* ============================================================================= *)
let build_fen_about () =
  (* Creation de la liste des fichiers de l'animation *)
  let l = ref [] and max_pixmaps = 15 in
  for i=1 to max_pixmaps do
    l:=(Printf.sprintf "Pixmaps/avion%d.xpm" i)::!l
  done ;

  let message = Printf.sprintf "Visu Drone v%s\n" version in

  Gtk_tools.animated_msg_box "About" message (List.rev !l)

(* ============================================================================= *)
(* = Creation de l'interface                                                   = *)
(* ============================================================================= *)
let build_interface = fun map_file mission_file ->
  let nb_menus = ref 0 in

  (* Liste des menus disponibles *)
  let liste_menus = ["Map"; "Tracks"; "Parameters"] in

  (* Mise en place des couleurs correctes *)
  Gtk_tools.init_colors () ;

  (* Initialisation de l'aide contextuelle *)
  let tooltips = Gtk_tools.init_tooltips () in

  (* Creation d'une fenetre *)
  let (window, vbox, factory, accel_group, menus, menu_help) =
    Gtk_tools.create_window_with_menubar_help ("Visu Drone v"^version)
      width height liste_menus in
  ignore (window#connect#destroy ~callback:GMain.Main.quit);

  (* Creation du Widget OpenGL *)
  let view3d = new widget_3d vbox#add false "" in

  (* Ajout des objets a la vue *)
  let id_sol = ref None in
  let lst_ids_trajs = ref [] in

  (* Creation des menus : Sol *)
  let factory = new GMenu.factory menus.(!nb_menus) ~accel_group in
  incr nb_menus ;
  ignore (factory#add_item "Load Background"
    ~callback:(on_load_surface window view3d id_sol)) ;

  (* Creation des menus : Trajectoires *)
  let factory = new GMenu.factory menus.(!nb_menus) ~accel_group in
  incr nb_menus ;
  ignore (factory#add_item "Load Track" ~callback:(load_trajectory view3d lst_ids_trajs)) ;
  ignore (factory#add_item "Edit Tracks"
    ~callback:(build_lst_traj tooltips view3d lst_ids_trajs)) ;

  (* Creation des menus : Parametres *)
  let factory = new GMenu.factory menus.(!nb_menus) ~accel_group in
  incr nb_menus ;
  ignore (factory#add_item "Edit" ~callback:(fun () -> ())) ;

  (* Aide *)
  let factory = new GMenu.factory menu_help in
  ignore (factory#add_item "A propos" ~callback:build_fen_about) ;
  ignore (factory#add_item "Help keys/mouse"
    ~callback:(fun () -> Gtk_tools.display_file filename_help_keys
	"Aide touches clavier" 370 500 tooltips (Some "fixed"))) ;

  (* Affichage de la fenetre principale *)
  window#show () ;

(*  let gps_regexp = "([a-z]*) +GPS +[0-9]* +([0-9]*) +([0-9]*) +[0-9\\.]* +([0-9\\.]*)" in
  ignore (Ivy.bind (fun _ args -> add_point view3d ((0., fos args.(1)/.100.,fos args.(2)/.100., fos args.(3)), `NAME "green")) gps_regexp); *)
  

  let flight_param_regexp = "([a-z0-9]*) +FLIGHT_PARAM +[0-9\\.]* +[0-9\\.]* +([0-9\\.]*) +([0-9\\.]*) +[0-9\\.]* +[0-9\\.]* +([0-9\\.]*) +[0-9\\.]*" in
  ignore (Ivy.bind (fun _ args -> 
    let name= args.(0) and
	x = fos args.(1) and
	y = fos args.(2) and
	z = fos args.(3) in
 (*   Printf.fprintf stderr "############ %f %f %f\n" x y z; *)
    if (Str.string_match (Str.regexp_string "twinstar1") name 0) then 
      add_point view3d ((0., x, y, z), `NAME "green");
    if (Str.string_match (Str.regexp_string "twinstar2") name 0) then 
      add_point view3d ((0., x, y, z), `NAME "blue"); 
(*    Printf.fprintf stderr "############\n"; *)
		   ) flight_param_regexp);


  (* Loading an initial map *)
  if map_file <> "" then begin
    let xml_map_file = Filename.concat (default_path_maps) map_file in
    load_surface view3d id_sol xml_map_file
  end;

  (* Loading an initial mission *)
  if mission_file <> "" then begin
    let xml_file = Filename.concat (default_path_missions) mission_file in
    load_mission view3d (Xml.parse_file xml_file)
  end;

 (* Lancement de la mainloop *)
  Gtk_tools.main_loop () 

(* ============================================================================= *)
(* = Programme principal                                                       = *)
(* ============================================================================= *)
let _ =
  let ivy_bus = ref "127.255.255.255:2010" and
      map_file = ref "" and
      mission_file = ref "" in
  let options =
    [ "-b", Arg.String (fun x -> ivy_bus := x), "Bus\tDefault is 127.255.255.25:2010";
      "-m", Arg.String (fun x -> map_file := x), "Map description file";
      "-f", Arg.String (fun x -> mission_file := x), "Mission description file"] in
  Arg.parse (options)
    (fun x -> Printf.fprintf stderr "Warning: Don't do anythig with %s\n" x)
    "Usage: ";
  (*                                 *)
  Ivy.init "Paparazzi 3d visu" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  Srtm.add_path default_path_SRTM;

  (* Lancement de l'interface *)
  build_interface !map_file !mission_file

 
(* =============================== FIN ========================================= *)
