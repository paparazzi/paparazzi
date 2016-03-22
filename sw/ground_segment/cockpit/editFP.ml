(***************** Editing ONE (single) flight plan **************************)
open Printf
open Latlong

let (//) = Filename.concat
let fp_example = Env.flight_plans_path // "example.xml"
let default_path_maps =  Env.paparazzi_home // "data" // "maps"

(** Dummy flight plan (for map calibration) *)
let dummy_fp = fun latlong ->
  Xml.Element("flight_plan",
              ["lat0", string_of_float ((Rad>>Deg)latlong.posn_lat);
               "lon0", string_of_float ((Rad>>Deg)latlong.posn_long);
               "alt", "42.";
               "MAX_DIST_FROM_HOME", "1000."],
              [Xml.Element("waypoints", [],[]);
               Xml.Element("blocks", [],[])])



let current_fp = ref None

(** Wrapper checking there is currently no flight plan loaded *)
let if_none = fun f ->
  match !current_fp with
      Some _ ->
        GToolbox.message_box "Error" "Only one editable flight plan at a time"
    | None ->
      f ()

let set_window_title = fun geomap ->
  let title_suffix =
    match !current_fp with
        None -> ""
      | Some (_fp, xml_file) -> sprintf " (%s)" (Filename.basename xml_file) in
  match GWindow.toplevel geomap#canvas with
      Some w ->
        w#set_title (sprintf "Flight Plan Editor%s" title_suffix)
    | None -> ()



let save_fp = fun geomap ->
  match !current_fp with
      None -> () (* Nothing to save *)
    | Some (fp, filename) ->
      match GToolbox.select_file ~title:"Save Flight Plan" ~filename () with
          None -> ()
        | Some file ->
          let f  = open_out file in
          let fp_path = Str.replace_first (Str.regexp Env.flight_plans_path) "" (Filename.dirname file) in
          let rel_path = Str.global_replace (Str.regexp (Printf.sprintf "%s[^%s]+" Filename.dir_sep Filename.dir_sep)) (Filename.parent_dir_name // "") fp_path in
          fprintf f "<!DOCTYPE flight_plan SYSTEM \"%s%s\">\n\n" rel_path "flight_plan.dtd";
          fprintf f "%s\n" (ExtXml.to_string_fmt fp#xml);
          close_out f;
          current_fp := Some (fp, file);
          set_window_title geomap


let close_fp = fun geomap ->
  match !current_fp with
      None -> () (* Nothing to close *)
    | Some (fp, _filename) ->
      let close = fun () ->
        fp#destroy ();
        geomap#clear_georefs ();
        current_fp := None in
      match GToolbox.question_box ~title:"Closing flight plan" ~buttons:["Close"; "Save&Close"; "Cancel"] "Do you want to save/close ?" with
          2 -> save_fp geomap; close ()
        | 1 -> close ()
        | _ -> ()

let load_xml_fp = fun geomap editor_frame _accel_group ?(xml_file=Env.flight_plans_path) xml ->
  Map2d.set_georef_if_none geomap (MapFP.georef_of_xml xml);
  let fp = new MapFP.flight_plan ~editable:true ~show_moved:false geomap "red" Env.flight_plan_dtd xml in
  editor_frame#add fp#window;
  current_fp := Some (fp,xml_file);

   (** Add waypoints as geo references *)
  List.iter
    (fun w ->
      let (_i, w) = fp#index w in
      geomap#add_info_georef (sprintf "%s" w#name) (w :> < pos : Latlong.geographic >))
    fp#waypoints;

  fp

let labelled_entry = fun ?width_chars text value h ->
  let _ = GMisc.label ~text ~packing:h#add () in
  GEdit.entry ?width_chars ~text:value ~packing:h#add ()

let new_fp = fun geomap editor_frame accel_group () ->
  if_none (fun () ->
    let dialog = GWindow.window ~border_width:10 ~title:"New flight plan" () in
    let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
    let h = GPack.hbox ~packing:dvbx#pack () in
    let default_latlong =
      match geomap#georef with
          None -> "WGS84 37.21098 -113.45678"
        | Some geo -> Latlong.string_of geo in
    let latlong  = labelled_entry ~width_chars:25 "Geographic Reference" default_latlong h in
    let alt0 = labelled_entry ~width_chars:4 "Ground Alt" "380" h in
    let h = GPack.hbox ~packing:dvbx#pack () in
    let alt = labelled_entry ~width_chars:4 "Default Alt" "430" h in
    let qfu = labelled_entry ~width_chars:4 "QFU" "270" h in
    let mdfh = labelled_entry ~width_chars:4 "Max distance from HOME" "500" h in

    let h = GPack.hbox ~packing:dvbx#pack () in
    let name  = labelled_entry "Name" "Test flight" h in

    let h = GPack.hbox ~packing:dvbx#pack () in
    let cancel = GButton.button ~stock:`CANCEL ~packing: h#add () in
    ignore(cancel#connect#clicked ~callback:dialog#destroy);

    let createfp = GButton.button ~stock:`OK ~packing: h#add () in
    createfp#grab_default ();
    ignore(createfp#connect#clicked ~callback:
             begin fun _ ->
               let xml = ExtXml.parse_file fp_example in
               let s = ExtXml.subst_attrib in
               let wgs84 = Latlong.of_string latlong#text in
               let xml = s "lat0" (deg_string_of_rad wgs84.posn_lat) xml in
               let xml = s "lon0" (deg_string_of_rad wgs84.posn_long) xml in
               let xml = s "ground_alt" alt0#text xml in
               let xml = s "qfu" qfu#text xml in
               let xml = s "alt" alt#text xml in
               let xml = s "max_dist_from_home" mdfh#text xml in
               let xml = s "name" name#text xml in
               ignore (load_xml_fp geomap editor_frame accel_group xml);
               dialog#destroy ()
             end);
    dialog#show ())


let loading_error = fun xml_file e ->
  let m = sprintf "Error while loading %s:\n%s" xml_file e in
  GToolbox.message_box "Error" m



let load_xml_file = fun geomap editor_frame accel_group xml_file ->
  try
    let xml = Xml.parse_file xml_file in
    ignore (load_xml_fp geomap editor_frame accel_group ~xml_file xml);
    geomap#fit_to_window ();
    set_window_title geomap
  with
      Dtd.Prove_error(e) -> loading_error xml_file (Dtd.prove_error e)
    | Dtd.Check_error(e) -> loading_error xml_file (Dtd.check_error e)
    | Xml.Error e -> loading_error xml_file (Xml.error e)



(** Loading a flight plan for edition *)
let load_fp = fun geomap editor_frame accel_group () ->
  if_none (fun () ->
    match GToolbox.select_file ~title:"Open flight plan" ~filename:(Env.flight_plans_path // "*.xml") () with
        None -> ()
      | Some xml_file -> load_xml_file geomap editor_frame accel_group xml_file)

let create_wp = fun geomap geo ->
  match !current_fp with
      None ->
        GToolbox.message_box "Error" "Load a flight plan first";
        failwith "create_wp"
    | Some (fp,_) ->
      let w = fp#add_waypoint geo in
      geomap#add_info_georef (sprintf "%s" w#name) (w :> < pos : Latlong.geographic >);
      w



let ref_point_of_waypoint = fun xml ->
  Xml.Element("point", ["x",Xml.attrib xml "x";
                        "y",Xml.attrib xml "y";
                        "geo", Xml.attrib xml "name"],[])


(** Calibration of chosen image (requires a dummy flight plan) *)
let calibrate_map = fun (geomap:MapCanvas.widget) editor_frame accel_group () ->
  match !current_fp with
    | Some (_fp,_) ->  GToolbox.message_box "Error" "Close current flight plan before calibration"
    | None ->
      match GToolbox.select_file ~filename:(default_path_maps // "") ~title:"Open Image" () with
          None -> ()
        | Some image ->
      (** Displaying the image in the NW corner *)
          let pixbuf = GdkPixbuf.from_file image in
          let pix = GnoCanvas.pixbuf ~pixbuf ~props:[`ANCHOR `NW] geomap#canvas#root in
          let (x0, y0) = geomap#canvas#get_scroll_offsets in
          let (x,y) = geomap#canvas#window_to_world (float x0) (float y0) in
          pix#move x y;

      (** Open a dummy flight plan *)
          let dummy_georef =
            match geomap#georef with
                None -> {posn_lat = (Deg>>Rad)43.; posn_long = (Deg>>Rad)1. }
              | Some geo -> geo in
          let fp_xml = dummy_fp dummy_georef in
          let fp = load_xml_fp geomap editor_frame accel_group fp_xml in

      (** Dialog to finish calibration *)
          let dialog = GWindow.window ~border_width:10 ~title:"Map calibration" () in
          let v = GPack.vbox ~packing:dialog#add () in
          let _ = GMisc.label ~text:"Choose 2 (or more) waypoints (Ctrl-Left)\nRename the waypoints with their geographic coordinates\nFor example: 'WGS84 43.123456 1.234567' or 'UTM 530134 3987652 12' or 'LBT2e 123456 543210'\nClick the button below to save the XML result file\n" ~packing:v#add () in
          let h = GPack.hbox ~packing:v#pack () in
          let cancel = GButton.button ~stock:`CLOSE ~packing:h#add () in
          let cal = GButton.button ~stock:`OK ~packing:h#add () in
          let destroy = fun () ->
            dialog#destroy ();
            close_fp geomap;
            pix#destroy () in
          ignore(cancel#connect#clicked ~callback:destroy);
          ignore(cal#connect#clicked ~callback:(fun _ ->
            let points = List.map XmlEdit.xml_of_node fp#waypoints in
            let points = List.map ref_point_of_waypoint points in
            let xml = Xml.Element ("map",
                                   ["file", Filename.basename image;
                                    "projection", geomap#projection],
                                   points) in
            match GToolbox.select_file ~filename:(default_path_maps//".xml") ~title:"Save calibrated map" () with
                None -> ()
              | Some xml_file ->
                let f = open_out xml_file in
                Printf.fprintf f "%s\n" (Xml.to_string_fmt xml);
                close_out f));
          cal#grab_default ();
          dialog#show ()
