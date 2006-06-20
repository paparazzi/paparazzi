(*****************************************************************************)
(* $id$                                                                      *)
(*                                                                           *)
(* information pages such as alert, infrared, gps, artificial horizon        *)
(*****************************************************************************)

open Latlong

(** alert page *)
class alert (widget: GBin.frame) =
  let scrolled = GBin.scrolled_window 
    ~hpolicy: `AUTOMATIC 
    ~vpolicy: `AUTOMATIC  
    ~packing: widget#add
    () 
  in
  let view = GText.view ~editable:false ~packing: scrolled#add () in
(* the object itselft *)
object (this)
  val active = Hashtbl.create 5
  method add text = 
    if not (Hashtbl.mem active text) then begin
      view#buffer#insert text;
      view#buffer#insert "\n";
      Hashtbl.add active text ()
    end
 end

(*****************************************************************************)
(* infrared page                                                             *)
(*****************************************************************************)
class infrared (widget: GBin.frame) =
  let table = GPack.table
    ~rows: 4
    ~columns: 2
    ~row_spacings: 5
    ~col_spacings: 5
    ~packing: widget#add
    ()
  in
  let contrast_status = 
    GMisc.label ~text: "" ~packing: (table#attach ~top:0 ~left: 1) ()
      in
  let contrast_value = 
    GMisc.label ~text: "" ~packing: (table#attach ~top:1 ~left: 1) ()
  in
  let gps_hybrid_mode = 
    GMisc.label ~text: "" ~packing: (table#attach ~top:2 ~left: 1) ()
  in
  let gps_hybrid_factor = 
    GMisc.label ~text: "" ~packing: (table#attach ~top:3 ~left: 1) ()
  in
  let _init =
    ignore (GMisc.label ~text: "contrast status" ~packing: (table#attach ~top:0 ~left: 0) ());
    ignore (GMisc.label ~text: "contrast" ~packing: (table#attach ~top:1 ~left: 0) ());
    ignore (GMisc.label ~text: "gps hybrid mode" ~packing: (table#attach ~top:2 ~left: 0) ());
    ignore (GMisc.label ~text: "gps hybrid factor" ~packing: (table#attach ~top:3 ~left: 0) ())
  in
object (this)
  val parent = widget
  val table = table

  val contrast_status = contrast_status
  val contrast_value = contrast_value
  val gps_hybrid_mode = gps_hybrid_mode
  val gps_hybrid_factor = gps_hybrid_factor

  method set_contrast_status (s:string) = 
    contrast_status#set_label s
  method set_contrast_value (s:int) =
    contrast_value#set_label (Printf.sprintf "%d" s)
  method set_gps_hybrid_mode (s:string) = 
    gps_hybrid_mode#set_label s
  method set_gps_hybrid_factor (s:float) =
    gps_hybrid_factor#set_label (Printf.sprintf "%.8f" s)
end

(*****************************************************************************)
(* gps page                                                                  *)
(*****************************************************************************)
class gps ?(visible = fun _ -> true) (widget: GBin.frame) =
  let table = GPack.table
      ~rows: 1
      ~columns: 3
      ~row_spacings: 5
      ~col_spacings: 40
      ~packing: widget#add
      () in
  let update_color = fun flags_eb flags ->
    let color = if flags land 0x01 = 1 then "green" else "red" in
    flags_eb#coerce#misc#modify_bg [`NORMAL, `NAME color] in
object (this)
  val parent = widget
  val table = table
  val mutable active_cno = []
  val mutable active_flags = []

  method svsinfo svid cno flags =
    if not (List.mem_assoc svid active_cno) then
      let rows = table#rows in
      let _svid_label = GMisc.label 
	  ~text: ("sat "^ svid) ~packing: (table#attach ~top: rows ~left: 0) () in
      let cno_label = GMisc.label 
	  ~text:(cno^" dB Hz") ~packing: (table#attach ~top: rows ~left: 1) () in
      let flags_eb = GBin.event_box ~width: 20 ~packing:(table#attach ~top: rows ~left: 2) ()
      in
      update_color flags_eb flags;
      active_cno <- (svid, cno_label)::active_cno;
      active_flags <- (svid, flags_eb)::active_flags;
      table#set_rows (table#rows +1)
    else if visible widget then
      let cno_label = List.assoc svid active_cno in
      let flags_eb = List.assoc svid active_flags in
      cno_label#set_label (cno^" dB Hz");
      update_color flags_eb flags
end

(*****************************************************************************)
(* pfd page                                                                  *)
(*****************************************************************************)
class pfd ?(visible = fun _ -> true) (widget: GBin.frame) =
  let horizon = new Horizon.h ~packing: widget#add 150 in
  let _lazy = fun f x -> if visible widget then f x in
    
object (this)
  method set_attitude roll pitch =
    _lazy (horizon#set_attitude ((Deg>>Rad)roll)) ((Deg>>Rad)pitch)
  method set_alt (a:float) = _lazy horizon#set_alt a
  method set_climb (c:float) = ()
  method set_speed (c:float) = _lazy horizon#set_speed c
end


(*****************************************************************************)
(* Misc page                                                                 *)
(*****************************************************************************)
class misc ~packing (widget: GBin.frame) =
  let table = GPack.table
      ~rows: 2
      ~columns: 2
      ~row_spacings: 5
      ~col_spacings: 40
      ~packing
      () in
  let label = fun text i j ->GMisc.label ~text ~packing:(table#attach ~top:i ~left:j) () in
  let _init =
    ignore (label "Wind speed" 0 0);
    ignore (label "Wind direction" 1 0) in
  let wind_speed = label "" 0 1
  and wind_dir = label "" 1 1 in
  object
    method set_wind_speed s = wind_speed#set_text s
    method set_wind_dir s = wind_dir#set_text s
  end

(*****************************************************************************)
(* Dataling settings paged                                                   *)
(*****************************************************************************)
class settings = fun ?(visible = fun _ -> true) xml_settings callback ->
  let sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
  let vbox = GPack.vbox ~packing:sw#add_with_viewport () in
  let n = List.length xml_settings in
  let current_values =
    Array.mapi
      (fun i s ->
	let f = fun a -> float_of_string (ExtXml.attrib s a) in
	let lower = f "min"
	and upper = f "max"
	and step_incr = f "step" in

	let hbox = GPack.hbox ~width:400 ~packing:vbox#add () in
	let text = ExtXml.attrib s "var" in
	let _l = GMisc.label ~width:100 ~text ~packing:hbox#pack () in
	let _v = GMisc.label ~width:50 ~text:"N/A" ~packing:hbox#pack () in
	let commit = GButton.button ~label:"Commit" ~stock:`APPLY () in
	(** For a small number of values, radio buttons *)
	let n = truncate ((upper -. lower) /. step_incr) in
	let callback =
	  if step_incr = 1. && upper -. lower <= 2. then
	    let ilower = truncate lower
	    and iupper = truncate upper in
	    let label = Printf.sprintf "%d" ilower in
	    let first = GButton.radio_button ~label ~packing:hbox#add () in
	    let value = ref lower in
	    ignore (first#connect#clicked (fun () -> value := lower));
	    let group = first#group in
	    for j = ilower+1 to iupper do
	      let label = Printf.sprintf "%d" j in
	      let b = GButton.radio_button ~group ~label ~packing:hbox#add () in
	      ignore (b#connect#clicked (fun () -> value := float j))
	    done;
	    (fun _ -> callback i !value)
	  else (* slider *)
	    let value = (lower +. upper) /. 2. in
	    let adj = GData.adjustment ~value ~lower ~upper:(upper+.10.) ~step_incr () in
	    let _scale = GRange.scale `HORIZONTAL ~digits:2 ~adjustment:adj ~packing:hbox#add () in
	    
	    (fun _ -> callback i adj#value)
	in
	hbox#pack commit#coerce;
	ignore (commit#connect#clicked ~callback);
	_v
      )
      (Array.of_list xml_settings) in
  object (self)
    method length = n
    method widget = sw#coerce
    method set = fun i v ->
      if visible self#widget then
	let s = string_of_float v in
	if current_values.(i)#text <> s then
	  current_values.(i)#set_text s
  end


