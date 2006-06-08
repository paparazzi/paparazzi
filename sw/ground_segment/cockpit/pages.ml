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
class gps (widget: GBin.frame) =
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
    if not (List.mem_assoc svid active_cno)
    then begin
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
    end
    else begin
      let cno_label = List.assoc svid active_cno in
      let flags_eb = List.assoc svid active_flags in
      cno_label#set_label (cno^" dB Hz");
      update_color flags_eb flags
    end
end

(*****************************************************************************)
(* pfd page                                                                  *)
(*****************************************************************************)
class pfd (widget: GBin.frame) =
  let horizon = new Horizon.h ~packing: widget#add 150 in
    
object (this)
  val mutable pitch = 0.
  val mutable roll = 0.

  method set_roll r =
    roll <- r;
    horizon#draw ((Deg>>Rad)roll) ((Deg>>Rad)pitch)
  method set_pitch p =
    pitch <- p;
    horizon#draw ((Deg>>Rad)roll) ((Deg>>Rad)pitch)
  method set_alt (a:float) = ()
  method set_climb (c:float) = ()
end

class settings = fun xml_settings callback ->
  let sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
  let vbox = GPack.vbox ~packing:sw#add_with_viewport () in
  let n = List.length xml_settings in
  let current_values = Array.create n 42. in
  let i = ref 0 in
  let _ =
    List.iter
      (fun s ->
	let f = fun a -> float_of_string (ExtXml.attrib s a) in
	let lower = f "min"
	and upper = f "max"
	and step_incr = f "step" in
	let value = (lower +. upper) /. 2. in
	let text = ExtXml.attrib s "var" in
	let adj = GData.adjustment ~value ~lower ~upper:(upper+.10.) ~step_incr () in
	let hbox = GPack.hbox ~width:400 ~packing:vbox#add () in
	let _l = GMisc.label ~width:100 ~text ~packing:hbox#pack () in
	let _v = GMisc.label ~width:50 ~text:"N/A" ~packing:hbox#pack () in
	let _scale = GRange.scale `HORIZONTAL ~digits:2 ~adjustment:adj ~packing:hbox#add () in
	let ii = !i in
	
	let update_current_value = fun _ ->
	  let s = string_of_float current_values.(ii) in
	  if _v#text <> s then
	    _v#set_text s;
	  true in
	
	ignore (Glib.Timeout.add 500 update_current_value);
	let callback = fun _ -> callback ii adj#value in
	let b = GButton.button ~label:"Commit" ~stock:`APPLY ~packing:hbox#pack () in
	ignore (b#connect#clicked ~callback);
	
	incr i
      )
      xml_settings in
  object
    method length = n
    method widget = sw#coerce
    method set = fun i v -> current_values.(i) <- v
  end


