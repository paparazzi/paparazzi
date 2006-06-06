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
    ()
  in
object (this)
  val parent = widget
  val table = table
  val mutable active_cno = []
  val mutable active_flags = []

  method svsinfo svid cno (flags:string) =
    if not (List.mem_assoc svid active_cno)
    then begin
	let rows = table#rows in
	let _svid_label = GMisc.label 
	  ~text: ("sat "^ svid) ~packing: (table#attach ~top: rows ~left: 0) () in
	let cno_label = GMisc.label 
	  ~text:(cno^" dB Hz") ~packing: (table#attach ~top: rows ~left: 1) () in
	let flags_eb = GBin.event_box ~width: 20 ~packing:(table#attach ~top: rows ~left: 2) ()
	in
	  if flags="1" 
	  then
	    flags_eb#coerce#misc#modify_bg [`NORMAL, `NAME "green";`ACTIVE, `NAME "green"]
	  else
	    flags_eb#coerce#misc#modify_bg [`NORMAL, `NAME "red";`ACTIVE, `NAME "red"];
	  active_cno <- (svid, cno_label)::active_cno;
	  active_flags <- (svid, flags_eb)::active_flags;
	  table#set_rows (table#rows +1)
      end
    else begin
	let cno_label = List.assoc svid active_cno in
	let flags_eb = List.assoc svid active_flags in
	  cno_label#set_label (cno^" dB Hz");
	   if flags="1" 
	  then
	    flags_eb#coerce#misc#modify_bg [`NORMAL, `NAME "green";`ACTIVE, `NAME "green"]
	  else
	    flags_eb#coerce#misc#modify_bg [`NORMAL, `NAME "red";`ACTIVE, `NAME "red"]
      end
end

(*****************************************************************************)
(* pfd page                                                                  *)
(*****************************************************************************)
class pfd (widget: GBin.frame) =
  let vbox = GPack.vbox ~packing: widget#add () in
  let attitude_table = GPack.table 
    ~rows: 2
    ~columns: 4
    ~row_spacings: 5
    ~col_spacings: 40
    ~packing: vbox#pack
    ()
  in
  let horizon_f = 
    GBin.frame ~packing: vbox#pack () in
  let horizon = new Horizon.h ~packing: horizon_f#add 300 in
  let roll = GMisc.label ~packing:(attitude_table#attach ~top:0 ~left:1) () in
  let pitch= GMisc.label ~packing:(attitude_table#attach ~top:0 ~left:3) () in
  let alt  = GMisc.label ~packing:(attitude_table#attach ~top:1 ~left:1) () in
  let climb= GMisc.label ~packing:(attitude_table#attach ~top:1 ~left:3) () in
  let _labels = 
    ignore (GMisc.label 
      ~text: "roll" ~packing:(attitude_table#attach ~top:0 ~left:0) ());
    ignore (GMisc.label 
      ~text: "pitch" ~packing:(attitude_table#attach ~top:0 ~left:2) ());
    ignore (GMisc.label 
      ~text: "alt" ~packing:(attitude_table#attach ~top:1 ~left:0) ());
    ignore (GMisc.label 
      ~text: "climb" ~packing:(attitude_table#attach ~top:1 ~left:2) ());
  in
    
object (this)
  val parent = widget
  val attitude_table = attitude_table
  val roll_l = roll
  val pitch_l = pitch
  val alt_l = alt
  val climb_l = climb
  val horizon = horizon
  val mutable pitch = 0.
  val mutable roll = 0.

  method set_roll r = roll_l#set_label (Printf.sprintf "%.1f" r);
    roll <- r;
    horizon#draw ((Deg>>Rad)roll) ((Deg>>Rad)pitch)
  method set_pitch p = pitch_l#set_label (Printf.sprintf "%.1f" p);
    pitch <- p;
    horizon#draw ((Deg>>Rad)roll) ((Deg>>Rad)pitch)
  method set_alt a = alt_l#set_label (Printf.sprintf "%.1f" a)
  method set_climb c = climb_l#set_label (Printf.sprintf "%.1f" c)
end
