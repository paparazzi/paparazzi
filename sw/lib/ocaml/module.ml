(*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

(**
 * 'Module' module for parsing XML config files
 *)

module OT = Ocaml_tools
module GC = Gen_common

let find_name = fun xml ->
  try
    let name = Xml.attrib xml "name" in
    if Filename.check_suffix name ".xml" then Filename.chop_extension name
    else name
  with
  | Not_found ->
    let msg = Printf.sprintf "Error: Attribute 'name' expected in %a"
        ExtXml.sprint_fields (Xml.attribs xml) in
    raise (ExtXml.Error msg)

type file = { filename: string; directory: string option; filecond: string option }
type file_arch = file

let parse_file = fun xml ->
  match xml with
  | Xml.Element ("file", _, []) | Xml.Element ("file_arch", _, []) ->
      { filename = find_name xml;
        directory = ExtXml.attrib_opt xml "dir";
        filecond = ExtXml.attrib_opt xml "cond" }
  | _ -> failwith "Module.parse_file: unreachable"

type configure = {
    cname: string;
    cvalue: string option;
    case: string option;
    default: string option;
    cdescription: string option
  }

let parse_configure = fun xml ->
  let get = fun x -> ExtXml.attrib_opt xml x in
  { cname = find_name xml; cvalue = get "value"; case = get "case";
    default = get "default"; cdescription = get "description" }

type define = {
    dname: string;
    dvalue: string option;
    integer: int option;
    dunit: string option;
    dtype: string option;
    ddescription: string option;
    cond: string option
  }

let parse_define = fun xml ->
  let get = fun x -> ExtXml.attrib_opt xml x in
  { dname = find_name xml; dvalue = get "value";
    integer = begin match get "integer" with
      | None -> None | Some i -> Some (int_of_string i) end;
    dunit = get "unit"; dtype = get "type";
    ddescription = get "description"; cond = get "cond" }

type incl = { element: string; condition: string option }

type flag = { flag: string; value: string; fcond: string option }

type raw = string

type makefile = {
    targets: string option;
    firmware: string option;
    condition: string option;
    configures: configure list;
    defines: define list;
    inclusions: incl list;
    flags: flag list;
    files: file list;
    files_arch: file list;
    raws: raw list
  }

let empty_makefile =
  { targets = None; firmware = None; condition = None;
    configures = []; defines = [];
    inclusions = []; flags = []; files = []; files_arch = []; raws = [] }

let rec parse_makefile mkf = function
  | Xml.Element ("makefile", _, children) as xml ->
    let targets = ExtXml.attrib_opt xml "target"
    and firmware = ExtXml.attrib_opt xml "firmware"
    and condition = ExtXml.attrib_opt xml "cond" in
    List.fold_left parse_makefile
      { mkf with targets; firmware; condition } children
  | Xml.Element ("configure", _, []) as xml ->
    { mkf with configures = parse_configure xml :: mkf.configures }
  | Xml.Element ("define", _, []) as xml ->
    { mkf with defines = parse_define xml :: mkf.defines }
  | Xml.Element ("include", _, []) as xml ->
    { mkf with inclusions =
                 { element = find_name xml;
                   condition = ExtXml.attrib_opt xml "cond" }
                 :: mkf.inclusions }
  | Xml.Element ("flag", _, []) as xml ->
    let flag = Xml.attrib xml "name" and value = Xml.attrib xml "value" in
    { mkf with flags = { flag; value; fcond = ExtXml.attrib_opt xml "cond" }
    :: mkf.flags }
  | Xml.Element ("file", _, []) as xml ->
    { mkf with files = parse_file xml :: mkf.files }
  | Xml.Element ("file_arch", _, []) as xml ->
    { mkf with files_arch = parse_file xml :: mkf.files_arch }
  | Xml.Element ("raw", [], [Xml.PCData raw]) ->
    { mkf with raws = raw :: mkf.raws }
  | Xml.Element ("test", _, _) ->
    mkf
  | _ -> failwith "Module.parse_makefile: unreachable"

type autorun = True | False | Lock

type period_freq = Unset | Set of float * float | Freq of string | Period of string

type periodic = {
    call: string;
    fname: string;
    period_freq: period_freq;
    delay: float option;
    start: string option;
    stop: string option;
    autorun: autorun;
    cond: string option
  }

let parse_periodic = fun xml ->
  let get = fun x -> ExtXml.attrib_opt xml x in
  let getf = fun x ->  ExtXml.attrib_opt_float xml x in
  let call = snd (List.find (fun (a, _) -> String.lowercase_ascii a = "fun")
                 (Xml.attribs xml)) in
  let call_regexp = Str.regexp "\\([a-zA-Z_][a-zA-Z0-9_]*\\)\\(.*\\)" in
  let fname =
    if Str.string_match call_regexp call 0 then
      let fname = Str.matched_group 1 call and args = Str.matched_group 2 call in
      if args = "" || Str.string_match (Str.regexp "(.*)") args 0 then fname
      else failwith ("Module.parse_periodic: invalid function call: " ^ call)
    else failwith ("Module.parse_periodic: invalid function name: " ^ call) in
  let period_freq = match get "period", get "freq" with
    | None, None -> Unset
    | None, Some f -> begin
        try let f = float_of_string f in Set (1. /. f, f)
        with _ -> Freq f
      end
    | Some p, None -> begin 
        try let p = float_of_string p in Set (p, 1. /. p)
        with _ -> Period p
      end
    | Some p, Some _ -> begin
        Printf.eprintf "Warning: both period and freq are defined ";
        Printf.eprintf "but only period is used for function %s\n%!" fname;
        try let p = float_of_string p in Set (p, 1. /. p)
        with _ -> Period p
      end
   in
  { call; fname; period_freq; delay = getf "delay";
    start = get "start"; stop = get "stop"; cond = get "cond";
    autorun = match get "autorun" with
      | None -> Lock
      | Some "TRUE" | Some "true" -> True
      | Some "FALSE" | Some "false" -> False
      | Some "LOCK" | Some "lock" -> Lock
      | Some a -> failwith ("Module.parse_periodic: unknown autorun: " ^ a) }

type init = { iname: string; cond: string option }

let make_init = fun f cond ->
  { iname = f;
    cond = cond
  }

type event = { ev: string; cond: string option }

let make_event = fun f cond ->
  { ev = f;
    cond = cond
  }


type datalink = { message: string; func: string; dl_class: string option; cond: string option }

let make_datalink = fun f m cl cond ->
  { message = m;
    func = f;
    dl_class = cl;
    cond = cond
  }

let fprint_datalink = fun ch d ->
  Printf.fprintf ch "(msg_id == DL_%s) { %s; }\n" d.message d.func

type dependencies = {
    requires: GC.bool_expr list;
    conflicts: string list;
    provides: string list;
  }

(* comma separated values *)
let parse_comma_list = Str.split (Str.regexp "[ \t]*,[ \t]*")
(* comma separated functionalities (add '@' in front of functionality name) *)
let parse_func_list = fun l -> List.map (fun x -> "@"^x) (Str.split (Str.regexp "[ \t]*,[ \t]*") l)
(* pipe separated values *)
let parse_module_options = Str.split (Str.regexp "[ \t]*|[ \t]*")

let empty_dep = { requires = []; conflicts = []; provides = [] }

let rec parse_dependencies dep = function
  | Xml.Element ("dep", _, children) ->
      List.fold_left parse_dependencies dep children
  | Xml.Element ("depends", _, [Xml.PCData depends]) ->
    { dep with requires = List.map (fun x -> GC.bool_expr_of_string (Some x)) (parse_comma_list depends) }
  | Xml.Element ("conflicts", _, [Xml.PCData conflicts]) ->
    { dep with conflicts = parse_comma_list conflicts }
  | Xml.Element ("provides", _, [Xml.PCData provides]) ->
    { dep with provides = parse_func_list provides }
  | _ -> failwith "Module.parse_dependencies: unreachable"

type autoload = {
    aname: string;
    atype: string option
  }

type config = { name: string;
                mtype: string option;
                dir: string option;
                configures: configure list;
                defines: define list;
                xml: Xml.xml }

let config_from_xml = function
  | Xml.Element ("module", _, children) as xml ->
     { name = Xml.attrib xml "name";
       mtype = ExtXml.attrib_opt xml "type";
       dir = ExtXml.attrib_opt xml "dir";
       configures = ExtXml.parse_children "configure" parse_configure children;
       defines = ExtXml.parse_children "define" parse_define children;
       xml }
  | _ -> failwith "Module.config_from_xml: unreachable"

type t = {
  xml_filename: string;
  name: string;
  dir: string option;
  task: string option;
  path: string;
  doc: Xml.xml;
  dependencies: dependencies option;
  autoloads: autoload list;
  settings: Settings.t list;
  headers: file list;
  inits: init list;
  periodics: periodic list;
  events: event list;
  datalinks: datalink list;
  makefiles: makefile list;
  xml: Xml.xml
}

let empty =
  { xml_filename = ""; name = ""; dir = None;
    task = None; path = ""; doc = Xml.Element ("doc", [], []);
    dependencies = None; autoloads = []; settings = [];
    headers = []; inits = []; periodics = []; events = []; datalinks = [];
    makefiles = []; xml = Xml.Element ("module", [], []) }

let rec parse_xml m = function
  | Xml.Element ("module", _, children) as xml ->
    let name = find_name xml
    and dir = ExtXml.attrib_opt xml "dir"
    and task = ExtXml.attrib_opt xml "task" in
    List.fold_left parse_xml { m with name; dir; task; xml } children
  | Xml.Element ("doc", _, _) as xml ->
    { m with doc = xml }
  | Xml.Element ("settings", _, _) as xml ->
    { m with settings = Settings.from_xml xml :: m.settings }
  | Xml.Element ("dep", _, _) as xml ->
    { m with dependencies = Some (parse_dependencies empty_dep xml) }
  | Xml.Element ("autoload", _, []) as xml ->
    let aname = find_name xml
    and atype = ExtXml.attrib_opt xml "type" in
    { m with autoloads = { aname; atype } :: m.autoloads }
  | Xml.Element ("header", [], files) ->
    { m with headers =
               List.fold_left (fun acc f -> parse_file f :: acc) m.headers files
    }
  | Xml.Element ("init", _, []) as xml ->
    let f = Xml.attrib xml "fun"
    and c = ExtXml.attrib_opt xml "cond" in
    { m with inits = make_init f c :: m.inits }
  | Xml.Element ("periodic", _, []) as xml ->
    { m with periodics = parse_periodic xml :: m.periodics }
  | Xml.Element ("event", _, []) as xml ->
    let f = Xml.attrib xml "fun"
    and c = ExtXml.attrib_opt xml "cond" in
    { m with events = make_event f c :: m.events }
  | Xml.Element ("datalink", _, []) as xml ->
    let message = Xml.attrib xml "message"
    and func = Xml.attrib xml "fun"
    and dl_class = ExtXml.attrib_opt xml "class"
    and c = ExtXml.attrib_opt xml "cond" in
    { m with datalinks = make_datalink func  message dl_class c :: m.datalinks }
  | Xml.Element ("makefile", _, _) as xml ->
    { m with makefiles = parse_makefile empty_makefile xml :: m.makefiles }
  | _ -> failwith "Module.parse_xml: unreachable"

let from_xml = fun xml ->
  let m = parse_xml empty xml in
  { m with
    settings = List.rev m.settings;
    headers = List.rev m.headers;
    inits = List.rev m.inits;
    makefiles = List.rev m.makefiles
  }

let from_file = fun filename -> from_xml (Xml.parse_file filename)

(** search and parse a module xml file and return a Module.t *)
(* FIXME search folder path: <PPRZ_PATH>/*/<module_name[_type]>.xml *)
exception Module_not_found of string
let from_module_name = fun name mtype ->
  (* concat module type if needed *)
  let name = match mtype with Some t -> name ^ "_" ^ t | None -> name in
  (* determine if name already have an extension *)
  let name = if Filename.check_suffix name ".xml" then name else name ^ ".xml" in
  (* determine if name is implicit
   * if not, search for absolute name in search path
   * may raise Module_not_found if no file found *)
  let name =
    if Filename.is_implicit name then
      let rec find_abs = function
        | [] -> raise (Module_not_found name)
        | b :: bl ->
          let full_name = Filename.concat b name in
          if Sys.file_exists full_name then full_name else find_abs bl
      in find_abs Env.modules_paths
    else if Sys.file_exists name then name
    else raise (Module_not_found name)
  in
  let m = from_xml (ExtXml.parse_file name) in
  let settings = List.map (fun s -> { s with Settings.filename = name }) m.settings in
  { m with xml_filename = name; settings }

(** check if a makefile node is compatible with a target and a firmware
 * TODO add 'board' type filter ? *)
let check_mk = fun target firmware mk ->
  (mk.firmware = (Some firmware) || mk.firmware = None || firmware = "none") && GC.test_targets target (GC.bool_expr_of_string mk.targets)

(** check if a module is compatible with a target and a firmware *)
let check_loading = fun target firmware m ->
  List.exists (check_mk target firmware) m.makefiles

(* TODO  merge *)
let status_name = fun mod_name p -> mod_name ^ "_" ^ p.fname ^ "_status"

(* return a Settings object from modules *)
let get_sys_modules_settings = fun modules ->
  (* build a XML node corresponding to the settings *)
  let mod_settings = List.fold_left (fun lm m ->
    let periodic_settings = List.fold_left (fun lp p ->
      if not (p.autorun = Lock) then
        lp @ [Xml.Element("dl_setting",
                    [("min","2");
                    ("max","3");
                    ("step","1");
                    ("var", status_name m.name p);
                    ("shortname", p.fname);
                    ("values","START|STOP")],[])]
      else lp
    ) [] m.periodics in
    lm @ periodic_settings
  ) [] modules in
  let xml = Xml.Element("dl_settings",[("name","Modules")],mod_settings) in
  if List.length mod_settings > 0 then
    Some (Settings.from_xml (Xml.Element("settings",[],[xml])))
  else
    None

