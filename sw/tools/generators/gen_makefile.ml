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
 * Generate makefile from the aircraft configuration
 *)

open Printf
open Gen_common

module AC = Aircraft

(* TODO: add condition in xml syntax ? *)

let get_string_opt = fun x -> match x with Some s -> s | None -> ""

let (//) = Filename.concat

let configure2mk = fun ?(default_configure=false) f c ->
  (* all makefiles variables are forced to uppercase *)
  let name = Compat.uppercase_ascii c.Module.cname
  and value = get_string_opt c.Module.cvalue
  and default = get_string_opt c.Module.default
  and case = get_string_opt c.Module.case in
  (* either print the default or the normal configure variable *)
  if default_configure then begin
    (* Only print variable if default is set but not value *)
    if String.length default > 0 && String.length value = 0 then
      fprintf f "%s ?= %s\n" name default
  end else begin
    (* Only print variable if value is not empty *)
    if String.length value > 0 then
      fprintf f "%s = %s\n" name value;
    (* Or if only the name is given (unset a variable *)
    if String.length value = 0 && String.length default = 0 && String.length case = 0 then
      fprintf f "%s =\n" name
  end;
  (* also providing lower and upper case version on request *)
  if Str.string_match (Str.regexp ".*lower.*") case 0 then
    fprintf f "%s_LOWER = $(shell echo $(%s) | tr A-Z a-z)\n" name name;
  if Str.string_match (Str.regexp ".*upper.*") case 0 then
    fprintf f "%s_UPPER = $(shell echo $(%s) | tr a-z A-Z)\n" name name

let include2mk = fun f ?(target="$(TARGET)") ?(vpath=None) inc ->
  let name = inc.Module.element
  and path = match vpath with Some vp -> vp ^ "/" | None -> "" in
  let flag = sprintf "%s.CFLAGS += -I%s%s" target path name in
  match inc.Module.condition with
  | None -> fprintf f "%s\n" flag
  | Some cond -> fprintf f "%s\n%s\nendif\n" cond flag

let flag2mk = fun f ?(target="$(TARGET)") x ->
  let name = x.Module.flag
  and value = x.Module.value in
  let flag = sprintf "%s.%s += -%s" target name value in
  match x.Module.fcond with
  | None -> fprintf f "%s\n" flag
  | Some cond -> fprintf f "%s\n%s\nendif\n" cond flag

let define2mk = fun f ?(target="$(TARGET)") define ->
  let name = define.Module.dname
  and value = define.Module.dvalue in
  let flag_type = fun s ->
    match define.Module.dtype, value with
    | Some "string", Some v -> "=\\\""^v^"\\\""
    | _, Some v -> "="^v
    | _, _ -> ""
  in
  let flag = sprintf "%s.CFLAGS += -D%s%s" target name (flag_type value) in
  match define.Module.cond with
  | None -> fprintf f "%s\n" flag
  | Some cond -> fprintf f "%s\n%s\nendif\n" cond flag

let raw2mk = fun f raw ->
  fprintf f "%s\n" raw

let file2mk = fun f ?(arch = false) dir_name target file ->
  let name = file.Module.filename in
  let dir_name = match file.Module.directory with
    | Some "." -> ""
    | Some d -> d^"/"
    | None -> "$(" ^ dir_name ^ ")/" in
  let cond, cond_end = match file.Module.filecond with None -> "", ""
    | Some c -> "\n"^c^"\n", "\nendif" in
  let fmt =
    if arch then format_of_string "%s%s.srcs += arch/$(ARCH)/%s%s%s\n"
    else format_of_string "%s%s.srcs += %s%s%s\n" in
  fprintf f fmt cond target dir_name name cond_end

(* module files and flags except configuration flags *)
let module2mk = fun f target firmware m ->
  let name = m.Module.name in
  let dir = match m.Module.dir with Some d -> d | None -> name in
  let dir_name = Compat.uppercase_ascii dir ^ "_DIR" in
  (* iter makefile section *)
  List.iter (fun mk ->
    if Module.check_mk target firmware mk then begin
      begin match mk.Module.condition with Some c -> fprintf f "%s\n" c | None -> () end;
      List.iter (define2mk f ~target) mk.Module.defines;
      List.iter (include2mk f ~target (*FIXME vpath*)) mk.Module.inclusions;
      List.iter (flag2mk f ~target) (List.rev mk.Module.flags) (* reverse list to restore original order, it matters for flags *);
      List.iter (file2mk f dir_name target) mk.Module.files;
      List.iter (file2mk f ~arch:true dir_name target) mk.Module.files_arch;
      List.iter (raw2mk f) mk.Module.raws;
      match mk.Module.condition with Some _ -> fprintf f "endif\n" | None -> ()
    end
  ) m.Module.makefiles

let dump_target_conf = fun out target conf ->
  fprintf out "\n####################################################\n";
  fprintf out   "# makefile target '%s' for firmware '%s'\n" target conf.AC.firmware_name;
  fprintf out   "####################################################\n\n";
  fprintf out "ifeq ($(TARGET), %s)\n\n" target;
  let dir_list = singletonize (List.fold_left (fun l (_, m) -> match m.Module.dir with
    | None -> m.Module.name::l | Some d -> d::l) [] conf.AC.modules) in
  List.iter (fun d -> fprintf out "%s_DIR = modules/%s\n" (Compat.uppercase_ascii d) d) dir_list;
  List.iter (fun p ->
    fprintf out "VPATH += %s\n" p;
    fprintf out "$(TARGET).CFLAGS += -I%s/modules\n" p
  ) Env.modules_ext_paths;
  fprintf out "\n";
  if conf.AC.autopilot then
    fprintf out "USE_GENERATED_AUTOPILOT = TRUE\n";
  List.iter (configure2mk out) conf.AC.configures;
  fprintf out "\ninclude $(PAPARAZZI_SRC)/conf/boards/%s.makefile\n" conf.AC.board_type;
  fprintf out "include $(PAPARAZZI_SRC)/conf/firmwares/%s.makefile\n\n" conf.AC.firmware_name;
  List.iter (configure2mk ~default_configure:true out) conf.AC.configures_default;
  fprintf out "\n";
  List.iter (define2mk out) conf.AC.defines;
  fprintf out "\n";
  List.iter (fun (l, m) -> match l with
              | AC.UserLoad | AC.AutoLoad | AC.Depend -> module2mk out target conf.AC.firmware_name m
              | _ -> ()
  ) conf.AC.modules;
  fprintf out "\nendif # end of target '%s'\n\n" target
  

(** Generate makefile configuration files *)
let generate_makefile = fun ac_id confs makefile_out ->
  let out = open_out makefile_out in
  fprintf out "# This file has been generated by gen_aircraft\n";
  fprintf out "# Version %s\n" (Env.get_paparazzi_version ());
  fprintf out "# Please DO NOT EDIT\n";
  fprintf out "AC_ID=%s\n\n" ac_id;
  fprintf out "$(TARGET).CFLAGS += -Imodules -Iarch/$(ARCH)/modules\n";

  (** Iter on targets and dump makefile *)
  Hashtbl.iter (fun target c -> dump_target_conf out target c) confs;

  close_out out

