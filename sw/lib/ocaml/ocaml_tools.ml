(*
 * $Id$
 *
 * Utilities
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

(* Versions : *)
(* 13/10/2004 : v2.01 : Fonction donnant le nombre de jours entre deux dates     *)
(* 23/09/2004 : v2.0 : Ajout de .bz2 et .gz aux extensions pour do_compress_file *)
(*              Fonction de tirage aleatoire d'un element d'une liste            *)
(* 13/01/2004 : v1.9 split_multiple2, do_read_file2,...                          *)
(* 12/09/2003 : v1.8 Ajout du support des fichier .zip                           *)
(* 25/06/2003 : v1.7 string_dos2unix                                             *)
(* 18/06/2003 : v1.6 split2                                                      *)
(* 28/04/2003 : v1.5 Ajout des fonctions du module Timer                         *)
(* 17/04/2003 : v1.4 ajour de cmp_float                                          *)
(* 24/03/2003 : v1.3 liste des fichiers/des repertoires dans un repertoire       *)
(*              Ajout de la doc                                                  *)
(* 05/03/2003 : parsing d'une chaine de caracteres                               *)
(* 10/02/2003 : v1.2 test d'existence d'un fichier avant sa lecture              *)
(* 17/01/2003 : v1.1 string_match_in, string_match_in_no_case                    *)
(*                   string_match_no_case, string_exact_match_no_case            *)
(* 14/01/2003 : v1.0 Debut versionnement                                         *)

let ocaml_tools_version = "2.01"

(* ============================================================================= *)
(* = Fonction de comparaison de deux entiers                                   = *)
(* ============================================================================= *)
let cmp_int a b = a-b

(* ============================================================================= *)
(* = Fonction de comparaison de deux flottants                                 = *)
(* ============================================================================= *)
let cmp_float d1 d2 = if d1<d2 then (-1) else if d1>d2 then 1 else 0

(* ============================================================================= *)
(* = Compare deux chaines                                                      = *)
(* ============================================================================= *)
let cmp_string s1 s2 = if s1<s2 then -1 else if s1>s2 then 1 else 0

(* ============================================================================= *)
(* = Decoupage d'une chaine de caracteres                                      = *)
(* ============================================================================= *)
let split c s =
  let i = ref (String.length s - 1) in
  let j = ref !i and r = ref [] in 
  if !i >= 0 & String.get s 0 <> '#'        (* skip lines starting with '#' *)
  then 
    while !i >= 0 do
      while !i >= 0 & String.get s !i <> c do decr i; done; 
      if !i < !j then r := (String.sub s (!i+1) (!j - !i)) :: !r;
      while !i >= 0 & String.get s !i = c do decr i; done;
      j := !i;
    done;
  !r

(* ============================================================================= *)
(* = Decoupage d'une chaine de caracteres qui s'arrete apres une occurence du  = *)
(* = caractere de decoupage contrairement a la fonction precedente             = *)
(* ============================================================================= *)
let split2 c s =
  let i = ref (String.length s - 1) in
  let j = ref !i and r = ref [] in 
  if !i >= 0 & String.get s 0 <> '#' then (* skip lines starting with '#' *)
    while !i >= 0 do
	  while !i >= 0 & String.get s !i <> c do decr i; done; 
	  if !i <= !j then r := (String.sub s (!i+1) (!j - !i)) :: !r;
	  decr i ;
	  j := !i;
    done;
  !r

(* ============================================================================= *)
(* = Decoupage d'une chaine de caracteres (separateurs multiples)              = *)
(* ============================================================================= *)
let split_multiple lst_c s =
  let match_char c = List.mem c lst_c in

  let i = ref (String.length s - 1) in
  let j = ref !i and r = ref [] in 
  if !i >= 0 & String.get s 0 <> '#' then   (* skip lines starting with '#' *)
    while !i >= 0 do
      while !i >= 0 & (not (match_char (String.get s !i))) do decr i; done; 
      if !i < !j then r := (String.sub s (!i+1) (!j - !i)) :: !r;
      while !i >= 0 & (match_char (String.get s !i)) do decr i; done;
      j := !i;
    done;
  !r

(* ============================================================================= *)
(* = Decoupage d'une chaine de caracteres (separateurs multiples).             = *)
(* = On s'arrete apres chaque occurence d'un separateur comme split2           = *)
(* ============================================================================= *)
let split_multiple2 lst_c s =
  let match_char c = List.mem c lst_c in

  let i = ref (String.length s - 1) in
  let j = ref !i and r = ref [] in 
  if !i >= 0 & String.get s 0 <> '#' then (* skip lines starting with '#' *)
    while !i >= 0 do
	  while !i >= 0 & (not (match_char (String.get s !i))) do decr i; done; 
	  if !i <= !j then r := (String.sub s (!i+1) (!j - !i)) :: !r;
	  decr i ;
	  j := !i;
    done;
  !r

(* ============================================================================= *)
(* = Fonction d'ajout d'espaces a une chaine de caracteres                     = *)
(* = lg = longueur desiree                                                     = *)
(* ============================================================================= *)
let rec add_spaces lg str =
  if (String.length str) >= lg then str else add_spaces lg (str ^ " ")

(* ============================================================================= *)
(* = Fonction de suppression d'espaces en fin de chaine, pour gagner un peu en = *)
(* = memoire                                                                   = *)
(* ============================================================================= *)
let delete_trailing_spaces s =
  if s <> "" then begin
	let i = ref ((String.length s)-1) in
	while !i>=0 && (String.get s !i = ' ') do decr i done ;
	if !i>=0 then String.sub s 0 (!i+1) else ""
  end else s

(* ============================================================================= *)
(* = Remplace le caractere c par c2 dans la chaine s                           = *)
(* ============================================================================= *)
let string_replace_char s c c2 =
  let i = ref 0 in
  String.iter (fun ch -> if ch=c then String.set s !i c2 ; incr i) s

(* ============================================================================= *)
(* = Supprime les eventuels CTRL-M en fin de chaine (DOS->Unix)                = *)
(* ============================================================================= *)
let string_dos2unix s =
  if s<>"" && String.get s (String.length s -1) = '
' then
	String.sub s 0 (String.length s -1)
  else s

(* ============================================================================= *)
(* = Teste si la chaine matche la pattern                                      = *)
(* ============================================================================= *)
let string_match pattern string =
  let l = String.length pattern in
  String.length string >= l && String.sub string 0 l = pattern
let string_exact_match pattern string = pattern = string

(* Meme chose sans tenir compte de la case des caracteres *)
let string_match_no_case pattern string =
  string_match (String.uppercase pattern) (String.uppercase string)

let string_exact_match_no_case pattern string =
  string_exact_match (String.uppercase pattern) (String.uppercase string)

let string_match_in pattern string =
  if pattern = "" then true else begin
	let do_match = ref false and substr = ref string in
	while not !do_match && String.length !substr>0  do
	  do_match := string_match pattern !substr ;
	  substr:=String.sub !substr 1 (String.length !substr -1)
	done ;
	!do_match
  end

let string_match_in_no_case pattern string =
  string_match_in (String.uppercase pattern) (String.uppercase string)

(* ============================================================================= *)
(* = Liste de chaines -> une chaine                                            = *)
(* ============================================================================= *)
let string_of_string_list lst_string =
  List.fold_left (fun str s -> if str <> "" then str^" "^s else s) "" lst_string

(* ============================================================================= *)
(* = Creation d'une chaine en fonction de la valeur d'un entier                = *)
(* =                                                                           = *)
(* = XXX n -> aucun(e) XXX si n=0, un(e) XXX si n=1, XXXs sinon                = *)
(* = female indique si on a un nom feminin et first_char_upper indique si le   = *)
(* = premier caractere doit etre en majuscule ou pas                           = *)
(* ============================================================================= *)
let eval_string base_string n female first_char_upper =
  let get_upper_and_female str =
	let str = if female then str^"e" else str in
	if first_char_upper then String.capitalize str
	else str
  in

  match n with
	0 -> (get_upper_and_female "aucun") ^ " " ^ base_string
  | 1 -> (get_upper_and_female "un") ^ " " ^ base_string
  | _ ->
	  if String.get base_string (String.length base_string -1) = 'x' then
		Printf.sprintf "%d %s" n base_string
	  else Printf.sprintf "%d %ss" n base_string

(* ============================================================================= *)
(* = Fonction de parsing d'une chaine                                          = *)
(* ============================================================================= *)
let do_parse_string s no_ligne_parsing parser_main lexer_token end_func =
  let lexbuf = Lexing.from_string s in
  let fin = ref false in
  no_ligne_parsing := 1 ;
  while not !fin do
	try parser_main lexer_token lexbuf ;
	with Parsing.Parse_error -> (* Erreur de syntaxe *)
	  raise Parsing.Parse_error
	| Failure("lexing: empty token") -> (* Est-ce la fin de la chaine ? *)
		fin := lexbuf.Lexing.lex_eof_reached ;
	| x -> raise x
  done ;
  (* Appel a la fonction de fin de lecture *)
  end_func ()

(* ============================================================================= *)
(* = Supprime les doublons dans une liste triee                                = *)
(* ============================================================================= *)
let rec supprime_dbl_list = function
    [] -> []
  | x::xs ->
      match xs with
	[] -> [x]
      | x'::xs' -> 
	  if x = x' then supprime_dbl_list xs' else x::supprime_dbl_list xs'

(* ============================================================================= *)
(* = Supprime l'element d'index idx dans la liste lst                          = *)
(* ============================================================================= *)
let del_elt_lst lst idx =
  if (idx>=0) && (idx<List.length lst) then	begin
	if (List.length lst) > 1 then begin
	  let a = Array.of_list lst in
	  let a1 = Array.sub a 0 idx and
		  a2 = Array.sub a (idx+1) ((List.length lst)-1-idx) in
	  Array.to_list (Array.append a1 a2)
	end else []
  end else lst

(* ============================================================================= *)
(* = Ouverture d'un fichier compresse avec gzip, bzip2, zip ou non compresse   = *)
(* ============================================================================= *)
let open_compress file = 
  if Filename.check_suffix file "gz" or Filename.check_suffix file "Z" or
	Filename.check_suffix file "zip" or Filename.check_suffix file "ZIP" then
    Unix.open_process_in ("gunzip -c "^file)
  else if Filename.check_suffix file "bz2"  then
    Unix.open_process_in ("bunzip2 -c "^file)
  else Pervasives.open_in file


let extensions = ["";".gz";".Z";".bz2";".zip";".ZIP"]
let find_file = fun path file ->
  let rec loop_path = function
      [] -> raise Not_found
    | p::ps ->
	let rec loop_ext = function
	    [] -> loop_path ps
	  | ext::es ->
	      let f = Filename.concat p file ^ ext in
	      if Sys.file_exists f then f else loop_ext es in
	loop_ext extensions in
  loop_path path

(* ============================================================================= *)
(* = Fermeture d'un fichier                                                    = *)
(* ============================================================================= *)
let close_compress file inchan =
  if (Filename.check_suffix file "gz") or (Filename.check_suffix file "bz2") or
	(Filename.check_suffix file "Z") or (Filename.check_suffix file "zip") or
	(Filename.check_suffix file) "ZIP"then
    ignore(Unix.close_process_in inchan)
  else close_in inchan

(* ============================================================================= *)
(* = Gestion des fichiers gzippes                                              = *)
(* ============================================================================= *)
let open_gzip file = 
  if Filename.check_suffix file "gz" or Filename.check_suffix file "Z" then
    Unix.open_process_in ("gunzip -c "^file) 
  else Pervasives.open_in file
let close_gzip file inchan =
  if Filename.check_suffix file "gz" or Filename.check_suffix file "Z" then
	ignore(Unix.close_process_in inchan)
  else close_in inchan
(* ============================================================================= *)

(* ============================================================================= *)
(* = Gestion des fichiers bzippes                                              = *)
(* ============================================================================= *)
let open_bzip file = 
  if Filename.check_suffix file "bz2"  then
	Unix.open_process_in ("bunzip2 -c "^file) 
  else Pervasives.open_in file
let close_bzip file inchan =
  if Filename.check_suffix file "bz2"  then ignore(Unix.close_process_in inchan)
  else close_in inchan
(* ============================================================================= *)

(* ============================================================================= *)
(* = Gestion des fichiers zippes                                               = *)
(* ============================================================================= *)
let open_zip file = 
  if Filename.check_suffix file "zip" or Filename.check_suffix file "ZIP" then
    Unix.open_process_in ("gunzip -c "^file) 
  else Pervasives.open_in file
let close_zip file inchan =
  if Filename.check_suffix file "zip" or Filename.check_suffix file "ZIP" then
	ignore(Unix.close_process_in inchan)
  else close_in inchan
(* ============================================================================= *)

(* ============================================================================= *)
(* = Compression d'un fichier suivant son extension                            = *)
(* ============================================================================= *)
let do_compress_file filename ext =
  ignore(match ext with
	"gz"  | ".gz"  -> Unix.system ("gzip "^filename)
  | "bz2" | ".bz2" -> Unix.system ("bzip2 "^filename)
  | _     -> Unix.WEXITED(0))

(* ============================================================================= *)
(* = Lecture d'un fichier et copie dans une chaine de caracteres               = *)
(* ============================================================================= *)
let string_of_file filename =
  let c = open_compress filename and texte = ref "" in
  (try
	while true do
	  if !texte = "" then texte := input_line c
	  else texte := !texte ^ "\n" ^ (input_line c)
	done
  with End_of_file -> close_compress filename c) ;
  !texte

(* ============================================================================= *)
(* = Fonction de lecture d'un fichier avec precision du separateur             = *)
(* ============================================================================= *)
let do_read_file_with_separators filename match_func end_func separators =
  let no_line = ref 0 in
  let error_func s =
	Printf.printf("Erreur ligne %d (%s)\n") !no_line s ; flush stdout in

  try
	let c = open_compress filename in
	let error = ref false in
	(try
	  while not !error do
		let s = input_line c in
		no_line := !no_line + 1 ;
		(* Passe les commentaires et decoupe la ligne *)
		let splitted = split_multiple separators s in
		(* On saute les lignes vides *)
		if splitted <> [] then
		  (try match_func splitted (fun () -> error_func s)
		  with | _ ->
			(* S'il y a une erreur ici, c'est qu'une ligne ne contient *)
			(* Pas ce qui est attendu : ex pb de int_of_string *)
			close_compress filename c ;
			error_func s ;error := true)
	  done
	with End_of_file -> close_compress filename c ; end_func ())
  with _ -> Printf.printf("Erreur d'ouverture du fichier %s\n") filename;
	flush stdout

(* ============================================================================= *)
(* = Fonction de lecture d'un fichier avec precision du separateur             = *)
(* ============================================================================= *)
let do_read_file_with_separator filename match_func end_func separator =
  do_read_file_with_separators filename match_func end_func [separator]

(* ============================================================================= *)
(* = Fonction de lecture d'un fichier (separateur par defaut = espace)         = *)
(* ============================================================================= *)
let do_read_file filename match_func end_func =
  do_read_file_with_separators filename match_func end_func [' ']

(* ============================================================================= *)
(* = Fonction de lecture d'un fichier avec precision du separateur             = *)
(* ============================================================================= *)
let do_read_file_with_separators2 filename match_func end_func separators =
  let no_line = ref 0 in
  let error_func s =
	Printf.printf("Erreur ligne %d (%s)\n") !no_line s ; flush stdout in

  try
	let c = open_compress filename in
	let error = ref false in
	(try
	  while not !error do
		let s = input_line c in
		no_line := !no_line + 1 ;
		(* Passe les commentaires et decoupe la ligne *)
		let splitted = split_multiple2 separators s in
		(* On saute les lignes vides *)
		if splitted <> [] then
		  (try match_func splitted (fun () -> error_func s)
		  with | _ ->
			(* S'il y a une erreur ici, c'est qu'une ligne ne contient *)
			(* Pas ce qui est attendu : ex pb de int_of_string *)
			close_compress filename c ;
			error_func s ;error := true)
	  done
	with End_of_file -> close_compress filename c ; end_func ())
  with _ -> Printf.printf("Erreur d'ouverture du fichier %s\n") filename;
	flush stdout

(* ============================================================================= *)
(* = Fonction de lecture d'un fichier avec precision du separateur             = *)
(* ============================================================================= *)
let do_read_file_with_separator2 filename match_func end_func separator =
  do_read_file_with_separators2 filename match_func end_func [separator]

(* ============================================================================= *)
(* = Fonction de lecture d'un fichier (separateur par defaut = espace)         = *)
(* ============================================================================= *)
let do_read_file2 filename match_func end_func =
  do_read_file_with_separators2 filename match_func end_func [' ']

(* ============================================================================= *)
(* = Fonction de parsing d'un fichier                                          = *)
(* ============================================================================= *)
let do_parse_file filename no_ligne_parsing parser_main lexer_token end_func =
  let c =
  (try Some (open_compress filename)
  with _ ->	Printf.printf "Erreur d'ouverture du fichier %s\n" filename;
	flush stdout; None) in

  match c with
	None -> ()
  | Some c ->
	  let lexbuf = Lexing.from_channel c in
	  let fin = ref false in
	  no_ligne_parsing := 1 ;
	  while not !fin do
		try parser_main lexer_token lexbuf ;
		with Parsing.Parse_error -> (* Erreur de syntaxe *)
		  Printf.printf "Erreur ligne %d : *%s*\n"
			!no_ligne_parsing (Lexing.lexeme lexbuf); flush stdout
		| Failure("lexing: empty token") -> (* Est-ce la fin du fichier ? *)
			fin := lexbuf.Lexing.lex_eof_reached ;
		| x -> raise x
	  done ;
	  close_compress filename c ;
	  (* Appel a la fonction de fin de lecture *)
	  end_func ()

(* ============================================================================= *)
(* = Parsing d'un fichier de configuration                                     = *)
(* ============================================================================= *)
let parse_config_file config_file spec_list anofun usage_msg =
 let c = open_compress config_file in
 (try
   while true do
     let s = input_line c in
     let l = (split ' ' s) in
     if l<>[] then begin
       Arg.current := 0;
       Arg.parse_argv (Array.of_list ("CMDE"::l)) spec_list anofun usage_msg;
     end;
   done;
 with End_of_file -> ());
 Arg.current := 0; close_compress config_file c

(* ============================================================================= *)
(* = Indique si le nom indique correspond a un repertoire                      = *)
(* ============================================================================= *)
let is_directory filename =
  let stats = Unix.stat filename in stats.Unix.st_kind = Unix.S_DIR

(* ============================================================================= *)
(* = Renvoie la liste des fichiers et repertoires contenus dans un repertoire  = *)
(* ============================================================================= *)
let get_files_from_dir dirname =
  let lst = ref [] in
  try
	let d = Unix.opendir dirname in
	try
	  while true do	let filename = Unix.readdir d in lst := filename::!lst done ;
	  []
	with End_of_file -> (* Lecture terminee *)
	  List.fast_sort cmp_string !lst
  with _ -> []

(* ============================================================================= *)
(* = Renvoie la liste des repertoires                                          = *)
(* ============================================================================= *)
let get_dirs_only_from_dir dirname =
  let l = get_files_from_dir dirname in
  List.fold_right (fun file l ->
	if (file<>"."&&file<>".."&&(is_directory (dirname^file)))
	then file::l else l) l []

(* ============================================================================= *)
(* = Renvoie la liste des fichiers                                             = *)
(* ============================================================================= *)
let get_files_only_from_dir dirname =
  let l = get_files_from_dir dirname in
  List.fold_right (fun file l ->
	if not (is_directory (dirname^file)) then file::l else l) l []

(* ============================================================================= *)
(* = Supprime l'eventuel chemin dans un nom de fichier                         = *)
(* ============================================================================= *)
let del_path_in_filename filename = Filename.basename filename


(* ***************************************************************************** *)
(* ***************************************************************************** *)
(*                      Manipulations de dates                                   *)
(* ***************************************************************************** *)
(* ***************************************************************************** *)

(* ============================================================================= *)
(* = Date sous la forme 20020114 -> 14 01 2002                                 = *)
(* ============================================================================= *)
let decompose_date date = (date mod 100, (date mod 10000)/100, date/10000)

(* ============================================================================= *)
(* = Date sous la forme 14 01 2002 -> 20020114                                 = *)
(* ============================================================================= *)
let compose_date (jj, mm, aa) = aa*10000+mm*100+jj

(* ============================================================================= *)
(* = Renvoie le nom du mois en fonction de son numero                          = *)
(* ============================================================================= *)
let get_month_of_num num =
  match num with
	1  -> "Janvier"   | 2  -> "Fevrier"  | 3  -> "Mars"     | 4  -> "Avril"
  | 5  -> "Mai"       | 6  -> "Juin"     | 7  -> "Juillet"  | 8  -> "Aout"
  | 9  -> "Septembre" | 10 -> "Octobre"  | 11 -> "Novembre" | 12 -> "Decembre"
  | _  -> "???"

(* ============================================================================= *)
(* = Renvoie le numero du jour de la semaine en fonction d'une date            = *)
(* ============================================================================= *)
let get_day_of_date (jj, mm, aa) =
  let jour_sem = [|"Dimanche"; "Lundi"; "Mardi"; "Mercredi"; "Jeudi";
				   "Vendredi"; "Samedi"|] in

  let (aa, mm) = if mm < 3 then (aa-1, mm+10) else (aa, mm-2) in
  let siecle = aa/100 and an = aa mod 100 in
  let js = (((26 * mm - 2) / 10) + jj + an + (an / 4) +
			  (siecle / 4) - (2 * siecle)) mod 7 in
  if js<0 then jour_sem.(js+7) else jour_sem.(js)

let get_day_of_date2 date = get_day_of_date (decompose_date date)

(* ============================================================================= *)
(* = L'annee indiquee est-elle bissextile ?                                    = *)
(* = Elle l'est si elle est divisible par 4, sauf si c'est un siecle. Cependant= *)
(* = tous les 4 siecles, elle est bissextile...                                = *)
(* ============================================================================= *)
let is_year_bis aa = (aa mod 400 = 0) || ((aa mod 4 = 0) && (aa mod 100 <> 0))

(* ============================================================================= *)
(* = Renvoie le nombre de jours d'un mois en fonction du mois et de l'annee    = *)
(* ============================================================================= *)
let get_nb_days_in_month mm aa =
  if mm = 2 then (* Annee bissextile ? *) if is_year_bis aa then 29 else 28
  else match mm with
	1 -> 31 | 2 -> 28 | 3 -> 31 | 4  -> 30 | 5  -> 31 | 6  -> 30
  | 7 -> 31 | 8 -> 31 | 9 -> 30 | 10 -> 31 | 11 -> 30 | 12 -> 31
  | _ -> (-1)

(* ============================================================================= *)
(* = Renvoie la date augmentee de delta jours (delta peut etre negatif)        = *)
(* ============================================================================= *)
let get_delta_date (jj, mm, aa) delta =
  let d = ref delta and j = ref jj and m = ref mm and a = ref aa in
  let adding = delta>=0 in
  while !d <> 0 do
	let lg_month = get_nb_days_in_month !m !a in
	if !d+ !j<=lg_month && !d+ !j>0 then begin j:=!j+ !d; d:=0 end else begin
	  if adding then begin
		if !m<12 then incr m else begin m:=1; incr a end ;
		d:=!d-(lg_month- !j)-1 ;
		j:=1
	  end else begin
		if !m>1 then decr m else begin m:=12; decr a end ;
		d:=!d+ !j ;
		j:=get_nb_days_in_month !m !a
	  end ;
	end
  done ;
  (!j, !m, !a)

let get_delta_date2 date delta =
  compose_date (get_delta_date (decompose_date date) delta)

(* ============================================================================= *)
(* = Renvoie la date precedente/suivante                                       = *)
(* ============================================================================= *)
let get_next_date (jj, mm, aa) = get_delta_date (jj, mm, aa) 1
let get_next_date2 date        = get_delta_date2 date 1
let get_prev_date (jj, mm, aa) = get_delta_date (jj, mm, aa) (-1)
let get_prev_date2 date        = get_delta_date2 date (-1)

(* ============================================================================= *)
(* = Fonction renvoyant le nombre de jours entre deux dates d2-d1              = *)
(* ============================================================================= *)
let get_diff_date (jj1, mm1, aa1) (jj2, mm2, aa2) =
  (* On se debarrasse du cas trivial *)
  if mm1=mm2 && aa1=aa2 then jj2-jj1 else begin
	(* Sens dans lequel on se deplace *)
	let delta =
	  if compose_date (jj1, mm1, aa1)<compose_date (jj2, mm2, aa2) then 1
	  else (-1)
	in

	let current_date=ref (jj1, mm1, aa1) and nb_days=ref 0 in
	while !current_date<>(jj2, mm2, aa2) do
	  nb_days:= !nb_days+delta ;
	  current_date:=get_delta_date !current_date delta
	done ;
	!nb_days
  end

let get_diff_date2 d1 d2 = get_diff_date (decompose_date d1) (decompose_date d2)

(* ============================================================================= *)
(* = Date -> Chaine JJ/MM/AAAA                                                 = *)
(* ============================================================================= *)
let string_of_date (jj, mm, aa) = Printf.sprintf "%02d/%02d/%d" jj mm aa
let string_of_date2 date = string_of_date (decompose_date date)

(* ============================================================================= *)
(* = Temps en secondes -> Chaine                                               = *)
(* ============================================================================= *)
let string_of_time s =
  Printf.sprintf "%02d:%02d:%02d" (s/3600) (s/60 mod 60) (s mod 60)

(* ============================================================================= *)
(* = Temps en secondes -> Chaine sans les secondes                             = *)
(* ============================================================================= *)
let string_of_time_without_seconds s =
  Printf.sprintf "%02d:%02d" (s/3600) (s/60 mod 60)

(* ============================================================================= *)
(* = Chaine -> Temps en secondes                                               = *)
(* ============================================================================= *)
let time_of_string t =
  match split ':' t with
    [h;m;s]->(int_of_string h*60 + int_of_string m)*60 + int_of_string s |_->0

(* ============================================================================= *)
(* = Renvoie l'heure                                                           = *)
(* ============================================================================= *)
let timer_get_time () = Unix.localtime (Unix.time ())

(* ============================================================================= *)
(* = Formate l'heure dans une chaine de caracteres                             = *)
(* ============================================================================= *)
let timer_string_of_time tm =
  Printf.sprintf "%02d:%02d:%02d" tm.Unix.tm_hour tm.Unix.tm_min tm.Unix.tm_sec

(* ============================================================================= *)
(* = Formate la date dans une chaine de caracteres                             = *)
(* ============================================================================= *)
let timer_string_of_date tm =
  Printf.sprintf "%02d/%02d/%04d" tm.Unix.tm_mday (tm.Unix.tm_mon+1)
	(tm.Unix.tm_year+1900)

(* ============================================================================= *)
(* = Renvoie le temps ecoule entre deux heures, en secondes                    = *)
(* ============================================================================= *)
let timer_sub tm1 tm2 =
  let (h1, m1, s1, d1) =
	(tm1.Unix.tm_hour,tm1.Unix.tm_min,tm1.Unix.tm_sec, tm1.Unix.tm_mday)
  and (h2, m2, s2, d2) =
	(tm2.Unix.tm_hour,tm2.Unix.tm_min,tm2.Unix.tm_sec, tm2.Unix.tm_mday) in
  (d2-d1)*24*3600+(h2-h1)*3600+(m2-m1)*60+s2-s1

(* ============================================================================= *)
(* = Chaine en Heures, Minutes, Secondes indiquant le temps ecoule             = *)
(* ============================================================================= *)
let timer_string_of_secondes sec =
  let h = sec / 3600 in
  let reste = sec-h*3600 in
  let m = reste/60 and s = reste mod 60 in
  Printf.sprintf "%d:%02d:%02d" h m s

(* ============================================================================= *)
(* = Tirage aleatoire d'un element dans une liste                              = *)
(* ============================================================================= *)
let tirage_aleatoire_lst (lst:'a list) = List.nth lst (Random.int (List.length lst))

(* =============================== FIN ========================================= *)
