(* Ocamlbuild plugin *)
open Ocamlbuild_plugin

let gtk_cflags = "-pthread -I/usr/include/gtk-2.0 -I/usr/lib/x86_64-linux-gnu/gtk-2.0/include -I/usr/include/atk-1.0 -I/usr/include/cairo -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/pango-1.0 -I/usr/include/gio-unix-2.0/ -I/usr/include/freetype2 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/pixman-1 -I/usr/include/libpng12 -I/usr/include/harfbuzz"

let _ =
  dispatch & function
  | After_rules ->
    (* uncomment to compile any C files with -fPIC, should be default *)
    (*flag ["c"; "compile";] (S[A"-ccopt"; A"-fPIC"]);*)

    (* When one compiles C code using gtk *)
    flag ["c"; "compile"; "use_gtk"]
      (S[A"-ccopt"; A gtk_cflags]);

    (* link a simple dependency, e.g. single object file: linkdep(foo.o) *)
    pdep ["link"] "linkdep" (fun param -> [param]);

    (* generate mix C/Caml library *)
    pflag ["ocamlmklib"; "c"] "linkclib" (fun param -> (S[A("-l"^param)]));

    (* Generate and link a library:
     * e.g. linklib(foo.a) to build libfoo.a from libfoo.clib
     *)
    (* add it as a dependency so the lib gets built *)
    pdep ["link"] "linklib" (fun param -> ["lib"^param]);
    (* link the lib in bytecode mode *)
    pflag ["link";"ocaml";"byte"] "linklib" (fun param ->
      let libname = String.sub param 0 (String.length param - 2) in
      (S[A"-dllib"; A("-l"^libname); A"-cclib"; A("-l"^libname)]));
    (* link the lib in native mode *)
    pflag ["link";"ocaml";"native"] "linklib" (fun param ->
      let libname = String.sub param 0 (String.length param - 2) in
      (S[A"-cclib"; A("-l"^libname)]));

    (* If `static' tag is given, then every ocaml link in bytecode will add -custom *)
    flag ["link"; "ocaml"; "byte"; "static"] (A"-custom");

    (* possibility to add defines for camlp4 in _tags file, e.g. define(FOO) *)
    pflag ["ocaml";"compile";] "define" (fun s -> S [A"-ppopt"; A ("-D"^s)]);
    pflag ["ocaml";"ocamldep";] "define" (fun s -> S [A"-ppopt"; A ("-D"^s)])
  | _ -> ()
