(******** Sectors **********************************************************)

open Printf

let (//) = Filename.concat

let rec display = fun (geomap:MapCanvas.widget) r ->

  match String.lowercase (Xml.tag r) with
      "disc" ->
        let rad = float_of_string (ExtXml.attrib r "radius")
        and geo = Latlong.of_string (ExtXml.attrib (ExtXml.child r "point") "pos") in
        ignore (geomap#circle ~width:5 ~color:"red" geo rad)
    | "union" ->
      List.iter (display geomap) (Xml.children r)
    | "polygon" ->
      let pts = List.map (fun x ->  Latlong.of_string (ExtXml.attrib x "pos")) (Xml.children r) in
      let pts = Array.of_list pts in
      let n = Array.length pts in
      for i = 0 to n - 1 do
        ignore (geomap#segment ~width:5 ~fill_color:"red" pts.(i) pts.((i+1)mod n))
      done
    |x -> fprintf stderr "Sector.display: '%s' not yet\n%!" x


let display_sector = fun (geomap:MapCanvas.widget) sector ->
  display geomap (ExtXml.child sector "0")


let load = fun geomap () ->
  match GToolbox.select_file ~title:"Load sectors" ~filename:(Env.flight_plans_path // "*.xml") () with
      None -> ()
    | Some f ->
      try
        let xml = Xml.parse_file f in
        List.iter (display_sector geomap) (Xml.children xml)
      with
          Dtd.Prove_error(e) ->
            let m = sprintf "Error while loading %s:\n%s" f (Dtd.prove_error e) in
            GToolbox.message_box "Error" m

