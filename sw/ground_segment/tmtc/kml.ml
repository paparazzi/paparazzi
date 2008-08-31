open Latlong
open Printf

let el = fun t a c -> Xml.Element (t, a, c)
let data = fun t d -> el t [] [Xml.PCData d]

let kml = fun c -> el "kml" ["xmlns", "http://earth.google.com/kml/2.1"] c

let waypoints = Hashtbl.create 97

let coordinates = fun wgs84 a ->
  sprintf "%f,%f,%f" ((Rad>>Deg)wgs84.posn_long) ((Rad>>Deg)wgs84.posn_lat) a

let waypoint = fun utm0 alt0 wp ->
  let x = ExtXml.float_attrib wp "x"
  and y = ExtXml.float_attrib wp "y"
  and a = float_of_string (ExtXml.attrib_or_default wp "alt" alt0)
  and name = ExtXml.attrib wp "name" in
  let wgs84 = of_utm WGS84 (utm_add utm0 (x,y)) in
  el "Placemark" ["id", name]
    [data "name" name;
     data "styleUrl" "#msn_wp_icon";
     el "Point" []
       [data "extrude" "1";
	data "coordinates" (coordinates wgs84 a)]]

let icon_style = fun ?(heading=0) ?(color="ffffffff") id icon ->
  el "Style" ["id", id]
    [el "IconStyle" []
       [data "heading" (string_of_int heading);
	data "color" color;
	el "Icon" []
	  [data "href" (sprintf "http://maps.google.com/mapfiles/kml/%s" icon)]]]

let line_style = fun id ?(width = 2) color ->
  el "Style" ["id", id]
    [el "LineStyle" []
       [data "color" color;
	data "width" (string_of_int width)]]

let pair = fun key icon ->
  el "Pair" []
    [data "key" key;
     data "styleUrl" ("#"^icon)]


let stylemap = fun id normal_icon highlight_icon ->
  el "StyleMap" ["id", id]
    [pair "normal" normal_icon;
     pair "hightlight" highlight_icon]

let style =
  [ icon_style "ac_style" "pal2/icon56.png";
    icon_style ~color:"a00000ff" "wp_style" "shapes/triangle.png";
    stylemap "msn_ac_icon" "ac_style" "ac_style";
    stylemap "msn_wp_icon" "wp_style" "wp_style"]

let circle = fun utm0 radius alt ->
  let degree_point = fun d ->
    let a = (Deg>>Rad) (float d) in
    let utm = utm_add utm0 (cos a *. radius, sin a *. radius) in
    let wgs84 = of_utm WGS84 utm in
    coordinates wgs84 alt in
  let points = Array.init 360 degree_point in
  String.concat " " (points.(359) :: Array.to_list points)


let ring_around_home = fun utm0 fp ->
  let radius = ExtXml.float_attrib fp "max_dist_from_home"
  and alt =  ExtXml.float_attrib fp "alt" in
  let coords = circle utm0 radius alt in
  el "Placemark" ["id", "max_dist_from_home"]
    [data "name" "Max Distance";
     line_style "red" "800000ff";
     el "LinearRing" []
       [ data "extrude" "1";
	 data "altitudeMode" "relativeToGround";
	 data "coordinates" coords]]


let horiz_mode =
  el "Placemark" ["id", "horiz_mode"]
    [data "name" "Horizontal Route";
     line_style ~width:4 "green" "8000ff00";
     el "LineString" []
       [ data "altitudeMode" "absolute";
	 data "coordinates" ""]]

let georef_of_xml = fun xml ->
  let lat0 = Latlong.deg_of_string (ExtXml.attrib xml "lat0")
  and lon0 = Latlong.deg_of_string (ExtXml.attrib xml "lon0") in
  { posn_lat = (Deg>>Rad)lat0; posn_long = (Deg>>Rad)lon0 }

let flight_plan = fun ac_name fp ->
  let geo0 = georef_of_xml fp
  and alt0 = ExtXml.attrib fp "alt" in
  let utm0 = utm_of WGS84 geo0 in
  let xml_waypoints = Xml.children (ExtXml.child fp "waypoints") in
  let array_waypoints = Array.of_list xml_waypoints in
  (* Index the waypoints for further access *)
  for i = 0 to Array.length array_waypoints - 1 do
    Hashtbl.add waypoints (ac_name, i+1) (ExtXml.attrib array_waypoints.(i) "name")
  done;
  let waypoints = List.map (waypoint utm0 alt0) xml_waypoints in
  let home = List.find (fun x -> Xml.attrib x "id" = "HOME") waypoints in
  let home_point = ExtXml.child home "Point" in
  let ac =  
    el "Placemark" ["id", ac_name]
      [data "name" ac_name;
       icon_style ~heading:45 "ac_style" "pal2/icon56.png";
       home_point] in
  let wgs84 = of_utm WGS84 utm0 in
  let lookat =
    el "LookAt" []
      [data "longigude" (sprintf "%f" ((Rad>>Deg)wgs84.posn_long));
       data "latitude" (sprintf "%f" ((Rad>>Deg)wgs84.posn_lat));
       data "range" (ExtXml.attrib fp "max_dist_from_home")] in
  kml
    [el "Document" [] (data "open" "1"::lookat::style@(horiz_mode::ring_around_home utm0 fp::ac::waypoints))]

let aircraft = fun ac url_flight_plan url_changes ->
  let dyn_links =
    List.map (fun url ->
      el "NetworkLink" []
	  [data "name" ("Update "^ac);
	   el "Link" []
	     [data "refreshMode" "onInterval";
	      data "refreshInterval" "0.5";
	      data "href" url]])
      url_changes in
  let description = data "description" "Beta version. Open and double-click on flight plan. You may need to refresh following Update objects on errors" in
  kml
    [el "Document" []
       (description::(el "NetworkLink" []
	  [data "name" (ac^" flight plan");
	   el "Link" []
	     [data "href" url_flight_plan]]):: dyn_links)]


let change_placemark = fun ?(description="") id wgs84 alt ->
  el "Change" []
    [el "Placemark" ["targetId", id]
       [data "description" description;
	el "Point" []
	  [data "altitudeMode" "absolute";
	   data "coordinates" (coordinates wgs84 alt)]]]
    

let link_update = fun target_href changes ->
  kml
    [el "NetworkLinkControl" []
       [el "Update" [] (data "targetHref" target_href :: changes)]]

       

let change_waypoint = fun ac_name wp_id wgs84 alt ->
  let wp_name = Hashtbl.find waypoints (ac_name, wp_id) in
  change_placemark wp_name wgs84 alt

let update_linear_ring = fun target_href id coordinates ->
  kml
    [el "NetworkLinkControl" []
       [el "Update" []
	  [data "targetHref" target_href;
	   el "Change" []
	     [el "Placemark" ["targetId", id]
		[el "LineString" []
		   [data "coordinates" coordinates]]]]]]
  
  


