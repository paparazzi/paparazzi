<%@page import="pow.ConfJSP,java.util.StringTokenizer,java.util.NoSuchElementException" %>

<% 
	if (session.getAttribute("login")==null)	{
		out.println("<script type=\"text/javascript\">window.location.href=\"index.jsp\";</script>");
	}
%>
<!DOCTYPE html "-//W3C//DTD XHTML 1.0 Strict//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml">
  <head id="head">
   <meta http-equiv="content-type" content="text/html; charset=utf-8"/>
	<meta http-equiv="Pragma" content="no-cache"/>
	<meta http-equiv="Cache-Control" content="no-cache, must-revalidate" />
	<meta http-equiv="Expires" content="-1" />
	<title>POW</title>
	<link rel="stylesheet" media="screen" type="text/css" title="Design" href="./CSS/design.css" />
	<link rel="shortcut icon" type="image/x-icon" href="Icons/favicon.ico" />
	<!-- GOOGLE MAPS API V2 (deprecated) -->
	<script src="http://maps.google.com/maps?file=api&amp;v=2&amp;sensor=false&amp;key=ABQIAAAAZYVipVs4ERk_pG9XD17RUBRdUwZHIVFpQVMTarh_2qCLzSnEFBTiIKoSzrMnmbmHfo-XJ2g7sM1QUg" type="text/javascript"></script>
	<script type="text/javascript" src="./js/eshapes.js"></script>
	
	<!-- AJAX Object -->
	<script type="text/javascript" src="./js/XHR_object.js"></script>
	<!-- DOM uploading -->
	<script type="text/javascript" src="./js/DOMImplementation.js"></script>
	<!-- old stuff from former version -->
	<script type="text/javascript" src="./js/prototype.js"></script>
	<script type="text/javascript" src="./js/menu1.js"></script>
	<script type="text/javascript" src="./js/menu2.js"></script>
	<script type="text/javascript" src="./js/browserdetect.js"></script>
	
	<!-- NEEDED JQUERY  -->
	<link type="text/css" href="./CSS/ui-lightness/jquery-ui-1.8.2.custom.css" rel="stylesheet" />	
	<script type="text/javascript" src="./js/jquery-1.4.2.min.js"></script>
	<script type="text/javascript" src="./js/jquery-ui-1.8.2.custom.min.js"></script>
	
	<!-- NEEDED XML TREE -->
	<link rel="StyleSheet" href="./CSS/dtree.css" type="text/css" />
	<script type="text/javascript" src="./js/dtree.js"></script><!-- NEEDED POUR LE PUSHLET -->
	
	<!-- NEEDED PUSHLET -->
	<script type="text/javascript" src="./lib/js-pushlet-client.js"></script>
	<!-- END NEEDED POUR LE PUSHLET -->
	
	<script type="text/javascript">

/* *********DEBUG***** */
	var cpt=0;
/* ***** TOTO ***** */
	 <%
		if (session.getAttribute("login")==null)	{
			out.println("<script type=\"text/javascript\">window.location.href=\"index.jsp\";</script>");
		}
		else {
		String login = ((String) (session.getAttribute("login"))).toString();
		String right = ((String) (session.getAttribute("rights"))).toString();
		 if (right.equals("visitor")) 
		 { 	 out.println("var usr_right=\"visitor\";");
		 	 out.println("var usr_login=\"nologged\";");
		 }
		 else if (right.equals("admin"))
		 {
			 out.println("var usr_right=\"admin\";");
			 out.println("var usr_login=\""+login+"\";");
		 } 
		 else if (right.equals("user")){ // gestion de la liste des drones controlables
			 out.println("var usr_right=\"user\";");
			 out.println("var usr_login=\""+login+"\";");
			 out.println("var drone_ctl=new Array();");
			 String droneCtl = ((String) (session.getAttribute("dronectl"))).toString();
			 StringTokenizer st = new StringTokenizer(droneCtl,";");
			 String d;
			 for(int i = 0 ; i< st.countTokens() ;i++){
				 d=st.nextToken();
				 out.println("drone_ctl[\""+ d +"\"]=1;");
			 }
			 try{
			     while (st.hasMoreTokens()) {
			         d=st.nextToken();
			     }
			 }
			 catch (NoSuchElementException ex){}
			 
		 } 
		}
   %>
    var noPlaneBefore=true; // pour initialiser au premier push flight param
    var ajax_url = "ajaxRqst.srv";
/* ********** drone state timeout ******** */
	var droneStateDieEvent = new Array(); // timeout en cas d'event die
	var droneStateDataEvent   = new Array(); // timeout en cas de non evenement data
	var dieEventTimeoutTime = 10000; //10sec after plane_die event remove drone
	var dataEventTimeoutTime = 5000;  //5sec without any position data about a drone remove drone
    var waypoint_modif = new Array(); // 
    var block_jump_timeout=-1;
    var setting_change_timeout=-1;
    var order_response_timeout = 30000; // 30 secondes
	var setting_to_change_id=-1;
/* *************************************************************************** Global variables *************************************************************************************** */
	var map;
	
	var planes = new Array();
	var nb_planes =0;
	var markers = new Array();
	var markers_color = new Array(); // stock les balises de couleurs de chaque drone
	var diametre_drone_balise = 50; //50m de base
	var selected_plane = "";
	var selected_plane_id = 0;
	var selected_index=0;
	var tracking = false;

/* *******************Flight Plan variables :******************** */
	var waypoints = new Array();
	var waypoint_tmp = null;
	var index_wpt = 0;
	var blocks= new Array();
	var index_block = 0;
	var active_block_id=0;
	var active_block_name="";
	var fpl_name="";
	var lat0=0;
	var lon0=0;
	var fpl_alt=0;
	var security_height=0;
	var fpl_ground_alt=0;
	var max_dist=0;


/* ********************Planes and waypoints icons******************** */
	var plane_icons = new Array();
	for (var i=0;i<8;i++){
		plane_icons[i] = new GIcon();
		plane_icons[i].shadow = "http://labs.google.com/ridefinder/images/mm_20_shadow.png";
		plane_icons[i].iconSize = new GSize(60,60);
		plane_icons[i].shadowSize = new GSize(22, 20);
		plane_icons[i].infoWindowAnchor = new GPoint(50, 1);
		plane_icons[i].iconAnchor = new GPoint(30, 30);
	}
	plane_icons[0].image="Icons/N.png";
	plane_icons[1].image="Icons/NE.png";
	plane_icons[2].image="Icons/E.png";
	plane_icons[3].image="Icons/SE.png";
	plane_icons[4].image="Icons/S.png";
	plane_icons[5].image="Icons/SW.png";
	plane_icons[6].image="Icons/W.png";
	plane_icons[7].image="Icons/NW.png";
	
	var wpt_icon=new GIcon();
	wpt_icon.shadow = "http://labs.google.com/ridefinder/images/mm_20_shadow.png";
	wpt_icon.iconSize = new GSize(20,20);
	wpt_icon.shadowSize = new GSize(22, 20);
	wpt_icon.infoWindowAnchor = new GPoint(8, 1);
	wpt_icon.iconAnchor = new GPoint(6, 20);
	//wpt_icon.image="Icons/losange.png";
	wpt_icon.image="Icons/blue-dot.png";
	
	var iconMarkerTemp=new GIcon();
	iconMarkerTemp.shadow = "http://labs.google.com/ridefinder/images/mm_20_shadow.png";
	iconMarkerTemp.iconSize = new GSize(20,20);
	iconMarkerTemp.shadowSize = new GSize(22, 20);
	iconMarkerTemp.infoWindowAnchor = new GPoint(8, 1);
	iconMarkerTemp.iconAnchor = new GPoint(6, 20);
	iconMarkerTemp.image="Icons/red-dot.png";
/* *************************************************************************** Functions **************************************************************************************************** */	

/* ************ prevent user from going back to this page by browser's forward button****************************** */

function backButtonOverride()
{
  // Work around a Safari bug
  // that sometimes produces a blank page
  setTimeout("backButtonOverrideBody()", 1);

}

function backButtonOverrideBody()
{
  // Works if we backed up to get here
  try {
    history.forward();
  } catch (e) {
    // OK to ignore
  }
  // Every quarter-second, try again. The only
  // guaranteed method for Opera, Firefox,
  // and Safari, which don't always call
  // onLoad but *do* resume any timers when
  // returning to a page
  setTimeout("backButtonOverrideBody()", 500);
}
/* *******************  Useful functions  ******************** */
// Gets an element of the html document by its class name
	function getElementsByClass(tag, className){
		var elements = document.getElementsByTagName(tag);
		var results = new Array();
		for(var i=0; i< elements.length; i++){
			if(elements[i].className == className){
				results[results.length] = elements[i];
			}
		}
		return results;
	}
	
	
	function pause(time){
   	d=new Date();
      diff=0;
      while(diff < time){
   		n=new Date();
   		diff=n-d;
   	} 
	}
  

 //Creates a GoogleMaps marker
 

 
 
	function createMarker(point, legend, icon){
		var marker = new GMarker(point, icon);
		GEvent.addListener(marker, 'click', function() {
			marker.openInfoWindowHtml(legend);
		});
		return marker; 
	}

 	var old_lat;
 	var old_lon;
	//Creates a draggable marker with an EventListener
	function createDraggableMarker(point,namewpt,wpt_icon,index_wpt, bool){
		var fpl_file_name="upload/"+planes[selected_index]["id"]+"/flight_plan.xml";
		var marker = new GMarker(point,{legend: namewpt,icon: wpt_icon,draggable: bool,bouncy:true});
		//var marker = new GMarker(point,{title: name,draggable: bool});
		//var marker = new GMarker(point,{draggable: bool});
		GEvent.addListener(marker, 'click', function() {
			marker.openInfoWindowHtml(namewpt);
		});
		if (bool){
			GEvent.addListener(marker, 'dragend', function(latlng) {
				if (latlng){
					//alert("id wpt="+index_wpt+" name=" +namewpt+" lat="+latlng.lat()+" lon="+latlng.lng());
					var lat = latlng.lat();
					var lon = latlng.lng();
					moveWpt(selected_plane_id,namewpt,lat,lon,false,true,index_wpt);
				}
			});

			GEvent.addListener(marker, 'dragstart', function(latlng) {
				if (latlng){
					old_lat=latlng.lat();
					old_lon=latlng.lng();		
				}
			});
		}
		
		return marker;
	}
		
	
	

	
//Converts ditances from (lat0,lon0) given in meters into latitudes or longitudes, and vice versa
	function xMetersToDegrees(x,lat0){ 
		return (x/1852/60+parseFloat(lat0));
	}
	function yMetersToDegrees(y,lat,lon0){ 
		return (y/1852/60*Math.cos(lat)+parseFloat(lon0));
	}
	
	function xDegreesToMeters(lat,lat0){ 
		return((lat-lat0)*1852*60);
	}
	function yDegreesToMeters(lat,lon,lon0){ 
		return(lat!=0?(lon-lon0)/Math.cos(lat)*1852*60:0); 
	}
		
	
	function name_from_id(id){
		var match=false;
		for (var i=0;i< planes.length;i++){
			if(planes[i]["id"]==id){match=true;break;}
		}
		if (match){
			return (planes[i]["name"]);
		}else{
			return (null);
		}
	}
	/* *************** JQUERY stuffs ***************************** */
	$(function(){
				//init();
				//$("#Settings").draggable();$("#Settings").resizable();			
				//$("#map_canvas").draggable();$("#map_canvas").resizable();
				//$("#Debug").draggable();$("#Debug").resizable();
				//$("#choicePanel").draggable();$("#choicePanel").resizable();
				//$("#FlightPlan").draggable();$("#FlightPlan").resizable();
				//$("#FlightParams").draggable();$("#FlightParams").resizable();
			});
/* ********************* Fonctions de Push ******************** */
 
	function initialize_push() {
 	// alert('go');
 	 p_join_listen(null, 'stream');
	 // TODO il faudra gerer le cas ou cela bloque !!!! et passer en pull
	 // p_join_listen(null, 'pull');
	 p_subscribe('/data/drone/iskill', 'msg from serveur');
	 p_subscribe('/data/drones_maj', 'msg from serveur');
	 p_subscribe('/data/order/waypoint_moved', 'msg from serveur');
	 p_subscribe('/data/order/change_setting', 'msg from serveur');
	 p_subscribe('/data/order/plane_die', 'msg from serveur');
	 p_subscribe('/data/order/plane_resurect', 'msg from serveur');
	 p_subscribe('/data/order/new_plane', 'msg from serveur');
	 p_subscribe('/data/order/block_changed', 'msg from serveur');
	 p_subscribe('/data/order/other', 'msg from serveur');
	 p_subscribe('/data/order/settings', 'msg from serveur');

	 p_subscribe('/chat','msg from web client');
	 p_subscribe('/client_action','action from web client');
	}

	function initialize_push_pullmode() {
	 	
		 p_join_listen(null, 'pull');
		 p_subscribe('/data/drone/iskill', 'msg from serveur');
		 p_subscribe('/data/drones_maj', 'msg from serveur');
		 p_subscribe('/data/order/waypoint_moved', 'msg from serveur');
		 p_subscribe('/data/order/change_setting', 'msg from serveur');
		 p_subscribe('/data/order/plane_die', 'msg from serveur');
		 p_subscribe('/data/order/plane_resurect', 'msg from serveur');
		 p_subscribe('/data/order/new_plane', 'msg from serveur');
		 p_subscribe('/data/order/block_changed', 'msg from serveur');
		 p_subscribe('/data/order/other', 'msg from serveur');
		 p_subscribe('/data/order/settings', 'msg from serveur');

		 p_subscribe('/chat','msg from web client');
		 p_subscribe('/client_action','action from web client');
		}

	function displayControl(aString) {
		  document.debugEventDisplay.event.value = aString;
		}
	// callback on data Events
	// call apropirate function according to the received event
	
	function onData(event) {
		var subject = event.get('p_subject');
		
	//	displayControl(""+ event.toString());
		
		if  (subject=="/data/drones_maj") { handleDronePositionUpsate(event);}
		else if (subject =="/data/order/waypoint_moved") {orderprocessing_waypoint(event);}
		else if (subject =="/data/order/block_changed") {orderprocessing_jump2block(event);}
		else if (subject =="/data/order/change_setting")	{orderprocessing_setting(event);}
		else if (subject =="/data/order/plane_die") 	{orderprocessing_planedie(event);}
		else if (subject =="/data/order/plane_resurect"){orderprocessing_planeressurect(event);}
		else if (subject =="/data/order/new_plane")		{orderprocessing_newplane(event);}
		else if (subject =="/data/drone/iskill")		{orderprocessing_planekilled(event);}
		else if (subject =="/data/order/settings")		{orderprocessing_csv_settings(event);}
		else if (subject =="/chat")					{orderprocessing_chat(event);}
		else if (subject =="/client_action")		{orderprocessing_client_action(event);}
	}

	// callback on all other kind of Events
	
	<%--function onEvent(event) {
		var subject = event.get('p_subject');
		displayControl("EVENT CALLBACK "+subject+"\n" + event.toString());
		
		
		}
  --%>
/* ********************* initilization de l'interface       ******************* */
	function initialize_display(){
		var aircraftList = document.getElementById("aircraftForm").aircraftList;
		if (nb_planes ==0)//If there is no active aircraft, the list only has one option : No aircraft
			{aircraftList.options[0]=new Option("No aircraft");}
}

/* *********************functions to check airplanes        ******************* */	
   
/* *********************Processing events          ******************* */	
	// recherche si l'id de l'avion est deja present dans le tableau des drones deja 
	function handleDronePositionUpsate(event){
		if (planes.length==0) {noPlaneBefore=true;}
		var id = parseInt(event.get('aircraftId'));
		var match=false;
		var insert=false;
		var aircraftList = document.getElementById("aircraftForm").aircraftList;
		nb_planes=planes.length;
    	for (var i=0;i< nb_planes;i++){
    		if(planes[i]["id"]> id){insert=true;break;}  // on garde le tableau trié
    		if (planes[i]["id"]==id){match=true;break;}  // on recupere en i le drone correspondant à id
    	}
    	if(!match){
			var k=0;
			if(insert){ // on insert au milieu
				k=i;
				planes.splice(i,0,new Array());
				markers.splice(i,0,null);
				markers_color.splice(i,0,null);
				// on insere l'aircraft dan sla liste deroulante
				//see http://www.mredkj.com/tutorials/tutorial005.html
				var elOptNew = document.createElement('option');
				elOptNew.text = event.get('dbName')
				var elOptOld = aircraftList.options[i];  
				try {
					aircraftList.add(elOptNew, elOptOld); // standards compliant; doesn't work in IE
				    }
				catch(ex) {
				    aircraftList.add(elOptNew, i); // IE only
				    }				
			}else{ // on met à la fin du tableau
				k=planes.length;
				planes[k] = new Array();
				markers[k]= null; // inutile
				markers_color[k]=null;
				aircraftList.options[k]=new Option(event.get('dbName'));
			} 
			droneStateDieEvent[k] = -1; // init un timeout vide
			
			//alert('new plane !!!!');
			// on ajoute l'aircraft à la liste  des drones selectionnables
			
			//aircraftList.options[k]=new Option(event.get('dbName'));
			//
			planes[k]["id"]=id;
			planes[k]["name"] =event.get('dbName');
			planes[k]["lat"] = event.get('dbLatitude');
			planes[k]["lon"] = event.get('dbLongitude');
			planes[k]["heading"]= event.get('dbCourse');
			// maj des param pour selected acrf
			planes[k]["speed"]= event.get('dbSpeed');
			planes[k]["altitude"]= event.get('dbAmsl');
			planes[k]["vspeed"]= event.get('dbVert_speed');
			planes[k]["height"]= event.get('dbAgl');
			planes[k]["battery"]= event.get('dbStat_battery');
			planes[k]["GPS"]= event.get('dbStat_gps');
			planes[k]["activeBlock"]= event.get('dbActive_block');
			planes[k]["engine"]= event.get('dbEngine_status');
			planes[k]["setting_id"]= event.get('dbId_Setting');
			planes[k]["setting_value"]= event.get('dbSetting_Value');
			planes[k]["drone_color"]= event.get('drone_color');
			// gere si le drone peut etre controlé ou non 
			if ((usr_right=="admin")||((usr_right=="user")&&(drone_ctl[event.get('dbName')]!=null))){
				planes[k]["rights"]= 1;
				}
			else 
				{planes[k]["rights"]=0;}
			
			var icon;
			if (planes[k]["heading"]>=338 || planes[i]["heading"]< 23){
				icon = plane_icons[0];
			}else if (planes[k]["heading"]>=23 && planes[i]["heading"]<68){
				icon = plane_icons[1];
			}else if (planes[k]["heading"]>=68 && planes[i]["heading"]<113){
				icon = plane_icons[2];
			}else if (planes[k]["heading"]>=113 && planes[i]["heading"]<158){
				icon = plane_icons[3];
			}else if (planes[k]["heading"]>=158 && planes[i]["heading"]<203){
				icon = plane_icons[4];
			}else if (planes[k]["heading"]>=203 && planes[i]["heading"]<248){
				icon = plane_icons[5];
			}else if (planes[k]["heading"]>=248 && planes[i]["heading"]<293){
				icon = plane_icons[6];
			}else if (planes[k]["heading"]>=293 && planes[i]["heading"]<337){
				icon = plane_icons[7];
			}
			//A marker is created and added on the map
			var pos = new GLatLng(planes[k]["lat"],planes[k]["lon"]);
			markers[k]=createMarker(pos,planes[k]["name"],icon);
			markers_color[k] = GPolygon.Circle(pos,diametre_drone_balise,"#000000",1,1,planes[i]["drone_color"],0.5)
			map.addOverlay(markers[k]);
			map.addOverlay(markers_color[k]);
			droneStateDataEvent[k] = setTimeout("nodataEventCallBack("+id+")",dataEventTimeoutTime);
		}
    	else // l event envoyé correspond à un drone deja dans le tableau
    	{
    		if (droneStateDataEvent[i]!=-1) {clearTimeout(droneStateDataEvent[i]);}
    		var heading_changed;
    		//Update of the planes array
    			planes[i]["lat"] = event.get('dbLatitude');
    			planes[i]["lon"] =  event.get('dbLongitude');
    			// maj des param pour selected acrf
    			planes[i]["speed"]= event.get('dbSpeed');
				planes[i]["altitude"]= event.get('dbAmsl');
				planes[i]["vspeed"]= event.get('dbVert_speed');
				planes[i]["height"]= event.get('dbAgl');
				planes[i]["battery"]= event.get('dbStat_battery');
				planes[i]["GPS"]= event.get('dbStat_gps');
				planes[i]["activeBlock"]= event.get('dbActive_block');
				planes[i]["engine"]= event.get('dbEngine_status');
				planes[i]["setting_id"]= event.get('dbId_Setting');
				planes[i]["setting_value"]= event.get('dbSetting_Value');
				planes[i]["drone_color"]= event.get('drone_color');
    			//
    			var new_heading = event.get('dbCourse');
    			var old_heading = planes[i]["heading"];
    			//Update of the icons if the heading has changed
    			if (Math.abs(new_heading-old_heading)>2){
    				var image;
    				if (new_heading>=338 || new_heading< 23){
    					image = plane_icons[0].image;
    				}else if (new_heading>=23 && new_heading<68){
    					image = plane_icons[1].image;
    				}else if (new_heading>=68 && new_heading<113){
    					image = plane_icons[2].image;
    				}else if (new_heading>=113 && new_heading<158){
    					image = plane_icons[3].image;
    				}else if (new_heading>=158 && new_heading<203){
    					image = plane_icons[4].image;
    				}else if (new_heading>=203 && new_heading<248){
    					image = plane_icons[5].image;
    				}else if (new_heading>=248 && new_heading<293){
    					image = plane_icons[6].image;
    				}else if (new_heading>=293 && new_heading<337){
    					image = plane_icons[7].image;
    				}
    				markers[i].setImage(image);
    			}
    			planes[i]["heading"] = new_heading;
    			//Setting the marker to its new position
    			var new_pos = new GLatLng(planes[i]["lat"],planes[i]["lon"]);
    			markers[i].setLatLng(new_pos);	
    			//see http://econym.org.uk/gmap/eshapes.htm
    			map.removeOverlay(markers_color[i]);
    			markers_color[i] = GPolygon.Circle(new_pos,diametre_drone_balise,"#000000",1,1,planes[i]["drone_color"],0.5);
    			map.addOverlay(markers_color[i]);
    			droneStateDataEvent[i] = setTimeout("nodataEventCallBack("+id+")",dataEventTimeoutTime);
    	}
		nb_planes=planes.length;
      
		document.getElementById("aircraftList").options.length=planes.length;
		if (tracking&&nb_planes!=0){//If tracking is activated, the map is centered on the aircraft
				var zoom = map.getZoom();
				var plane_number = aircraftList.selectedIndex;
				map.setCenter(new GLatLng(planes[plane_number]["lat"],planes[plane_number]["lon"]),zoom);
			}

		if (noPlaneBefore==true){
			noPlaneBefore=false;// !!! interruption possible
			active_block_id=planes[0]["activeBlock"];
			aircraftList.options[0].selected=true;
			selected_plane = planes[0]["name"];
			selected_plane_id = planes[0]["id"];
			DOMImplementation("upload/"+planes[0]["id"]+"/flight_plan.xml",fplDisplay);
			DOMImplementation("upload/"+planes[0]["id"]+"/settings.xml",settingsDisplay);
			}
	   selected_plane_update(selected_plane_id);
	}


/* ******************** affichage log  ******************* */

var coloryellow = true;
function addMsgLog(msg)
{
	 var element = document.getElementById("info");
	 //var element = document.getElementsByTagName("info");
	 var newinfodiv = document.createElement("div");
	 var newinfotxt = document.createTextNode(msg);
	 if (coloryellow) {newinfodiv.style.background = '#FFE4B5';coloryellow=false;}
	 else {newinfodiv.style.background = '#D2B48C';coloryellow=true;}
	 newinfodiv.appendChild(newinfotxt);
	 //element[0].appendChild(newinfodiv);
	 element.appendChild(newinfodiv);
}
/* ******************** Initialization of the page ******************* */

/* Initialization of the page : map, markers, aircraft list... */
	
	
    function initialize() { 
    	
		if (GBrowserIsCompatible()) {
		//	initialize_planes_data();
			initialize_display();
			initialize_push();
			map = new GMap2(document.getElementById("map_canvas"));//Map creation
		//	map.setZoom(8);
			map.setMapType(G_HYBRID_MAP);
			var point= new GLatLng(43.46223, 1.27289);
			map.setCenter(point,15);//Center the map on a  temporary point 
			map.setUIToDefault();
			//
			GEvent.addListener(map, "zoomend", function(oldlevel,newlevel) {
				var d = newlevel-oldlevel;
				if (d<0) {
				diametre_drone_balise=diametre_drone_balise*(-2*d);
				}
				else
				{
				diametre_drone_balise=diametre_drone_balise/(2*d);
				}
				//addMsgLog("d="+diametre_drone_balise+" oldz="+oldlevel+" newz=" +newlevel);
				});
		}else{
		alert("Your browser is not compatible with Google Maps !");
		}
    }
	
	

/* Updating of the selected aircraft flight parameters */
// recherche la place de l'avion dans le tableau 
	function seekIndex(plane_id)
	{
		var i = 0;
		var trouve = false;
		var res=-1;
		while ((i<nb_planes)&&(!trouve)){
			if (planes[i]["id"]==plane_id) { trouve = true ;res = i;}
			i++;
		}
		return res;
	}

	function reinit_param_panel()
	{
		document.getElementById("altitude").innerHTML="##";
		document.getElementById("height").innerHTML="##";
		document.getElementById("battery").innerHTML="##";
		document.getElementById("speed").innerHTML="##";
		document.getElementById("GPS").innerHTML="##";
		document.getElementById("vspeed").innerHTML="##";
		document.getElementById("engine").innerHTML="##";
	}

	// update filght parameter of the given drone
	function selected_plane_update(plane_id){ // plane _id a la place de data
			var index_plane = seekIndex(plane_id);
			//var old_active_block_id = active_block_id;			
			active_block_id=planes[index_plane]["activeBlock"]; // alert(''+active_block_id);
			/*// plus necessaire, assuré par le callback orderprocessing_jump2block
			if (old_active_block_id!=active_block_id)
			{	
				// a chercher pour faire mieux, ne changer que la couleur du noeud de l'arbre...
				DOMImplementation("upload/"+plane_id+"/flight_plan.xml",fplDisplay);
			}
			*/ 
			active_block_name=blocks[active_block_id]["name"];  //alert(''+active_block_name);
			document.getElementById('active_block').innerHTML='Active block : '+active_block_name+'';
			
			//Update the flight parameters
			var alti=document.getElementById("altitude");
			alti.innerHTML = planes[index_plane]["altitude"];
			if(parseFloat(planes[index_plane][altitude])<0){
				alti.style.cssText='color:red;';
			}else{
				alti.style.cssText='color:black;';
			}
			var height=document.getElementById("height");
			height.innerHTML =planes[index_plane]["height"]+" m";
			if(parseFloat(planes[index_plane][height]) < security_height){
				height.style.cssText='color:red;';
			}else{
				height.style.cssText='color:black;';
			}
			var bat=document.getElementById("battery");
			var n = parseFloat(planes[index_plane]["battery"]);
			var nf = new Number(n);
			bat.innerHTML = nf.toFixed(2)+" V";
			if(parseFloat(planes[index_plane][battery])<2){
				bat.style.cssText='color:red;';
			}else{
				bat.style.cssText='color:black;';
			}
			var spd=document.getElementById("speed");
			n = parseFloat(planes[index_plane]["speed"]);
			nf = new Number(n);
			spd.innerHTML = nf.toFixed(2)+" m/s";
			var gps=document.getElementById("GPS");
			
			if(planes[index_plane]["GPS"]=="3D"){
				gps.innerHTML = "OK";
				gps.style.cssText='color:black;';
			}else{
				gps.innerHTML = "KO";
				gps.style.cssText='color:red;';
			}
			var vspd=document.getElementById("vspeed");
			vspd.innerHTML = planes[index_plane]["vspeed"]+" m/s";
			var engine_stat=document.getElementById("engine");
			engine_stat.innerHTML = planes[index_plane]["engine"]+"%";
		}

	
/*Centers the map on the selected aircraft and dispalys its flight plan, settings and flight parameters*/
	function plane_focus(aircraftList){
		// raz des waypoints en cours de modif
		for(var j=0; j<waypoint_modif.length;j++)
		{
			if (waypoint_modif[j]!=null) {clearTimeout(waypoint_modif[j].timer);}
			waypoint_modif[j] = null;
		}	 
		clearTimeout( block_jump_timeout);
		clearTimeout( setting_change_timeout);
		block_jump_timeout=-1;
		setting_change_timeout=-1;
		////////////////////////  
		// maj des pln et setting du nouveau drone selectionné
		selected_index = aircraftList.selectedIndex;
		selected_plane = planes[selected_index]["name"];
		selected_plane_id = planes[selected_index]["id"];
		map.setCenter(markers[selected_index].getLatLng(),map.getZoom());
		DOMImplementation("upload/"+planes[selected_index]["id"]+"/flight_plan.xml",fplDisplay);
		DOMImplementation("upload/"+planes[selected_index]["id"]+"/settings.xml",settingsDisplay);
		selected_plane_update(selected_plane_id);
	}

	// handle tha tracking checkbox
	function track(checkbox){
		if (checkbox.checked){
			tracking = true;
		} else{
			tracking = false;
		}
	}
	
	
/* ******************** Flight Plan processing *********************** */

/////////////////////////////////////////////////
// node cleaner 
// remove all useless node inserted by Firefox in the DOM tree
////////////////////////////////////////////////
function go(c){
	if(!c.data.replace(/\s/g,''))
		c.parentNode.removeChild(c);
}

function clean(d){
	var bal=d.getElementsByTagName('*');

	for(i=0;i<bal.length;i++){
		a=bal[i].previousSibling;
		if(a && a.nodeType==3)
			go(a);
		b=bal[i].nextSibling;
		if(b && b.nodeType==3)
			go(b);
	}
	return d;
}
//////////////////////////////////////////////////////////
//build tree for FPL Display
/////////////////////////////////////////////////////////:
var index=0;
var fpl_tree=null;

 function create_tree_fpl(docXML)
 {  index = 0;
    fpl_tree = new dTree('fpl_tree');
    //////// reinitialisation ///////////
	if (waypoints.length!=0){
		// TODO vider google map
		for(var i =0; i < waypoints.length;i++)
		{
		 map.removeOverlay(waypoints[i]["marker"]);
		}
		waypoints=new Array();
		index_wpt = 0;		
	}
	if (blocks.length!=0){
		blocks=new Array();
		index_block = 0;
	}
    var doc=docXML.documentElement; 
    document.getElementById("FlightPlan").innerHTML="loading ...";           
    addChildFPL(doc,-1);
    document.getElementById("FlightPlan").innerHTML=fpl_tree;  
    fpl_tree.closeAll();   
 }

// recursive function
function addChildFPL(childXML,index_pere)
{
	if ((childXML.nodeType==1)|| (childXML.nodeType==9)) // element node or document node
	{	
		if ((childXML.nodeName !="dump" )&&(childXML.nodeName !="stages" ))
		{    //var newindex = index++;
			 //d.add(newindex,index_pere,childXML.nodeName);
			 if ((childXML.nodeName!="waypoint")&&(childXML.nodeName!="block"))
			 {
				 var txt = ""+childXML.nodeName+" : " ;
				 if(childXML.attributes!=null){
					var nbr_attrib  = childXML.attributes.length;	
					for(var k =0;k< nbr_attrib;k++){
						 var newindex_attrib_name = index++;
						 var newindex_attrib_value = index++;
						  //d.add(newindex_attrib_name,newindex,childXML.attributes[k].nodeName);
						  //d.add(newindex_attrib_value,newindex_attrib_name,childXML.attributes[k].nodeValue);			
						txt = txt + childXML.attributes[k].nodeName + "=" + childXML.attributes[k].nodeValue +" ";
					}	
				 }
				 var newindex = index++;
				 fpl_tree.add(newindex,index_pere,txt);
				 //
				  if (childXML.nodeName=="flight_plan"){
				    //Flight plan informations processing:
					fpl_alt=parseFloat(childXML.getAttribute("alt"));
					fpl_ground_alt=parseFloat(childXML.getAttribute("ground_alt"));
					lat0=parseFloat(childXML.getAttribute("lat0"));
					lon0=parseFloat(childXML.getAttribute("lon0"));
					max_dist=parseFloat(childXML.getAttribute("max_dist_from_home"));
					fpl_name=childXML.getAttribute("name");
					security_height=parseFloat(childXML.getAttribute("security_height"));
				  }			 
			 }
			 else if (childXML.nodeName=="waypoint") // balise waypoint
			 {
				if(childXML.attributes!=null){
					var nbr_attrib  = childXML.attributes.length;	
					var nameWpt = childXML.getAttribute("name");
					var x;
					var y;
					var lat;
					var lon;
					// ********************************************************** //
					// traitement des waypoints pour affichage sur la carte et function de deplacement
					waypoint_modif[index_wpt]=null;
					waypoints[index_wpt]=new Array();
					// ********************************************************** //
					waypoints[index_wpt]["name"]=nameWpt;
					waypoints[index_wpt]["alt"]=false;
					if (childXML.getAttribute("x")!=null)
					{
						alert('conversion x->lat y->lon !');
						x = parseFloat(childXML.getAttribute("x"));
						y = parseFloat(childXML.getAttribute("y"));
						lat=xMetersToDegrees(x,lat0);
						lon=yMetersToDegrees(y,lat,lon0);
					}
					else
					{
					 lat=parseFloat(childXML.getAttribute("lat"));
					 lon=parseFloat(childXML.getAttribute("long"));
					}
					var right=planes[selected_index]["rights"];
					waypoints[index_wpt]["marker"]=null;

					if (right==1){
						waypoints[index_wpt]["marker"]=createDraggableMarker(new GLatLng(lat,lon),nameWpt,wpt_icon,index_wpt, true);
					}else {
						waypoints[index_wpt]["marker"]=createDraggableMarker(new GLatLng(lat,lon),nameWpt,wpt_icon,index_wpt, false);
					}
					map.addOverlay(waypoints[index_wpt]["marker"]);
					
					var alt=false;
					if (childXML.getAttribute("alt")!=null){
						alt=childXML.getAttribute("alt");
						waypoints[index_wpt]["alt"]=parseFloat(childXML.getAttribute("alt"));
					}
					var node_url="";
					if (right==1){
					 	 node_url = "javascript:moveWpt("+selected_index+",'"+nameWpt+"',"+lat+","+lon+","+alt+",false,"+(index_wpt)+")";
					}
					/////
					index_wpt++;
					var newindex = index++;
					fpl_tree.add(newindex,index_pere,nameWpt,node_url);
					for(var k =0;k< nbr_attrib;k++){
						 var newindex_attrib_name = index++;
						 var newindex_attrib_value = index++;
						 fpl_tree.add(newindex_attrib_name,newindex,childXML.attributes[k].nodeName);
						 fpl_tree.add(newindex_attrib_value,newindex_attrib_name,childXML.attributes[k].nodeValue);			
						//txt = txt + childXML.attributes[k].nodeName + "=" + childXML.attributes[k].nodeValue +" ";
					}	
				}
			 }
			 else if (childXML.nodeName=="block")
			 {   var node_url="";var txt_block_name="";
				 if(childXML.attributes!=null){
					var block_name=childXML.getAttribute("name");//.replace(/ /g, '_');
					blocks[index_block]=new Array();
					blocks[index_block]["name"]=block_name;
					if (active_block_id==index_block){
						active_block_name=block_name;
						txt_block_name = "<span style='background-color:#FF0000'>"+block_name+"</span>"; 
						}
					else
					{
						txt_block_name = block_name;
					}
					var right=planes[selected_index]["rights"];
					
					if (right==1){
						node_url="javascript:activateBlock("+selected_index+",'"+block_name+"',"+index_block+")";
					}					
					index_block++;
					var txt = txt_block_name+ " : ";
					var nbr_attrib  = childXML.attributes.length;	
					for(var k =0;k< nbr_attrib;k++){
						if (childXML.attributes[k].nodeName!="name"){
							 var newindex_attrib_name = index++;
							 var newindex_attrib_value = index++;
							  //d.add(newindex_attrib_name,newindex,childXML.attributes[k].nodeName);
							  //d.add(newindex_attrib_value,newindex_attrib_name,childXML.attributes[k].nodeValue);			
							txt = txt + childXML.attributes[k].nodeName + "=" + childXML.attributes[k].nodeValue +" ";
						}
					}	
				 }
				 var newindex = index++;
				 fpl_tree.add(newindex,index_pere,txt,node_url);
			 }
			 // recursion
			 if(childXML.hasChildNodes()) {
				 var nodes=childXML.childNodes.length;
				 for(var i=0; i<childXML.childNodes.length; i++) {
					addChildFPL(childXML.childNodes[i],newindex);
				 }
			 }
		}
		else if (childXML.nodeName =="dump" )
			{
				if(childXML.hasChildNodes()) {
					 var nodes=childXML.childNodes.length;
					 for(var i=0; i<childXML.childNodes.length; i++) {
						addChildFPL(childXML.childNodes[i],index_pere);
					 }
				 }
			}
	}		
	else if ((childXML.nodeType==3)) //text node
	{ 	var newindex = index++;	 
		fpl_tree.add(newindex,index_pere,childXML.nodeValue);
	}
	else if ((childXML.nodeType==4)) //comment node
	{ 
	  //skip
	}	
} 

function cleanAndTraverse_fpl(docXML)
{
 var dom_cleaned = clean(docXML); // remove useless node inserted by firefox in xml tree
 create_tree_fpl(dom_cleaned);
}
/////////////////////////////////////////////////////////////////



function fplDisplay(data){
	cleanAndTraverse_fpl(data)
	document.getElementById("FlightPlan").innerHTML=fpl_tree;
}

/////////////////////////////////////////////////////////////////
/* ******** Moves a waypoint : moves the marker, updates the description in the waypoints list,   ******** */
/* ********	updates the flight plan via an Ajax request.                                          ******** */

function createWaypoint(acid,wptid,namewpt,marker,timer)
{
 this.acid = acid;
 this.wptid = wptid;
 this.namewpt = namewpt;
 this.marker = marker;
 this.timer = timer;
}

// create the ajax request to move a waypoint
	function moveWpt(ac_webid,name,mylat,mylon,alt,dragged,idwpt){

		// verifie qu'il n'y pas pas un verrou sur le waypoint posé par un autre client
		var k=0;
		var trouve = false;
		while ((!trouve)&&(k<external_waypoint_lock.length))
		{
			if((external_waypoint_lock[k].id_drone == ac_webid)&&
				(external_waypoint_lock[k].id_object == idwpt))
			{
				trouve = true;
			}
			k++;
		}
		if (trouve) {alert("Waypoint has been locked by another user");return;}
		
		var new_lat;
		var new_lon;
		var new_alt=false;
		
		if(!dragged){
			new_lat=prompt("latitude=",mylat);
			if ((new_lat==null)){
				return;
			}
			var l = parseFloat(new_lat);
			if ((l>90.0)||(l<-90.0)||isNaN(new_lat)) {
				alert('wrong format for a latitude... should be within [-90:+90]');
				return;	
			}
			new_lon=prompt("longitude=",mylon);
			if ((new_lon==null)){
				return;
			}
			l = parseFloat(new_lon);
			if ((l>180.0)||(l<-180.0)||isNaN(new_lon)) {
				alert('wrong format for a longitude... should be within [-180:+180]');
				return;	
			}
			if (alt){
				new_alt=prompt("altitude=",alt);
				if ((new_alt==null)){
					return;
				}
				l = parseFloat(new_alt);
				if ((l<=0.0)||isNaN(new_alt)) {
					alert('wrong format for an altitude... should be positive');
					return;	
				}
			}
			//
			old_lat = waypoints[idwpt]["lat"];
			old_lon = waypoints[idwpt]["lon"];
		}else{
			new_lat=mylat;
			new_lon=mylon;
			//alert("acid="+ac_webid+" wptid="+idwpt+" lat="+mylat+"lon="+mylon+"alt="+alt);
			//new_lat = waypoints[idwpt]["marker"].getLatLng().lat();
			//new_lon = waypoints[idwpt]["marker"].getLatLng().lat();
		}
		
		
	    //Ajax request to edit the flight plan xml file
		var xhr = getXMLHttpRequest();
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4 && xhr.status == 200) {
				// maj de l'affichage fpl
				var xmlResponse = xhr.responseXML.documentElement;
				var rep = xmlResponse.getElementsByTagName("waypoint_to_move")[0];
				var idwpt_requested = parseInt(rep.getAttribute("idwpt"))-1;// cf waypoint fictif ivy
				var acid_requested  = parseInt(rep.getAttribute("acid"));
				var lat_requested   = parseFloat(rep.getAttribute("newlat"));
				var lon_requested   = parseFloat(rep.getAttribute("newlon"));
				var was_dragged     = rep.getAttribute("dragged");
				var name_drone = planes[seekIndex(acid_requested)]["name"];
				// on ne met à jour visuellement que si le focus est sur le drone selecté
				if (acid_requested==selected_plane_id){
					  // mettre le waypoint dans un etat intermediaire 
					  // en attendant l'accuse de reception
					  var right=planes[selected_index]["rights"];
					  var marker_tmp;
					  if (right==1){
						  marker_tmp=createDraggableMarker(new GLatLng(old_lat,old_lon),waypoints[idwpt_requested]["name"],wpt_icon,idwpt_requested, true);
					  }else {
						  marker_tmp=createDraggableMarker(new GLatLng(old_lat,old_lon),waypoints[idwpt_requested]["name"],wpt_icon,idwpt_requested, false);
					  }
					  var name_wpt =  waypoints[idwpt_requested]["name"];
					  var timer=setTimeout("callback_waypoint("+acid_requested+","+idwpt_requested+")",order_response_timeout);
 					  waypoint_modif[idwpt_requested] = new createWaypoint(acid_requested,idwpt_requested,name_wpt,marker_tmp,timer);
 					  // on efface le marker courant
 					  // meme si elle a ete deplacé par drag & drop
					  // on cree un marker non draggable avec une icone temporaire
					  map.removeOverlay(waypoints[idwpt_requested]["marker"]);
					  waypoints[idwpt_requested]["marker"] = createMarker(new GLatLng(lat_requested,lon_requested),name_wpt,iconMarkerTemp);
					  if (waypoints[idwpt_requested]["marker"]!=null) map.addOverlay(waypoints[idwpt_requested]["marker"]);
					  else alert('echec creation wpt tmp');
					  addMsgLog("waypoint "+ name_wpt +" for drone " + name_drone +" will be moved");
				}
				else
				{
					 addMsgLog("waypoint "+ idwpt_requested +" for drone " + name_drone +" will be moved");
				}
				
			}
		};
		xhr.open("POST",ajax_url,true);
		xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
		// on informe les autres clients de ne pas modifier le waypoint en cours (via pushlet)
		p_publish('/client_action', 'login',usr_login,'action', 'lock', 'type_obj', 'waypoint', 'id_obj',idwpt,'webiddrone',selected_plane_id);
		// on passe en parametre le index_wpt+1 car dans paparazzi il y a un wpt predefini a zero
		
		if(alt!=false){
		xhr.send("order=fpl_update&wpt_name="+name+"&aircraft_id=" +selected_plane_id+"&wpt_id="+(idwpt+1)+"&new_lat="+new_lat+"&new_lon="+new_lon+"&new_alt="+new_alt+"&dragged="+dragged+"&new_alt_for_fpl=1");
		//alert("wptid="+i+" lat="+new_lat+"lon="+new_lon+"alt="+new_alt);
		}else{
		var alti=document.getElementById("altitude");var h=parseFloat(alti.innerHTML);
		xhr.send("order=fpl_update&wpt_name="+name+"&aircraft_id=" +selected_plane_id+"&wpt_id="+(idwpt+1)+"&new_lat="+new_lat+"&new_lon="+new_lon+"&new_alt="+parseFloat(alti.innerHTML)+"&dragged="+dragged+"&new_alt_for_fpl=0");
		//alert("wptid="+i+" lat="+new_lat+"lon="+new_lon+"alt="+h);
		}	
}

	
/* ******** Activate a flight plan block ******** */
 
 

	function activateBlock(ac_webid,blockname,block_id){
		// verifie qu'il n'y pas pas un verrou sur le block posé par un autre client
		var k=0;
		var trouve = false;
		while ((!trouve)&&(k<external_block_lock.length))
		{
			if((external_block_lock[k].id_drone == ac_webid)&&
				(external_block_lock[k].id_object == block_id))
			{
				trouve = true;
			}
			k++;
		}
		if (trouve) {alert("Block has been locked by another user");return;}
		// 
		var xhr = getXMLHttpRequest();
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4 && xhr.status == 200) {
				//alert("block activated !");
				var xmlResponse = xhr.responseXML.documentElement;
				var rep = xmlResponse.getElementsByTagName("block_to_activate")[0];
				var idblock_requested = parseInt(rep.getAttribute("idblock"));
				var acid_requested  = parseInt(rep.getAttribute("acid"));
				var name_drone = planes[seekIndex(acid_requested)]["name"];
				addMsgLog("request jump to block "+ idblock_requested +" for drone " + name_drone +" has been sent");
				// mise en place d'un timer d'accusé reception
				if(acid_requested==planes[selected_index]["id"]){
					block_jump_timeout=setTimeout("callback_block("+acid_requested+","+idblock_requested+")",order_response_timeout);
				}
				
			}
		};
		// on informe les autres clients de ne pas modifier le setting en cours (via pushlet)
		p_publish('/client_action', 'login',usr_login,'action', 'lock', 'type_obj', 'block', 'id_obj',block_id,'webiddrone',selected_plane_id);
		//
		xhr.open("POST",ajax_url,true);
		xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
		//Send the parameters to the php page :
		xhr.send("order=activate_block&aircraft_id=" +selected_plane_id+"&block_id="+block_id);
	}

	
	
	
	
/* ******************************************* Settings file processing **************************************************** */

//////////////////////////////////////////////////////////
//build tree for Setting  Display
/////////////////////////////////////////////////////////:
var tabs_id_array = new Array();
var slider_array = new Array();
var old_value_setting;
var csv = null;

 function settingsDisplay(docXML){
		var dom_cleaned = clean(docXML);
		var root=dom_cleaned.documentElement; 
		// init
		tabs_id_array = new Array();
		slider_array = new Array();
		index_setting_id=0;
		remove_setting_display();
		if (planes[selected_index]["csv"]!=null){
			csv = planes[selected_index]["csv"].split(",");
		}
		else {csv=null;}
		//
		addChild(root,0,0);
		var i = 0; 
		var id_tabs;
		tabs_id_array.push("0");
		while(i < tabs_id_array.length){
				id_tabs = tabs_id_array.pop();
				$("#tabs_"+id_tabs).tabs();	
				//alert("creation tabs : " +"#tabs_"+id_tabs); 					
		}
		i=0;var sli;
		while(i < slider_array.length){
			sli = slider_array.pop();
			//alert("slider " + sli.slidername);
			$("#slider_"+sli.slidername).slider({ max: sli.max,min:sli.min,step:sli.step, value: sli.value,
													stop: function(event, ui) { 
														var div=ui.handle.parentNode;
														var name = div.getAttribute("var");
														var idsetting = div.getAttribute("idsetting");
														var idnum = div.getAttribute("idnum");			
														var right=planes[selected_index]["rights"];
														if(right==1) { changeSetting2(idsetting,ui.value); }
														else {
															alert("you don't have enough rights to deal with this aircraft");
															$("#slider_"+idnum).slider("value",old_value_setting);
															$("#slider_label_"+idnum).val(old_value_setting);
															} 											
													},
													slide : function(event,ui) {
														var div=ui.handle.parentNode;
														var idnum = div.getAttribute("idnum");
														$("#slider_label_"+idnum).val(ui.value);
													},
													start: function(event, ui) {
														// verifie qu'il n'y pas pas un verrou sur le block posé par un autre client
														var div=ui.handle.parentNode;
														var idsetting = div.getAttribute("idsetting");
														var k=0;
														var trouve = false;
														while ((!trouve)&&(k<external_setting_lock.length))
														{
															if((external_setting_lock[k].id_drone == selected_plane_id)&&
																(external_setting_lock[k].id_object == idsetting))
															{
																trouve = true;
															}
															k++;
														}
														old_value_setting=ui.value;
														if (trouve) {
															$("#slider_"+idnum).slider("value",old_value_setting);
															$("#slider_label_"+idnum).val(old_value_setting);
															alert("Settings has been locked by another user");return;
															}
														//
													}
													
				 });
			$("#slider_label_"+sli.slidername).val($("#slider_"+sli.slidername).slider("value"));

		}	
				 			
	}

 function createSliderObj(n,v,mi,ma,s,v){
		this.slidername=n ;
		this.variable = v;
		this.max = ma;
		this.min = mi;
		this.step=s;
		this.value = v;
	}
	
	function addChild(childXML,id_pere,nextid_fils){
		var idChild = id_pere;	
		if ((childXML.nodeType==1)|| (childXML.nodeType==9)) // element node or document node
		{
			if (childXML.nodeName=="dl_settings") {
				 var name = childXML.getAttribute("name");
				 if (name!=null){
				   idChild=id_pere+"_"+nextid_fils;
				 	var cur_tabs = document.getElementById('tabs_'+id_pere);
				 	//alert('recherche ul : ' + 'tabs_'+id_pere+'_ul')
					var cur_ul= document.getElementById('tabs_'+id_pere+'_ul');
				 	var new_li= document.createElement("li"); 								
					var new_a = document.createElement("a");
					var ahref =document.createAttribute("href");
					ahref.nodeValue = "#tabs_"+idChild;
					new_a.setAttributeNode(ahref);
					new_a.innerHTML = name;    								
				 	new_li.appendChild(new_a);						 	    					 
				 	cur_ul.appendChild(new_li);
				 	var new_div = document.createElement("div");
				 	var new_div_id = document.createAttribute("id");    					 	
				 	new_div_id.nodeValue = "tabs_"+idChild;
				 	new_div.setAttributeNode(new_div_id);
				 	var new_ul = document.createElement("ul");
				 	var new_ul_id = document.createAttribute("id");
				 	new_ul_id.nodeValue = "tabs_"+idChild+"_ul";
				 	new_ul.setAttributeNode(new_ul_id);
				 	new_div.appendChild(new_ul);
				 	cur_tabs.appendChild(new_div);
				 	tabs_id_array.push(idChild); 						 
				 }
			}
			else if (childXML.nodeName=="dl_setting") {
			
				var smax = parseFloat(childXML.getAttribute("max"));
				var smin = parseFloat(childXML.getAttribute("min"));
				var svariable = childXML.getAttribute("var");
				var sstep = parseFloat(childXML.getAttribute("step"));
				var svalue=0;
				if (csv!=null){
					svalue = parseFloat(csv[index_setting_id]);
				}
				else {
					svalue= (smin+smax)/2;
				}
				idChild=id_pere+"_"+nextid_fils;//alert('setting slider '+ idChild);
				var cur_div= document.getElementById('tabs_'+id_pere);
				var newtable = document.createElement("table");
				var newligne = document.createElement("tr");
				var newcol = document.createElement("td");
				var newstyle = document.createAttribute("style");
				newstyle.nodeValue = "white-space:nowrap;";
				newcol.setAttributeNode(newstyle);
				var newp = document.createElement("p");
				var newlabel = document.createElement("label");
				var newinput = document.createElement("input");
				newstyle = document.createAttribute("style");
				newstyle.nodeValue = "width:30px;font-size:8px;"
				newinput.setAttributeNode(newstyle);
				var newtype = document.createAttribute("type");
				newtype.nodeValue = "text";
				var newid = document.createAttribute("id");
				newid.nodeValue = "slider_label_"+id_pere+"_"+nextid_fils;
				newinput.setAttributeNode(newid);newinput.setAttributeNode(newtype);
				var newfor = document.createAttribute("for");
				newfor.nodeValue = "slider_label_"+id_pere+"_"+nextid_fils;
				newlabel.innerHTML = ""+ svariable +" :";
				newp.appendChild(newlabel);newp.appendChild(newinput);      					
				//
				newcol.appendChild(newp);
				newligne.appendChild(newcol);
				//cur_div.appendChild(newp);
				
				var new_div_slide = document.createElement("div");
				var slide_name = id_pere+"_"+nextid_fils;
				var new_div_slide_id = document.createAttribute("id");
				new_div_slide_id.nodeValue = "slider_"+slide_name;
				new_div_slide.setAttributeNode(new_div_slide_id);
				var new_param = document.createAttribute("var");
				new_param.nodeValue = svariable;
				new_div_slide.setAttributeNode(new_param);
				new_param = document.createAttribute("idnum");
				new_param.nodeValue = slide_name;
				new_div_slide.setAttributeNode(new_param);
				new_param = document.createAttribute("idsetting");
				new_param.nodeValue = index_setting_id;
				index_setting_id++;
				new_div_slide.setAttributeNode(new_param);
				var width_param = document.createAttribute("style"); 
				width_param.nodeValue = "width:150px;height:5px;";
				new_div_slide.setAttributeNode(width_param);
				
				//cur_div.appendChild(new_div_slide); 						
				newcol = document.createElement("td");
				newcol.appendChild(new_div_slide);
				newligne.appendChild(newcol);
				newtable.appendChild(newligne)
				cur_div.appendChild(newtable);
				//
				var slider = new createSliderObj(slide_name,svariable,smin,smax,sstep,svalue);
				slider_array.push(slider);
				
			}
			// recursion
			if(childXML.hasChildNodes()) {
			 var nodes=childXML.childNodes.length;
			 for(var i=0; i<childXML.childNodes.length; i++) {
			    //alert("adchild p="+idChild + " nextfils="+i);
				 addChild(childXML.childNodes[i],idChild,i);
			 }
		 }
		}
		else if ((childXML.nodeType==3)) //text node
		{ 	
			// skip
		}
		else if ((childXML.nodeType==4)) //comment node
		{ 
			//skip
		}
	}


	function remove_setting_display() {
		var setting = document.getElementById("setting");
		var noeud = document.getElementById("setting").firstChild;
		setting.removeChild(noeud);
		var newdiv = document.createElement("div");
		var id = document.createAttribute("id");
		id.nodeValue = "tabs_0";
		newdiv.setAttributeNode(id);
		var newul = document.createElement("ul");
		id = document.createAttribute("id");
		id.nodeValue = "tabs_0_ul";
		newul.setAttributeNode(id);
		newdiv.appendChild(newul);
		setting.appendChild(newdiv);
	}
	
/* ******************************************************** */


/* ******************************************************** */
	function changeSetting(idsetting,max,min,step){

		// verifie qu'il n'y pas pas un verrou sur le block posé par un autre client
		var k=0;
		var trouve = false;
		while ((!trouve)&&(k<external_setting_lock.length))
		{
			if((external_setting_lock[k].id_drone == selected_plane_id)&&
				(external_setting_lock[k].id_object == idsetting))
			{
				trouve = true;
			}
			k++;
		}
		if (trouve) {alert("Settings has been locked by another user");return;}
		// 
		var prompt_string='';
		if (parseFloat(max-min)/parseFloat(step)<=5){
			prompt_string='Please enter a value between '+min+' and '+max+' with a step of '+step;
		}else{
			prompt_string='Please enter a value between '+min+' and '+max;		
		}
		var new_value=prompt(prompt_string);
		if (new_value==null){
			return;
		}
		var correct= false;
		while (!correct){
			if ( (new_value <= parseFloat(max)) && (new_value >= parseFloat(min))  ){
				correct=true;
				var xhr = getXMLHttpRequest();
				xhr.onreadystatechange = function() {
					if (xhr.readyState == 4 && xhr.status == 200) {	
						var xmlResponse = xhr.responseXML.documentElement;
						var rep = xmlResponse.getElementsByTagName("setting_to_change")[0];
						var idsetting_requested = parseInt(rep.getAttribute("setting_id"));
						var acid_requested  = parseInt(rep.getAttribute("acid"));
						var name_drone = planes[seekIndex(acid_requested)]["name"];
						addMsgLog("request change setting "+ idsetting_requested +" for drone " + name_drone +" has been sent");
						// mise en place d'un timer d'accusé reception
						if(acid_requested==planes[selected_index]["id"]){
							setting_change_timeout=setTimeout("callback_setting("+acid_requested+","+idsetting_requested+")",order_response_timeout);
							setting_to_change_id = idsetting_requested;
						}
					}
				};
				xhr.open("POST",ajax_url,true);
				xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
				//Send the parameters to the php page :
				xhr.send("order=modif_setting&aircraft_id=" +selected_plane_id+"&setting_id="+idsetting+"&value="+new_value);				
				
			}else{
				new_value=prompt("Wrong value, please enter a value between "+min+" and "+max+" with a step of "+step+" :");
				if (new_value==null){return;}
			}
		}
	}
	
	function changeSetting2(idsetting,new_value){
				
				var xhr = getXMLHttpRequest();
				xhr.onreadystatechange = function() {
					if (xhr.readyState == 4 && xhr.status == 200) {	
						var xmlResponse = xhr.responseXML.documentElement;
						var rep = xmlResponse.getElementsByTagName("setting_to_change")[0];
						var idsetting_requested = parseInt(rep.getAttribute("setting_id"));
						var acid_requested  = parseInt(rep.getAttribute("acid"));
						var name_drone = planes[seekIndex(acid_requested)]["name"];
						addMsgLog("request change setting "+ idsetting_requested +" for drone " + name_drone +" has been sent");
						// mise en place d'un timer d'accusé reception
						if(acid_requested==planes[selected_index]["id"]){
							setting_change_timeout=setTimeout("callback_setting("+acid_requested+","+idsetting_requested+")",order_response_timeout);
							setting_to_change_id = idsetting_requested;
						}
					}
				};
				// on informe les autres clients de ne pas modifier le setting en cours (via pushlet)
				p_publish('/client_action', 'login',usr_login,'action', 'lock', 'type_obj', 'setting', 'id_obj',idsetting,'webiddrone',selected_plane_id);
				//
				xhr.open("POST",ajax_url,true);
				xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
				//Send the parameters to the servlet :
				xhr.send("order=modif_setting&aircraft_id=" +selected_plane_id+"&setting_id="+idsetting+"&value="+new_value);				
				
	}
	
/* ************************* handle event orders from server ********************************************* */
			
	function orderprocessing_waypoint(event){
	
		var acft_id = parseInt(event.get('aircraftId'));
		var wpt_id = parseInt(event.get('waypointId'))-1; // cf indentation des waypoint sous ivy
		var lat = event.get('latitude');
		var lon = event.get('longitude');
		var alt = event.get('altitude');
		var name=name_from_id(acft_id);
		//alert('drone=' +name+' wpt = '+wpt_id);
		if ((wpt_id>=0)&&(acft_id==selected_plane_id)) {
			//alert('drone=' +name+' wpt = '+wpt_id);
			if (waypoint_modif[wpt_id]!=null)
			{
				addMsgLog('waypoint ' + wpt_id+' for drone '+name+ ' has been moved to '+lat+"//"+lon+' !');
				clearTimeout(waypoint_modif[wpt_id].timer)
				waypoint_modif[wpt_id] = null;
				DOMImplementation("upload/"+planes[selected_index]["id"]+"/flight_plan.xml",fplDisplay);
				// inform other web client that waypoint can be changed
				p_publish('/client_action', 'login',usr_login,'action', 'unlock', 'type_obj', 'waypoint', 'id_obj',wpt_id,'webiddrone',acft_id);
			}
		}
		//document.getElementById("info").innerHTML='waypoint' + wpt_id+' for drone '+name+ ' has been moved !';
		//addMsgLog('waypoint' + wpt_id+' for drone '+name+ ' has been moved !');
		//alert('<br>waypoint' + wpt_id+' for drone '+name+ ' has been moved !');
	}

	function orderprocessing_jump2block(event) {
		var acft_id    = event.get('aircraftId');
		var current_block_id = event.get('currentBlockId');
		if (acft_id==selected_plane_id){
			/// maj de l'affichage fpl
			var name_block=blocks[current_block_id]["name"];
			var name_drone=name_from_id(acft_id);
			clearTimeout(block_jump_timeout);
			active_block_id = current_block_id;
			active_block_name =name_block;
			// inform other web client that block can be changed
			p_publish('/client_action', 'login',usr_login,'action', 'unlock', 'type_obj', 'block', 'id_obj',current_block_id,'webiddrone',acft_id);
			addMsgLog('drone '+ name_drone + ' has jump to block '+ name_block +' !');
			DOMImplementation("upload/"+planes[selected_index]["id"]+"/flight_plan.xml",fplDisplay);			
		}
	}

	function orderprocessing_setting(event) {
		var acft_id    = event.get('aircraftId');
		var setting_id = parseInt(event.get('settingId'));
		var value      = event.get('settingValue');
		if ((acft_id==selected_plane_id)&&(setting_to_change_id==setting_id)){
			clearTimeout(setting_change_timeout);
			setting_to_change_id = -1; // reset
			var name=name_from_id(acft_id);
			//DOMImplementation("upload/"+planes[selected_index]["id"]+"/settings.xml",settingsDisplay);
			selected_plane_update(acft_id);
			addMsgLog('setting ' + setting_id +' for drone '+ name + ' has been changed to value '+ value +' !');
			// inform other web client that block can be changed
			p_publish('/client_action', 'login',usr_login,'action', 'unlock', 'type_obj', 'setting', 'id_obj',setting_id,'webiddrone',acft_id);
		}
	}


	function orderprocessing_planedie(event){
		var acft_id = event.get('aircraftId');
		var order_string   = event.get('order');
		var name=name_from_id(acft_id);
		//document.getElementById('info').innerHTML='drone ' + name+' is not alive !';
		addMsgLog('drone ' + name+' is not alive !');
		var drone_index = seekIndex(acft_id);
		if(drone_index!=-1)
		{
			droneStateDieEvent[drone_index] = setTimeout ( "droneDieCallback("+acft_id+")", dieEventTimeoutTime );
			// on grise l'aircraft sur l'interface
		}
	}
	
	function orderprocessing_planeressurect(event){
		var acft_id = event.get('aircraftId');
		var order_string = event.get('order');
		var name=name_from_id(acft_id);
		//document.getElementById('info').innerHTML='drone ' +name+' has been resurrected !';
		addMsgLog('drone ' +name+' has been resurrected !');
		var drone_index = seekIndex(acft_id);
		if(drone_index!=-1)
		{
			clearTimeout ( droneStateDieEvent[drone_index] );
			// on remet l'aircraft en couleur 
		}
	}

	function orderprocessing_newplane(event){
		var acft_id = event.get('aircraftId');
		var order_string   = event.get('order');
		var name= event.get('aircraftName');
		//document.getElementById('info').innerHTML='drone ' +name+' has been connected !';
		addMsgLog('drone ' +name+' has been connected !');
	}


	function orderprocessing_planekilled(event){
		var acft_id = event.get('aircraftId');
		var name=name_from_id(acft_id);
		addMsgLog("drone " + name + " deconnection caused by death of ivy bus...");
		//document.getElementById('info').innerHTML="drone " + name + " deconnection caused by death of ivy bus...";
		// remove drone...markers....
		removeDroneDisplay(acft_id);
	}

	function orderprocessing_csv_settings(event){
		var acft_id = event.get('aircraftId');
		var csv = event.get('csv');
		var index_drone = seekIndex(acft_id);
		var name=name_from_id(acft_id);
		if (index_drone!=-1) {
			planes[index_drone]["csv"]= csv;
			//addMsgLog("settings updated for drone " + name);
			if(index_drone==selected_index)
			{
				DOMImplementation("upload/"+planes[selected_index]["id"]+"/settings.xml",settingsDisplay);
			}
		}
	}

	function orderprocessing_chat(event){
		}

	function createLock(id_drone,typ_obj,id_obj,login){
		this.id_drone = id_drone;
		this.type_object = typ_obj;
		this.id_object = id_obj;
		this.request_login = login;
		this.lockmsg = function()
			{return ("user "+this.request_login+ " has locked "+this.type_object +" " +this.id_object);}
		this.unlockmsg = function()
			{return ("user "+this.request_login+ " has unlocked "+this.type_object +" " +this.id_object);}
	}
	
	var external_waypoint_lock = new Array();
	var external_block_lock    = new Array();
	var external_setting_lock  = new Array();
	// set or remove a lock to prevent user to change sthg taht is to be changed by another user
	function orderprocessing_client_action(event){
		var rqst_login    = event.get('login');
		var rqst_action   = event.get('action');
		var rqst_obj_type     = event.get('type_obj');
		var rqst_obj_id   = event.get('id_obj');
		var rqst_id_drone = event.get('webiddrone');
		if (rqst_action=='lock'){
			var lock_obj = new createLock(rqst_id_drone,rqst_obj_type,rqst_obj_id,rqst_login);
			if (rqst_obj_type=='waypoint') {external_waypoint_lock.push(lock_obj);}
			else if (rqst_obj_type=='block') {external_block_lock.push(lock_obj);}
			else if (rqst_obj_type=='setting') {external_setting_lock.push(lock_obj);}
			addMsgLog(lock_obj.lockmsg());	
		}
		else if (rqst_action =='unlock'){
			var trouve = false;
			var cur_id_obj=-1;
			var i=0;
			if (rqst_obj_type=='waypoint') 
			{
				while((!trouve)&&(i<external_waypoint_lock.length)){
					cur_id_obj = external_waypoint_lock[i].id_object ;
					if (rqst_obj_id==cur_id_obj) 
					{
						trouve =true ;
						addMsgLog(external_waypoint_lock[i].unlockmsg());
						external_waypoint_lock.splice(i,1);
						
					}
					i++;
				}
				if (rqst_id_drone==selected_plane_id) DOMImplementation("upload/"+planes[selected_index]["id"]+"/flight_plan.xml",fplDisplay);
			}
			else if (rqst_obj_type=='block')
			{
				while((!trouve)&&(i<external_block_lock.length)){
					cur_id_obj = external_block_lock[i].id_object ;
					if (rqst_obj_id==cur_id_obj) 
					{
						trouve =true ;
						addMsgLog(external_block_lock[i].unlockmsg());
						external_block_lock.splice(i,1);
					}
					i++;
				}
				if (rqst_id_drone==selected_plane_id) DOMImplementation("upload/"+planes[selected_index]["id"]+"/flight_plan.xml",fplDisplay);
			}
			else if (rqst_obj_type=='setting') 
			{
				while((!trouve)&&(i<external_setting_lock.length)){
					cur_id_obj = external_setting_lock[i].id_object ;
					if (rqst_obj_id==cur_id_obj) 
					{
						trouve =true ;
						addMsgLog(external_setting_lock[i].unlockmsg());
						external_setting_lock.splice(i,1);
					}
					i++;
				}
				if (rqst_id_drone==selected_plane_id) DOMImplementation("upload/"+planes[selected_index]["id"]+"/settings.xml",settingsDisplay);
			}			
		}
		
	}
	
/* ***************gestion des timeout **************** */
	function droneDieCallback(acft_id){
		var drone_index = seekIndex(acft_id);
		if(drone_index!=-1)
		{
		}
	}


   	function nodataEventCallBack(acft_id){
   		var drone_index = seekIndex(acft_id);
		if(drone_index!=-1)
		{
		}
	}
    /* fonction (timeout) declenchée en cas de non A/R d'une action move_waypoint*/
	function callback_waypoint(ac_id,wpt_id){
		if (ac_id==selected_plane_id) {
			var name=name_from_id(ac_id);
			map.removeOverlay(waypoints[wpt_id]["marker"]);
			waypoints[wpt_id]["marker"] = waypoint_modif[wpt_id].marker;
			map.addOverlay(waypoints[wpt_id]["marker"]);
			waypoint_modif[wpt_id] =null;
			addMsgLog("Waypoint " + wpt_id + " for drone " + name + " has not been moved");
			// inform other web client that waypoint can be changed
			p_publish('/client_action', 'login',usr_login,'action', 'unlock', 'type_obj', 'waypoint', 'id_obj',wpt_id,'webiddrone',ac_id);
			//document.getElementById('info').innerHTML="Waypoint " + wpt_id + " for drone " + name + " has not been moved";
		}
	}

	function callback_block(ac_id,block_id){
		if (ac_id==selected_plane_id) {
			var name_drone = name_from_id(ac_id);
			var name_block = blocks[block_id]["name"];
			setting_to_change_id = -1; // reset
			addMsgLog("jump to block "+ name_block +" for drone " + name_drone +" not performed ... retry please");
			// inform other web client that block can be changed
			p_publish('/client_action', 'login',usr_login,'action', 'unlock', 'type_obj', 'block', 'id_obj',block_id,'webiddrone',ac_id);
		}
	}

	function callback_setting(ac_id,setting_id){
		if (ac_id==selected_plane_id) {
			var name_drone = name_from_id(ac_id);
			addMsgLog("change setting "+ setting_id +" request for drone " + name_drone +" not performed ... retry please");
			// inform other web client that block can be changed
			p_publish('/client_action', 'login',usr_login,'action', 'unlock', 'type_obj', 'setting', 'id_obj',setting_id,'webiddrone',ac_id);
		}
	}
/* ******************************************* */	
	function removeDroneDisplay(droneid){
		var drone_index = seekIndex(droneid);
		if(drone_index!=-1)
		{		
			if (droneStateDieEvent[drone_index]!=-1) clearTimeout ( droneStateDieEvent[drone_index] ); //
			if (droneStateDataEvent[drone_index]!=-1) clearTimeout ( droneStateDataEvent[drone_index] ); //
			planes.splice(drone_index,1);
			nb_planes=planes.length;
			map.removeOverlay(markers[drone_index]);
			map.removeOverlay(markers_color[drone_index]);
			markers.splice(drone_index,1);
			markers_color.splice(drone_index,1);
			//document.getElementById("aircraftList").options.splice(drone_index,1);
			document.getElementById("aircraftList").remove(drone_index);
			// si le drone deconnecté etait celui connecté on met à jour les parametres affichés
			if ((drone_index==selected_plane_id)&&(planes.length!=0))
			{
				active_block_id=planes[0]["activeBlock"];
				aircraftList.options[0].selected=true;
				selected_plane = planes[0]["name"];
				selected_plane_id = planes[0]["id"];
				DOMImplementation("upload/"+planes[0]["id"]+"/flight_plan.xml",fplDisplay);
				DOMImplementation("upload/"+planes[0]["id"]+"/settings.xml",settingsDisplay);
				selected_plane_update(selected_plane_id);
			}
			if (planes.length==0)
			{
				
				document.getElementById("active_block").innerHTML="none ...";     
				document.getElementById("FlightPlan").innerHTML="none ...";
				reinit_param_panel();
				fpl_tree=null;index = 0;
				setting_tree=null;index_setting_node=0;
				for(var i =0; i < waypoints.length;i++)
				{
				 map.removeOverlay(waypoints[i]["marker"]);
				}
				// inutile ?
				if (waypoints.length!=0){
					for(var i =0; i < waypoints.length;i++)
					{
					 map.removeOverlay(waypoints[i]["marker"]);
					}
					waypoints=new Array();
					index_wpt = 0;
				}
				if (blocks.length!=0){
					blocks=new Array();
					index_block = 0;
				}
				
			}			
		}	
	}

	
	
/* ******************* ********************* */
	function orders_checkXHR(){
		var xhr = getXMLHttpRequest();
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4 && xhr.status == 200) {
				orders_processing(xhr.responseXML);
			}
		};
		xhr.open("GET", "orders_check.php", true);
		xhr.send(null);
	}
				
			
	function orders_processing(data){
		var orders=data.getElementsByTagName('orders').item(0);
		if (orders.hasChildNodes()){
			var wpt_orders=data.getElementsByTagName('wpt_orders').item(0);
			if (wpt_orders.hasChildNodes()){
				var wpts=data.getElementsByTagName('wpt_order');
				var wpt_id=0;
				var acft_id=0;
				var lat=0;
				var lon=0;
				var alt=0;
				for (var i=0;i< wpts.length;i++){
					wpt_id=wpts[i].getAttribute("wpt_id");
					acft_id=wpts[i].getAttribute("acft_id");
					lat=wpts[i].getAttribute("lat");
					lon=wpts[i].getAttribute("lon");
					alt=wpts[i].getAttribute("alt");
					waypoint_moved(acft_id,wpt_id,lat,lon,alt);
				}
			}
			
			var settings_orders=data.getElementsByTagName('settings_orders').item(0);
			if(settings_orders.hasChildNodes()){
				var settings=data.getElementsByTagName('settings_order');
				var acft_id=0;
				var setting_id=0;
				var value=0;
				for (var i=0;i< settings.length;i++){
					acft_id=settings[i].getAttribute("acft_id");
					setting_id=settings[i].getAttribute("setting_id");
					value=settings[i].getAttribute("value");
					setting_changed(acft_id,setting_id,value);
				}
			}
			
			
			var status_orders=data.getElementsByTagName('status_orders').item(0);
			if (status_orders.hasChildNodes()){
				var stat_orders=status_orders.childNodes;
				var acft_id=0;
				var order_string="";
				for (var i=0;i< stat_orders.length;i++){
					acft_id=stat_orders[i].getAttribute("acft_id");
					order_string=stat_orders[i].getAttribute("order");
					var name=name_from_id(acft_id);
					if (order_string=="Plane_Die"){
						if (name!=null){
							document.getElementById('acft_status').innerHTML=name+' has been disconnected !';
							pause(1);
						}
					}else if (order_string=="Plane_Resurect"){
						if (name!=null){
							document.getElementById('acft_status').innerHTML=name+' has been resurrected !';
							pause(1);
						}
					}
				}
			}else{
				document.getElementById('acft_status').innerHTML='';
			}
			
		}else{
			document.getElementById('acft_status').innerHTML='';
		}
	}
</script>

</head>

<body onload="backButtonOverride();initialize();" onunload="GUnload()">



<div id="welcome">
-------- Paparazzi On the Web ! --------
<div id="usr">
Welcome, 
<% 
   String log = (String) session.getAttribute("login");
	if(log != null) {
		out.print(log + "!");	
		String rights = (String) session.getAttribute("rights");
		if (rights.equals("admin")){
			out.println(" | <a href=\"admin.jsp\">admin page</a>");
		}
		out.println(" | <a href=\"#\" onclick=\"javascript:initialize_push_pullmode()\">\"pull\" mode</a>");
		out.println(" | <a href=\"logOut.jsp\">log out</a>");
	}
	else
	{ 
		out.println("<script type=\"text/javascript\">window.location.href=\"idError.html\";</script>");
	}
	
%>
	
</div>
</div>
 
<div id="page">

<div id="choicePanel">

<div id="aircraftSelector">
<form id="aircraftForm" method="post" action="">
	<label for="aircraftList">Choose an aircraft:</label>
	<select name="aircraftList", id="aircraftList"  , onChange="plane_focus(this)">
	</select>
</form>
</div>

<div id="trackdiv">
	<form id="tracking" method="post" action="">
		<p>
		<input type="checkbox" name="tracking" onChange="track(this)"/> 
			<label for="tracking">activate tracking</label><br />
		</p>
	</form>
</div>

<div id="active_block">loading...</div>

</div>

<div id="FlightPlan">
loading...
</div>



<div id="FlightParams">
<center>Flight Parameters</center>
<table class="param">
  <col />
  <col />
  <tbody>
    <tr>
      <td class="param">altitude (m)</td>
      <td class="param" id="altitude">0</td>
	</tr>
	<tr>
      <td class="param">height</td>
      <td class="param" id="height">0 m</td>
	</tr>
    <tr>
      <td class="param">battery level</td>
      <td class="param" id="battery">0V</td>
    </tr>
    <tr>
      <td class="param">speed</td>
      <td class="param" id="speed">0 km/h</td>
	</tr>
    <tr>
      <td class="param">GPS status</td>
      <td class="param" id="GPS">OFF</td>
    </tr>
    <tr>
      <td class="param">vertical speed</td>
      <td class="param" id="vspeed">0 m/s</td>
	</tr>
    <tr>
      <td class="param">engine power</td>
      <td class="param" id="engine">0%</td>
    </tr>
  </tbody>
</table>
</div>




<div id="map_canvas" >
</div>

<div id="Settings">
	<div id="page_setting">
		<div id="setting">
		 	<div id="tabs_0">
	 			<ul id="tabs_0_ul">
	 			</ul>
	 		</div>
	 	</div>
	 </div>
</div>
<!-- 
<div id="Debug">
<form name="debugEventDisplay">
  <table border="2" bordercolor="white" cellpadding="4" cellspacing="0" >
    <tr>
     <td>
       <textarea cols="60" rows="16" name="event">
        NO DATA (idle)
       </textarea>
     </td>
    </tr>
  </table>
</form>
</div>
 -->
<div id="NewsDiv">
<div id="info">no news.. </div>
</div>

<!--
<div id="Settings">
<form name="debugEventDisplay">
  <table border="2" bordercolor="white" cellpadding="4" cellspacing="0" >
    <tr>
     <td>
       <textarea cols="60" rows="16" name="event">
        NO DATA (idle)
       </textarea>
     </td>
    </tr>
  </table>
</form>
</div>

-->

</div> <!-- fin de page  --> 


<!-- ------------ PUSHLET purpose------------ -->


<% 
 out.println("<script type=\"text/javascript\">p_embed()</script>");
%>


<%-- <script type="text/javascript">p_embed()</script> --%>
</body>
</html>