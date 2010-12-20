<%@page import="pow.webserver.Conf" %>

<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN"
      "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">
<html xmlns="http://www.w3.org/1999/xhtml">
<head>
  	<meta http-equiv="content-type" content="text/html; charset=iso-8859-1" />
 	<title>Welcome to Paparazzi On the Web</title>
	<meta http-equiv="Pragma" content="no-cache"/>
	<meta http-equiv="Cache-Control" content="no-cache, must-revalidate" />
	<meta http-equiv="Expires" content="-1" />
  	<link rel="stylesheet" media="screen" type="text/css" title="Design" href="./CSS/designHelp.css" />
  	<link rel="shortcut icon" type="image/x-icon" href="Icons/favicon.ico" />
</head>
<body>


<h2>Welcome on Paparazzi On the Web help page.</h2>
<p> This page is here to help you to use the Paparazzi On the Web (POW) application. POW is a part of Paparazzi which is a open-project of civil UAV control. With Paparazzi On the Web, you are able to observe and eventually control the UAVs of the system Paparazzi thanks to a browser with your internet connexion. To learn more about Paparazzi project, <a href=http://paparazzi.enac.fr>click here</a>.</p>
<p>Paparazzi On the Web is optimised for Firefox browser. If you want to have a better performance, you can dowload Firefox <a href=http://www.mozilla-europe.org/en>here</a>.</p>
<h3>Start page</h3>
<p>If you have a login and a password on Paparazzi On the Web, you can use them to enter the application. Else, click on enter as a guest.</p>

<h3>Main page</h3>
<p>You are now on Paparazzi On the Web main page. You can see several boxes. We will describe all those boxes and show you the interactions you can have with them.</p>

<h3>Choose an aircraft</h3>
<p>Here you can select the aircraft you want to observe or control. Click on the arrow to see the list of the aircrafts that are available. Click on the aircraft you want to interacte with.</p>
<p>You can also choose to activate or desactivate tracking. When tracking is activated, the map is always centered on the selected aircraft.</p>



<h3>The map</h3>
<p>On the map, you can see all the aircrafts that are available. If tracking is activated, the map will be centered on the selected aircraft. There are also red lozenges which are the waypoints of the selected UAV. If you are autentified on Paparazzi On the Web and if your profile gives you the authorization, you can move those waypoints with drag and drop.<p/>

<h3>The flight plan</h3>
<p>Through this box, you can obtain some informations on the flight plan that is followed by the selected plane. If you move your cursor on the informations tab, you obtain informations on the flightplan. If you put your cursor on the "active block" tab, you can see the list of the flight plan blocks that the plane can follow. The active block is in red. If you are authentified and if you have the authorization, you can change the active block by clicking on its name. The waypoints tab gives you informations on the different waypoints. If you are authentified and if you have the authorization, you can move those waypoints by clicking on "move waypoint". In this case, a new windows is openned where you indicate where you want to move the waypoint.
You can also move a waypoint by dragging the icon on the map to its new position.<p/>

<h3>The flight parameters</h3>
<p>This box displays the flight parameters of the selected aircraft. When a value is out of its normal range, it becomes red.</p>

<h3>The settings</h3>
<p>In this box you can, if you are allowed to control the selected aircraft, you change some settings of this aircraft by clicking on the name of the setting.</p>

<h3 id="problem">Any problem ?</h3>
<p>If you have any display problem, such as a ghost aircraft icon, or an aircraft that is not displayed in the aircrafts list, you might resolve it by reloading the page by pressing F5 key.
For any other problem, please  inform 
<%
out.print("<a href=\"mailto:"); 
String default_folder = this.getServletConfig().getServletContext().getRealPath("");
Conf myconf =new Conf(default_folder,"pow_conf.xml");
out.print(myconf.mailAdmin()); 
out.print("\">the administrator</a>"); %>.</p>
<a href="index.jsp">Click here</a> to return to the homepage.

</body>
</html>
