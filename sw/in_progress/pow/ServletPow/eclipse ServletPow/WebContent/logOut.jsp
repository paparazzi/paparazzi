
<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN"
      "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">
<html xmlns="http://www.w3.org/1999/xhtml">

<head>
  <meta http-equiv="content-type" content="text/html; charset=iso-8859-1" />
  <meta http-equiv="Pragma" content="no-cache"/>
	<meta http-equiv="Cache-Control" content="no-cache, must-revalidate" />
	<meta http-equiv="Expires" content="-1" />
  <title>Log Out</title>
  <link rel="stylesheet" media="screen" type="text/css" title="Design"
  href="./CSS/designLogOut.css" />
  <link rel="shortcut icon" type="image/x-icon" href="Icons/favicon.ico" />
  <!-- NEEDED POUR LE PUSHLET -->
	<script type="text/javascript" src="./lib/js-pushlet-client.js"></script>
	<!-- END NEEDED POUR LE PUSHLET -->
	
		
 </head>
 
<body onload="p_leave()"> <%-- desabonnement du pushlet--%>
<center>
Bye bye <% out.println(session.getAttribute("login")); %> !<br/>
Thank you for using Paparazzi On the Web !<br/>

<% 
session.removeAttribute("login");
session.removeAttribute("rights");
session.invalidate();
%>

<p>
Click <a href=http://paparazzi.enac.fr>here</a> to go to the Paparazzi project homepage.<br/>
Click <a href=index.jsp>here</a> to return to the Paparazzi On the Web homepage.
</p>
</center>
</body>
</html>