<%@ page language="java" contentType="text/html; charset=ISO-8859-1"
    pageEncoding="ISO-8859-1"%>
<%@page import="java.util.*,pow.webserver.UserTab,pow.webserver.User,javax.xml.parsers.*, org.w3c.dom.*,org.xml.sax.*,java.io.*" %>   
<% 
// recuperation des données sur le serveur
    String default_folder = this.getServletContext().getRealPath("");
	UserTab logTab = UserTab.unserialize(default_folder + "/conf/"+"userTable.tbl");
	Iterator<String> itr=logTab.getLoginIterator();
	//
	UserTab logIvyTab = UserTab.unserialize(default_folder + "/conf/"+"userIvyTable.tbl");
	Iterator<String> itrIvy=logIvyTab.getLoginIterator();
	// lecture liste des noms de drones ds immat.xml
	DocumentBuilderFactory fabrique = DocumentBuilderFactory.newInstance();
	// création d'un constructeur de documents
	DocumentBuilder constructeur = fabrique.newDocumentBuilder();
	// lecture du contenu d'un fichier XML avec DOM
	File xml = new File(default_folder + "/conf/"+"immat.xml");
	Document document = constructeur.parse(xml);
	

%>

<html>
<head>
<meta http-equiv="Content-Type" content="text/html; charset=ISO-8859-1">
<meta http-equiv="Pragma" content="no-cache"/>
	<meta http-equiv="Cache-Control" content="no-cache, must-revalidate" />
	<meta http-equiv="Expires" content="-1" />
<title>Administration page</title>
<script type="text/javascript" src="./js/tabber.js"></script>

<link rel="stylesheet" href="./CSS/tab.css" TYPE="text/css" MEDIA="screen">
<link rel="stylesheet" href="./CSS/design_admin.css" TYPE="text/css" MEDIA="screen">

<script type="text/javascript" src="./js/XHR_object.js"></script>


<script type="text/javascript">
var tab_user = new Array();
var ivy_user = new Array();
var dronename_liste = new Array();
var ajax_url = "ajaxRqst.srv";
//var user_selected_index = liste_user.selectedIndex;
<%
    String login;
	int i = 0;
    while (itr.hasNext()){
		login = itr.next();   
		out.println("tab_user["+i+"] = '"+ login +"';");
		i++;
   }
    String ivyusr;
	i = 0;
    while (itrIvy.hasNext()){
    	ivyusr = itrIvy.next();   
		out.println("ivy_user["+i+"] = '"+ ivyusr +"';");
		i++;
   }
    NodeList dronenameList = document.getElementsByTagName("drone");
    for(i=0; i<dronenameList.getLength(); i++){
		Element e = (Element)dronenameList.item(i);
		out.println("dronename_liste["+i+"] = '"+ e.getAttribute("name") +"';");
	}
   
%>
// verif de robustesse
function passwordChanged(pwdid,strengthid) {
	var strength = document.getElementById(strengthid);
	var strongRegex = new RegExp("^(?=.{8,})(?=.*[A-Z])(?=.*[a-z])(?=.*[0-9])(?=.*\W).*$", "g");
	var mediumRegex = new RegExp("^(?=.{7,})(((?=.*[A-Z])(?=.*[a-z]))|((?=.*[A-Z])(?=.*[0-9]))|((?=.*[a-z])(?=.*[0-9]))).*$", "g");
	var enoughRegex = new RegExp("(?=.{6,}).*", "g");
	var pwd = document.getElementById(pwdid);
	if (pwd.value.length==0) {
	strength.innerHTML = 'Type Password';
	} else if (false == enoughRegex.test(pwd.value)) {
	strength.innerHTML = 'More Characters';
	} else if (strongRegex.test(pwd.value)) {
	strength.innerHTML = '<span style="color:green">Strong!</span>';
	} else if (mediumRegex.test(pwd.value)) {
	strength.innerHTML = '<span style="color:orange">Medium!</span>';
	} else {
	strength.innerHTML = '<span style="color:red">Weak!</span>';
	}
}



function validatePwd(pwd1id,pwd2id,pwdreturn) {
	var pwdok = document.getElementById(pwdreturn);
	var pw1 = document.getElementById(pwd1id).value;
	var pw2 = document.getElementById(pwd2id).value;
	if (pw1 != pw2) {
		pwdok.innerHTML="not ok";
		return false;
	}
	else {
		pwdok.innerHTML="ok";
		return true;
	}
}

function validatePwdChange() {
	var pwdok = document.getElementById('verifnewpwd');
	var pw1 = document.getElementById("modifPwdForm").newpwd.value;
	var pw2 = document.getElementById("modifPwdForm").newpwd_check.value;
	if (pw1 != pw2) {
		pwdok.innerHTML="not ok";
		return false;
	}
	else {
		pwdok.innerHTML="ok";
		return true;
	}
}
// initialize the different list and the tabs
function init()
{
	var listUser = document.getElementById("mainForm").liste_user;
	for(var i= 0 ; i< tab_user.length;i++){
		listUser.options[i]=new Option(tab_user[i]);
	}
	//
	var listIvyUser = document.getElementById("listIvyForm").liste_ivy;
	for(var i= 0 ; i< ivy_user.length;i++){
		listIvyUser.options[i]=new Option(ivy_user[i]);
	}
	//
	var listImmat = document.getElementById("createUserForm").liste_immat;
	var listImmatAdmin = document.getElementById("listImmatForm").liste_immat_admin;
	for(var j= 0 ; j< dronename_liste.length;j++){
		listImmat.options[j]     = new Option(dronename_liste[j]);
		listImmatAdmin.options[j]= new Option(dronename_liste[j]);
	}
	init_tabs();
}
//

function get_create_user_form(){
	document.getElementById('main').style.visibility='hidden';
	document.getElementById('createUser').style.visibility='visible';
}

function remove_create_user_form(){
	document.getElementById('main').style.visibility='visible';
	document.getElementById('createUser').style.visibility='hidden';
	reset_create_user_form();
}

function reset_create_user_form(){
	document.getElementById("pwd").value="";
	document.getElementById("login").value="";
	document.getElementById("radio_admin").checked=false;
	document.getElementById("radio_user").checked=false;
	document.getElementById("radio_visitor").checked=true;
	var listImmat = document.getElementById("createUserForm").liste_immat;
	for(var j= 0 ; j< listImmat.options.length;j++){
		listImmat.options[j].selected = false;
	}
}

function create_user(){
	if (!validatePwd('pwd','pwd_check','verifpwd')) {alert('check your password please');return;}
	//Ajax request to edit the flight plan xml file
	var order = "order=create_user";
	var login = document.getElementById("login").value;
	var pwd = document.getElementById("pwd").value;
	var right;
	var i = 0 ;
	while(!document.getElementById("createUserForm").right[i].checked) {i++;}
	right = document.getElementById("createUserForm").right[i].value;
	order = order+"&login="+login+"&pwd="+pwd+"&right="+right;

	var listImmat = document.getElementById("createUserForm").liste_immat;
	var liste_immat_selected="";
	if(right=="user")
	{
		for(var j= 0 ; j< listImmat.options.length;j++){
			if (listImmat.options[j].selected){
				liste_immat_selected = liste_immat_selected+dronename_liste[j]+";";
			}
		}
		if (liste_immat_selected!="") {
			order = order+"&immats="+liste_immat_selected;
		}
	}
	
	var xhr = getXMLHttpRequest();
	xhr.onreadystatechange = function() {
		if (xhr.readyState == 4 && xhr.status == 200) {
			// A/R
			var xmlResponse = xhr.responseXML.documentElement;
			var rep = xmlResponse.getElementsByTagName("create_user")[0];
			var log = rep.getAttribute("login");
			var status = rep.getAttribute("status");
			if (status == "OK") {
				alert("user "+ log +" created");
			}
			else {
				alert("user "+ log +" not created");
			}
			window.location.href="admin.jsp";
		}
	};
	xhr.open("POST",ajax_url,true);
	xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
	xhr.send(order);
}

function delete_user(){
	var listUser = document.getElementById("mainForm").liste_user;
	var res = confirm('Do you really want to delete user '+ tab_user[listUser.selectedIndex] +' ?')
	if (res){
		var order = "order=delete_user&login="+tab_user[listUser.selectedIndex];
		var xhr = getXMLHttpRequest();
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4 && xhr.status == 200) {
				// A/R
				var xmlResponse = xhr.responseXML.documentElement;
				var rep = xmlResponse.getElementsByTagName("delete_user")[0];
				var log = rep.getAttribute("login");
				var status = rep.getAttribute("status");
				if (status == "OK") {
					alert("user "+ log +" deleted");
				}
				else {
					alert("user "+ log +" not deleted");
				}
				window.location.href="admin.jsp";
			}
		};
		xhr.open("POST",ajax_url,true);
		xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
		xhr.send(order);
	}
}




var usr_log ;
var usr_right;
var usr_array_drone;
var usr_pwd;

function get_modify_user_form(){
	var listUser = document.getElementById("mainForm").liste_user;
    var log = tab_user[listUser.selectedIndex];
    document.getElementById('loginLabel').innerHTML = log;
	
	document.getElementById('main').style.visibility='hidden';
	document.getElementById('modifyUser').style.visibility='visible';
	var order = "order=info_user&login="+tab_user[listUser.selectedIndex];
	var xhr = getXMLHttpRequest();
	xhr.onreadystatechange = function() {
		if (xhr.readyState == 4 && xhr.status == 200) {
			// A/R
			var xmlResponse = xhr.responseXML.documentElement;
			var rep = xmlResponse.getElementsByTagName("info_user")[0];
			usr_log = rep.getAttribute("login");
			usr_right = rep.getAttribute("right");
			var liste_drone = rep.getAttribute("liste_drone");
			usr_array_drone = liste_drone.split(';');
			usr_pwd="";
			// init change right form
			if (usr_right=="admin") {document.getElementById("modifRightForm").modright[0].checked=true;}
			else if (usr_right=="user") {document.getElementById("modifRightForm").modright[1].checked=true;}
			else {document.getElementById("modifRightForm").modright[2].checked=true;}
			// init liste drone form
			var liste = document.getElementById('change_immat');
			var i;
			for (i = 0; i<liste.length; i++) {
			   	liste.remove(i);
			}
			for(i= 0 ; i< dronename_liste.length;i++){
				liste.options[i]=new Option(dronename_liste[i]);
				for(var j =0; j< usr_array_drone.length;j++){
					if (usr_array_drone[j]==dronename_liste[i]) {liste.options[i].selected=true;}
				}
			}
			//
						
		}
	};
	xhr.open("POST",ajax_url,true);
	xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
	xhr.send(order);
	
}

function modify_password(){
	
	document.getElementById('loginLabelPwd').innerHTML = usr_log;
	document.getElementById('modifPwd').style.visibility='visible';
	document.getElementById('modifyUser').style.visibility='hidden';
}
function modify_right(){
	document.getElementById('loginLabelRight').innerHTML = usr_log;
	document.getElementById('modifRight').style.visibility='visible';
	document.getElementById('modifyUser').style.visibility='hidden';
}
function modify_list(){
	if (usr_right=="user"){
	document.getElementById('loginLabelImmat').innerHTML = usr_log;
	document.getElementById('modifImmat').style.visibility='visible';
	document.getElementById('modifyUser').style.visibility='hidden';
	}
	else
	{
		alert('In order to specify drones to be controled, rights should be set to "user"');
	}
}
function validate_change(){
	var usr_liste_drone="";
	for(var i = 0 ; i < usr_array_drone.length;i++){
		usr_liste_drone=usr_liste_drone+usr_array_drone[i]+";";
	}
	//alert("log="+usr_log+" pwd="+usr_pwd+" right="+usr_right+" liste='"+usr_liste_drone+"'");
	// do change
	var order = "order=modify_user&login="+usr_log+"&pwd="+usr_pwd+"&right="+usr_right+"&liste_drone="+usr_liste_drone;
	
	var xhr = getXMLHttpRequest();
	xhr.onreadystatechange = function() {
		if (xhr.readyState == 4 && xhr.status == 200) {
			// A/R
			var xmlResponse = xhr.responseXML.documentElement;
			var rep = xmlResponse.getElementsByTagName("modified_user")[0];
			var log = rep.getAttribute("login");
			//
			alert('user '+log+' account modified');	
			usr_log   = "";
			usr_right = "";
			usr_array_drone = new Array();
			usr_pwd = "";
			document.getElementById('modifyUser').style.visibility='hidden';
			document.getElementById('main').style.visibility='visible';	
		}
	};
	xhr.open("POST",ajax_url,true);
	xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
	xhr.send(order);
	//
	
}
function remove_modify_user_form(){
	usr_log   = "";
	usr_right = "";
	usr_array_drone = new Array();
	usr_pwd = "";
	document.getElementById('modifyUser').style.visibility='hidden';
	document.getElementById('main').style.visibility='visible';
}
function validate_change_right(){
	var i=0;
	while(!document.getElementById("modifRightForm").modright[i].checked) {i++;}
	usr_right = document.getElementById("modifRightForm").modright[i].value;
	document.getElementById('modifRight').style.visibility='hidden';
	document.getElementById('modifyUser').style.visibility='visible';
}
function remove_change_right_form(){
	document.getElementById('modifRight').style.visibility='hidden';
	document.getElementById('modifyUser').style.visibility='visible';
	// reinit
	if (usr_right=="admin") {document.getElementById("modifRightForm").modright[0].checked=true;}
	else if (usr_right=="user") {document.getElementById("modifRightForm").modright[1].checked=true;}
	else {document.getElementById("modifRightForm").modright[2].checked=true;}
}
function validate_change_pwd(){
	if(!validatePwdChange()) {return ;}
	usr_pwd = document.getElementById("modifPwdForm").newpwd.value;
	document.getElementById('modifPwd').style.visibility='hidden';
	document.getElementById('modifyUser').style.visibility='visible';
}
function remove_change_pwd_form(){
	document.getElementById('modifPwd').style.visibility='hidden';
	document.getElementById('modifyUser').style.visibility='visible';
}
function validate_change_immat(){
	if(usr_right=="user")
	{
		var listImmat = document.getElementById("modifImmatForm").change_immat;
		usr_array_drone=new Array() ;var k = 0;
		for(var j= 0 ; j< listImmat.options.length;j++){
			if (listImmat.options[j].selected==true){
				usr_array_drone[k] = listImmat.options[j].value;
				k++;
			}
		}
		document.getElementById('modifImmat').style.visibility='hidden';
		document.getElementById('modifyUser').style.visibility='visible';
	}
}
function remove_change_immat_form(){
	document.getElementById('modifImmat').style.visibility='hidden';
	document.getElementById('modifyUser').style.visibility='visible';
	var liste = document.getElementById('change_immat');
	var i;
	for (i = 0; i<liste.length; i++) {
	   	liste.remove(i);
	}
	for(i= 0 ; i< dronename_liste.length;i++){
		liste.options[i]=new Option(dronename_liste[i]);
		for(var j =0; j< usr_array_drone.length;j++){
			if (usr_array_drone[j]==dronename_liste[i]) {liste.options[i].selected=true;}
		}
	}
}


// fonctions de gestion de la liste des drones pouvant etre controlés


function add_drone(){
	var reg=new RegExp("\s", "g");
	new_name=prompt("New name ? = ", "newdrone"+ dronename_liste.length);
	if ((new_name==null)){
		return;
	}
	new_name.replace(reg,"");
 	var i=0; var trouve=false;
 	while((i<dronename_liste.length)&&(!trouve)){
		if (new_name==dronename_liste[i]) 
			{
			trouve = true;
			alert("this name is already in list");
			}	
		i++;
 	 }
	
	if ((!trouve)&&(new_name!="")){
		var order = "order=add_drone&name="+new_name;
		var xhr = getXMLHttpRequest();
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4 && xhr.status == 200) {
				// A/R
				var xmlResponse = xhr.responseXML.documentElement;
				var rep = xmlResponse.getElementsByTagName("added_drone")[0];
				var name = rep.getAttribute("name");
				var status = rep.getAttribute("status");
				if (status == "OK") {
					alert("drone "+ name +" added to the list");
				}
				else {
					alert("drone "+ name +" not added to the list");
				}
				window.location.href="admin.jsp";
			}
		};
		xhr.open("POST",ajax_url,true);
		xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
		xhr.send(order);
	}
}

function delete_drone(){
	var listDrone = document.getElementById("listImmatForm").liste_immat_admin;
	var res = confirm('Do you really want to remove drone name : '+ dronename_liste[listDrone.selectedIndex] +' ?')
	if (res){
		var order = "order=delete_drone&name="+dronename_liste[listDrone.selectedIndex];
		var xhr = getXMLHttpRequest();
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4 && xhr.status == 200) {
				// A/R
				var xmlResponse = xhr.responseXML.documentElement;
				var rep = xmlResponse.getElementsByTagName("removed_drone")[0];
				var name = rep.getAttribute("name");
				var status = rep.getAttribute("status");
				if (status == "OK") {
					alert("drone "+ name +" deleted from list");
				}
				else {
					alert("drone "+ name +" not deleted from list");
				}
				window.location.href="admin.jsp";
			}
		};
		xhr.open("POST",ajax_url,true);
		xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
		xhr.send(order);
	}	
}
// gestion des ivy users...

function delete_ivy()
{
	var listIvyUser = document.getElementById("listIvyForm").liste_ivy;
	var res = confirm('Do you really want to remove drone name : '+ ivy_user[listIvyUser.selectedIndex] +' ?')
	if (res){
		var order = "order=delete_ivyusr&login="+ ivy_user[listIvyUser.selectedIndex];
		var xhr = getXMLHttpRequest();
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4 && xhr.status == 200) {
				// A/R
				var xmlResponse = xhr.responseXML.documentElement;
				var rep = xmlResponse.getElementsByTagName("removed_ivyusr")[0];
				var name = rep.getAttribute("login");
				var status = rep.getAttribute("status");
				if (status == "OK") {
					alert("ivy user "+ name +" deleted from list");
				}
				else {
					alert("ivy user "+ name +" not deleted from list");
				}
				window.location.href="admin.jsp";
			}
		};
		xhr.open("POST",ajax_url,true);
		xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
		xhr.send(order);
	}
}

function create_ivyuser(){
	if (!validatePwd('ivypwd','ivypwd_check','ivyverifpwd')) {alert('check your password please');return;}
        var log = document.getElementById("ivylogin").value;
    	var pwd = document.getElementById("ivypwd").value;
		var order = "order=add_ivyusr&login="+ log+"&pwd="+ pwd;
		var xhr = getXMLHttpRequest();
		xhr.onreadystatechange = function() {
			if (xhr.readyState == 4 && xhr.status == 200) {
				// A/R
				var xmlResponse = xhr.responseXML.documentElement;
				var rep = xmlResponse.getElementsByTagName("added_ivyusr")[0];
				var name = rep.getAttribute("login");
				var status = rep.getAttribute("status");
				if (status == "OK") {
					alert("ivy user "+ name +" added to list");
				}
				else {
					alert("ivy user "+ name +" not added to list");
				}
				window.location.href="admin.jsp";
			}
		};
		xhr.open("POST",ajax_url,true);
		xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
		xhr.send(order);	
}
function show_create_ivyuser_form(){
	document.getElementById('listIvyUser').style.visibility='hidden';
	document.getElementById('createIvyUser').style.visibility='visible';
}

function remove_create_ivyuser_form(){
	document.getElementById('listIvyUser').style.visibility='visible';
	document.getElementById('createIvyUser').style.visibility='hidden';
	reset_create_ivyuser_form();
}

function reset_create_ivyuser_form(){
	document.getElementById("ivypwd").value="";
	document.getElementById("ivylogin").value="";
}

</script>

</head>
<body onload="init();"> 
<!--see http://www.barelyfitz.com/projects/tabber/ for tabs in javascript-->
<div id="title">
	<a href="Interface.jsp">back to pow</a> | <a href="logOut.jsp">log out</a>
</div>

<div id="page">
<div class="tabber">
<div class="tabbertab" title="web users">

		<div id="main" style="visibility:visible">
			<form id="mainForm" method="post" action="">
				<fieldset><legend><b>Select an item</b></legend>
					<table>
						<tr><td rowspan="3">
							<select name="liste_user"  size=3 onChange="">
			
							</select>
						</td><td></td><td><INPUT type="button" Value="create user" onClick="get_create_user_form()"> </td></tr>
						<tr>
							<td></td><td><input type="button" Value="delete user" onClick="delete_user()"> </td>
						</tr>
						<tr>
							<td></td><td><input type="button" Value="modify user" onClick="get_modify_user_form()"> </td>
						</tr>
					</table>
				</fieldset>
			</form>
		</div>


		
<div id="createUser" style="visibility:hidden">
	<form id="createUserForm">
	<fieldset>
		<legend><b>Create User</b></legend>
		<table>
			 <tr>
			    <td><label for="login">Login :</label></td>
			    <td><input type="text" id="login" name="login"/></td>
			  </tr>
			  <tr>
			    <td><label for="pwd">Password :</label></td>
			    <td><input type="password" id="pwd" name="pwd" size="15" maxlength="20" onkeyup="return passwordChanged('pwd','strength');"/><span id="strength">Type Password</span></td>
			  </tr>
			  <tr>
			    <td><label for="pwd_check">Password Check :</label></td>
			    <td><input type="password" id="pwd_check" name="pwd_check" size="15" maxlength="20" onblur="validatePwd('pwd','pwd_check','verifpwd');"/><span id="verifpwd">not ok</span></td>
			  </tr>	
			  <tr>
			    <td><label for="right">Rights :</label></td>
			    <td>
			    	<input type="radio" name="right" id="radio_admin" value="admin"/> Administrator<br>
					<input type="radio" name="right" id="radio_user" value="user"/> User<br>
					<input type="radio" name="right" id="radio_visitor" value="visitor" checked="checked"/> Visitor
			    </td>
			  </tr>	
			  <tr>
			  		 <td><label for="immat">Controled drones :</label></td>
			  		 <td>
			  		 	<select name="liste_immat"  size="3" multiple="multiple"></select>
			  		 </td>
			  </tr>
			  <tr>
				  <td><input type="button" Value="create user" onClick="create_user()"/></td>
				  <td><input type="button" Value="cancel" onClick="remove_create_user_form()"/></td>
			  </tr>		
		</table>
	</fieldset>
	</form>
</div>


<div id="modifyUser" style="visibility:hidden">
	<form id="modifyUserForm">
	<fieldset>
		<legend><b>Modify User : </b><span id="loginLabel" name="loginLabel">######</span></legend>
		<table>	 
			  <tr>
			    <td>Change Password :</td>
			    <td><input type="button" value="modify password" onClick="modify_password()"/></td>
			  </tr>
			  <tr>
			    <td>Change Rights</td>
			    <td><input type="button" value="modify rights" onClick="modify_right()"/></td>
			  </tr>	
			  <tr>
		  		 <td>Change Controled drones list:</td>
		  		 <td><input type="button" value="modify list" onClick="modify_list()"/></td>
			  </tr>
			  <tr>
				  <td><input type="button" Value="validate change" onClick="validate_change()"/></td>
				  <td><input type="button" Value="cancel" onClick="remove_modify_user_form()"/></td>
			  </tr>		
		</table>
	</fieldset>
	</form>
</div>

<div id="modifRight" style="visibility:hidden">
   <form id="modifRightForm">
   <fieldset>
   <legend><b>Change right for user : </b><span id="loginLabelRight" >######</span></legend>
	<table>
	
 		<tr>
			    <td><label for="modright">Rights :</label></td>
			    <td>
			    	<input type="radio" name="modright" id="modradio_admin" value="admin"/> Administrator<br>
					<input type="radio" name="modright" id="modradio_user" value="user"/> User<br>
					<input type="radio" name="modright" id="modradio_visitor" value="visitor" checked="checked"/> Visitor
			    </td>
		</tr>	
	    <tr>
			<td><input type="button" Value="validate change" onClick="validate_change_right()"/></td>
			<td><input type="button" Value="cancel" onClick="remove_change_right_form()"/></td>
		</tr>	
	</table>
	</fieldset>
	</form>
</div>
<div id="modifPwd" style="visibility:hidden">
	<form id="modifPwdForm">
    <fieldset>
  	 <legend><b>Change Password for user : </b><span id="loginLabelPwd" >######</span></legend>
		<table>
	 		<tr>
			    <td><label for="newpwd">New Password :</label></td>
			    <td><input type="password" id="newpwd" name="newpwd" size="15" maxlength="20" onkeyup="return passwordChanged('newpwd','newstrength');"/><span id="newstrength">Type Password</span></td>
			</tr>
			<tr>
			    <td><label for="newpwd_check">Password Check :</label></td>
			    <td><input type="password" id="newpwd_check" name="newpwd_check" size="15" maxlength="20" onblur="validatePwd('newpwd','newpwd_check','verifnewpwd');"/><span id="verifnewpwd">not ok</span></td>
		    </tr>
		    <tr>
				<td><input type="button" Value="validate change" onClick="validate_change_pwd()"/></td>
				<td><input type="button" Value="cancel" onClick="remove_change_pwd_form()"/></td>
			</tr>	
		</table>
	</fieldset>
	</form>
</div>
<div id="modifImmat" style="visibility:hidden">
<form id = "modifImmatForm">
	<fieldset>
	<legend><b>Change List of Controled Immat for user : </b><span id="loginLabelImmat" >######</span></legend>
	<table>
	
 		<tr>
			<td><label for="change_immat">Controled drones :</label></td>
			<td>
			 	<select name="change_immat" id="change_immat" size="3" multiple="multiple"></select>
			</td>
		</tr>	
	    <tr>
			<td><input type="button" Value="validate change" onClick="validate_change_immat()"/></td>
			<td><input type="button" Value="cancel" onClick="remove_change_immat_form()"/></td>
		</tr>	
	</table>
	</fieldset>
	</form>
</div>
</div><!--  fin tabbertab 1 -->
<div class="tabbertab" title="liste des drones" >
		<div id="listImmat">
		<form id = "listImmatForm">
			<fieldset>
			<legend><b>Add or Remove a name among drones which may be controlled by users</b></legend>
			<table>
						<tr><td rowspan="2">
							<select name="liste_immat_admin"  size=3 onChange="">
					
							</select>
						</td>
							<td></td><td><input type="button" Value="add drone" onClick="add_drone()"> </td></tr>
						<tr>
							<td></td><td><input type="button" Value="delete drone" onClick="delete_drone()"> </td>
						</tr>
					</table>
			</fieldset>
			</form>
		</div>
</div> <!--  fin tabbertab 2 -->
<div class="tabbertab" title="ivy users" >
	<div id="listIvyUser">
		<form id = "listIvyForm">
			<fieldset>
			<legend><b>Add or Remove an Ivy User</b></legend>
			<table>
						<tr><td rowspan="2">
							<select name="liste_ivy"  size=3 onChange="">
					
							</select>
						</td>
							<td></td><td><input type="button" Value="add ivy user" onClick="show_create_ivyuser_form()"> </td></tr>
						<tr>
							<td></td><td><input type="button" Value="delete ivy user" onClick="delete_ivy()"> </td>
						</tr>
					</table>
			</fieldset>
			</form>
		</div>
		
<div id="createIvyUser" style="visibility:hidden">
	<form id="createIvyUserForm">
	<fieldset>
		<legend><b>Create a new Ivy User</b></legend>
		<table>
			 <tr>
			    <td><label for="login">Login :</label></td>
			    <td><input type="text" id="ivylogin" name="ivylogin"/></td>
			  </tr>
			  <tr>
			    <td><label for="pwd">Password :</label></td>
			    <td><input type="password" id="ivypwd" name="ivypwd" size="15" maxlength="20" onkeyup="return passwordChanged('ivypwd','ivystrength');"/><span id="ivystrength">Type Password</span></td>
			  </tr>
			  <tr>
			    <td><label for="pwd_check">Password Check :</label></td>
			    <td><input type="password" id="ivypwd_check" name="ivypwd_check" size="15" maxlength="20" onblur="validatePwd('ivypwd','ivypwd_check','ivyverifpwd');"/><span id="ivyverifpwd">not ok</span></td>
			  </tr>	
			  <tr>
				  <td><input type="button" Value="create user" onClick="create_ivyuser()"/></td>
				  <td><input type="button" Value="cancel" onClick="remove_create_ivyuser_form()"/></td>
			  </tr>		
		</table>
	</fieldset>
	</form>
</div>
</div> <!--  fin tabbertab 3 -->

</div> <!-- fin tabber -->
</div>
</body>
</html>