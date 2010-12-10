<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN"
      "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">
<html xmlns="http://www.w3.org/1999/xhtml">
<head>
  	<meta http-equiv="content-type" content="text/html; charset=iso-8859-1" />
 	<title>Welcome to Paparazzi On the Web</title>
	<meta http-equiv="Pragma" content="no-cache"/>
	<meta http-equiv="Cache-Control" content="no-cache, must-revalidate" />
	<meta http-equiv="Expires" content="-1" />
  	<link rel="stylesheet" media="screen" type="text/css" title="Design" href="./CSS/designWelcome.css" />
  	<link rel="shortcut icon" type="image/x-icon" href="Icons/favicon.ico" />
  	<script type="text/javascript">
  
	function highlight(field, error){
		if(error)
			field.style.backgroundColor = "#fba";
		else
			field.style.backgroundColor = "";
	}

	function field_verif(field){
		if(field.value.length < 2 || field.value.length > 32){
			highlight(field, true);
			return false;
		}
		else{
			highlight(field, false);
			return true;
		}
	}
	
	function form_verif(f){
		var login_ok = field_verif(f.login);
		var pass_ok = field_verif(f.password);
		return(pass_ok && login_ok);
	}
   </script>

</head>

<body style="text-align:center;margin-left:auto;margin-right:auto;">

<h2>Welcome to Paparazzi On The Web !</h2>

<p></p>

<p></p>
<div id="signIn">
<form action="Greeting.srv" method="post" onsubmit= "return form_verif(this)">
<p>Login : <input id="log", type="text", name="login", onblur = "field_verif(this)" /></p>
<p>Password : <input id="pass", type="password", name="password" onblur= "field_verif(this)" /></p>
<input type="submit" value="Sign in !" />
</form>
<p></p>


</div>
<p></p>
<br/>
<p>or</p>

<p> 
<form action="Greeting.srv" method="post">
<input type="submit" value="Enter as guest" />
</form>
</p>
<a href="help.jsp">help ?</a>
</body>
</html>