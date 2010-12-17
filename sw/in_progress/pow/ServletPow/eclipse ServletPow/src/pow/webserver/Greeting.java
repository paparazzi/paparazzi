package pow.webserver;

import java.io.File;
import java.io.PrintWriter;
import java.io.IOException;
import javax.servlet.ServletConfig;
import javax.servlet.ServletException;
import javax.servlet.http.*;
import java.util.Iterator;

/**
 * Servlet implementation class Greeting
 * Handles the connection request from a new web client
 * check the login's information and rights
 */
public class Greeting extends HttpServlet {
	private static final long serialVersionUID = 1L;
    private String default_folder="blabla";
	
	public void init(ServletConfig config) throws ServletException {
    	super.init(config); // needed otherwise getServletContext return null in doPost method
    	default_folder = config.getServletContext().getRealPath("");
    	// create a default web user file if it does not exist
    	File def_user_file = new File(default_folder + "/conf/"+"userTable.tbl");
    	if (!def_user_file.exists()){
    		User usr0 = new User("admin","pwdadmin",Rights.ADMIN);
    		User usr1 = new User("user1","pwduser1",Rights.USER);
    		usr1.addDrone("MJ5");
    		usr1.addDrone("TJ1");
            UserTab logTab = new UserTab();
            try{
            	logTab.insert(usr0);
    	        logTab.insert(usr1);
            }
            catch(AlreadyRegisteredUserException e) {}    
            logTab.serialize(default_folder + "/conf/"+"userTable.tbl");
    	}       
    }
	
	public Greeting(){
		super();
		
	}

	/**
	 * handles web user login request
	 */
	protected void doPost(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {		
		// jsp session
		HttpSession session = request.getSession(true);
		String login = request.getParameter("login");
		String password;
		response.setContentType("text/html;charset=UTF-8");	
		PrintWriter out = response.getWriter();
		if (login ==null)
		{
			login = "VISITOR";
			password=null;
			session.setAttribute("login",login);
			session.setAttribute("rights","visitor");
			// ok start the interface
			out.println("<script type=\"text/javascript\">window.location.href=\"Interface.jsp\";</script>");
		}
		else {
			// on verifie la validite du mot de passe
			login = request.getParameter("login").toString();
			password = request.getParameter("password").toString();
			UserTab logTab = UserTab.unserialize(default_folder + "/conf/"+"userTable.tbl");
			if (logTab.checkUser(login, password)){
				
				Rights r  = logTab.seek(login).getRights();
				session.setAttribute("login",login);
				if (r==Rights.ADMIN){
					session.setAttribute("rights","admin");
				}
				else if (r==Rights.USER) {
					session.setAttribute("rights","user");
					Iterator<String> itr = logTab.getItrUsr(login);
					String dronenameallowedtobectl="";
					if (itr!=null){
						while(itr.hasNext()){
							dronenameallowedtobectl+=itr.next()+";";
							// transmettre la liste des avions a la page web
						}
					}
					session.setAttribute("dronectl",dronenameallowedtobectl);
				}
				else {
					session.setAttribute("rights","visitor");// inutile normalement
				}
				// ok on lance l'interface
				out.println("<script type=\"text/javascript\">window.location.href=\"Interface.jsp\";</script>");
			}
			else {
				// we send the error page
				out.println("<script type=\"text/javascript\">window.location.href=\"idError.html\";</script>");
			}			
		}
        out.close();
	}
}
