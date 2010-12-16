package pow;

import java.io.File;
import java.io.PrintWriter;
import java.io.IOException;

import javax.servlet.ServletConfig;
import javax.servlet.ServletException;
import javax.servlet.http.*;
import java.net.*;
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
    	super.init(config); // necessaire sinon getServletContext renvoie null dans doPost
    	default_folder = config.getServletContext().getRealPath("");
    	// creation repertoire conf
    	/*
    	String filepath = default_folder + "/conf";
    	File repository = new File(filepath);
    	deleteDirectory(repository);
		repository = new File(filepath);
		if ((!repository.exists()) && (!repository.isDirectory())) {
			repository.mkdir(); // exists isDirectory() 
		}
		*/
		// creation d'un fichier d'user par defaut si n'existe pas
    	File def_user_file = new File(default_folder + "/conf/"+"userTable.tbl");
    	if (!def_user_file.exists()){
    		User usr0 = new User("admin","pwdadmin",Rights.ADMIN);
    		User usr1 = new User("toto","pwdtoto",Rights.ADMIN);
            User usr2 = new User("tata","pwdtata",Rights.USER);
            User usr3 = new User("titi","pwdtiti",Rights.USER);
            usr2.addDrone("MJ5");
            usr3.addDrone("TJ1");
            usr3.addDrone("MJ5");
            UserTab logTab = new UserTab();
            try{
            	logTab.insert(usr0);
    	        logTab.insert(usr1);
    	        logTab.insert(usr2);
    	        logTab.insert(usr3);
            }
            catch(AlreadyRegisteredUserException e) {}    
            logTab.serialize(default_folder + "/conf/"+"userTable.tbl");
    	}       
    }
	
	public Greeting(){
		super();
		
	}

	/**
	 * @see HttpServlet#doGet(HttpServletRequest request, HttpServletResponse response)
	 */
	protected void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
		
	}

	/**
	 * @see HttpServlet#doPost(HttpServletRequest request, HttpServletResponse response)
	 */
	protected void doPost(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {		
		  // va chercherla session correspondant à la requete et la crée sinon (param true)
		  HttpSession session = request.getSession(true);
		  //Cookie c = new Cookie("nom","valeur");
		  //response.addCookie(MonCookie);		
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
			// ok on lance l'interface
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
				// on renvoie une page d'erreur de login
				out.println("<script type=\"text/javascript\">window.location.href=\"idError.html\";</script>");
			}
			
		}
        out.close();
	}

	
	static private boolean deleteDirectory(File path) { 
        boolean resultat = true; 
        
        if( path.exists() ) { 
                File[] files = path.listFiles(); 
                for(int i=0; i<files.length; i++) { 
                        if(files[i].isDirectory()) { 
                                resultat &= deleteDirectory(files[i]); 
                        } 
                        else { 
                        resultat &= files[i].delete(); 
                        } 
                } 
        } 
        resultat &= path.delete(); 
        return( resultat ); 
} 
}
