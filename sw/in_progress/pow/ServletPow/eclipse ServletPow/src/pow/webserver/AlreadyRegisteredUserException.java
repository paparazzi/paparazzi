package pow.webserver;
/**
 * exception to inform that a user cannot be created in the users' file 
 * because it exists yet
 * @author genin
 *
 */
public class AlreadyRegisteredUserException extends Exception {
	private static final long serialVersionUID = 1L;
	private User usr;
	
	public AlreadyRegisteredUserException (User u){
		usr = u;
	}
	
	public String toString() {
		return "AlreadyRegisteredUserException : login " + usr.getLogin() + " is already used" ;
	}
	
}
