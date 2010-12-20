package pow;
/**
 * exception to inform that a user cannot be created in the users' file 
 * because it exists yet
 * @author genin
 *
 */
public class AlreadyRegisteredUserException extends Exception {

	private User usr;
	
	public AlreadyRegisteredUserException (User u){
		usr = u;
	}
	
	public String toString() {
		return "AlreadyRegisteredUserException : login " + usr.getLogin() + " is already used" ;
	}
	
}
