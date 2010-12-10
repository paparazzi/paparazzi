package pow.webserver;

import java.util.HashMap;
import java.util.Iterator;
import java.io.*;
/**
 * User objects are stored in a UserTab object which can be serialized in a file on the server disk
 * @see User
 * @author genin
 */
public class UserTab  implements java.io.Serializable {

	
	private static final long serialVersionUID = 1L;
	private HashMap<String,User> tabUser;
	/**
	 * create a empty list of users
	 */
	public UserTab(){
		tabUser = new HashMap<String,User>();
	}
	/**
	 * insert a user in the list
	 * @param u a user object to insert
	 * @throws AlreadyRegisteredUserException
	 */
	public void insert(User u) throws AlreadyRegisteredUserException {
		String log = u.getLogin();
		if (!tabUser.containsKey(log)) tabUser.put(u.getLogin(),u);
		else { throw new AlreadyRegisteredUserException(u) ;}
	}
	/**
	 * remove the user with the spefied login
	 * @param log the login of the user to remove
	 */
	public void remove(String log){
		tabUser.remove(log) ;
	}
	/**
	 * return the user corresponding to the given login
	 * @param log the login of the user to seek 
	 * @return the user object corresponding to the login or null if it is not present
	 */
	public User seek(String log){
		return tabUser.get(log);
	}
	/**
	 * check if an user with this logging exists yet
	 * @param log the login of the user to check
	 * @return true if the user having the login exists
	 */
	public boolean isInside(String log){
		return tabUser.containsKey(log); 
	}
	
	/**
	 * Check if the password of the user having the specified login is correct
	 * @param log the login of the user
	 * @param pwd the password 
	 * @return true if there is a user with this login and this password
	 */
	public boolean checkUser(String log,String pwd){
		boolean res = false;
		User usr = tabUser.get(log);
		if (usr!=null){
			res=usr.IsPwdTrue(pwd);
		}
		return res;
	}
	/**
	 * return the list of all drone which may be controlled by the user
	 * @param log the login of the user
	 * @return an iterator containing the list of drone or null is the user does not exist
	 */
	public Iterator<String> getItrUsr(String log){
		Iterator<String> res = null;
		User usr = tabUser.get(log);
		if (usr!=null){
			res=usr.getSetItr();
		}
		return res;
	}
	/**
	 * @return an iterator on all login in the list
	 */
	public Iterator<String> getLoginIterator(){
		return tabUser.keySet().iterator();
	}
	/**
	 * Save the object in a file
	 * @param nomfichier the file where the user's accounts will be stored
	 */
	public void serialize(String nomfichier){
		try {
			      FileOutputStream fichier = new FileOutputStream(nomfichier);
			      ObjectOutputStream oos   = new ObjectOutputStream(fichier);
			      oos.writeObject(this);
			      oos.flush();
			      oos.close();
			    }
			    catch (java.io.IOException e) {
			      e.printStackTrace();
			    }
	}
	/**
	 * load the object from a file
	 * @param nomfichier  the file where the user's accounts are stored
	 * @return an object containing the users list or null if the pathname is not correct 
	 */
	public static UserTab unserialize(String nomfichier) {
		    try {
		      FileInputStream fichier = new FileInputStream(nomfichier);
		      ObjectInputStream ois = new ObjectInputStream(fichier);
		      UserTab usrtab = (UserTab) ois.readObject();
		      return usrtab;
		    }
		    catch (java.io.IOException e) {
		      e.printStackTrace();
		    }
		    catch (ClassNotFoundException e) {
		      e.printStackTrace();
		    }
		    return null;
		   }
}
