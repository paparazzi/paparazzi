package pow;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.io.*;

public class UserTab  implements java.io.Serializable {

	private HashMap<String,User> tabUser;
	/**
	 * create a empty list of users
	 */
	public UserTab(){
		tabUser = new HashMap<String,User>();
	}
	/**
	 * insert a user in the list
	 * @param u
	 * @throws AlreadyRegisteredUserException
	 */
	public void insert(User u) throws AlreadyRegisteredUserException {
		String log = u.getLogin();
		if (!tabUser.containsKey(log)) tabUser.put(u.getLogin(),u);
		else { throw new AlreadyRegisteredUserException(u) ;}
	}
	/**
	 * remove the user with the spefied login
	 * @param log
	 */
	public void remove(String log){
		tabUser.remove(log) ;
	}
	/**
	 * return the user corresponding to the given login
	 * @param log
	 * @return
	 */
	public User seek(String log){
		return tabUser.get(log);
	}
	/**
	 * check if an user with this logging exists yet
	 * @param log
	 * @return
	 */
	public boolean isInside(String log){
		return tabUser.containsKey(log); 
	}
	
	/**
	 * Check if the login is correct
	 * @param log
	 * @param pwd
	 * @return
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
	 * return the list of all drone which may be controled by the user
	 * @param log
	 * @return
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
	 * return the iterator on all login in the list
	 * @param log
	 * @return
	 */
	public Iterator<String> getLoginIterator(){
		return tabUser.keySet().iterator();
	}
	/**
	 * Save the object in a file
	 * @param nomfichier
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
	 * @param nomfichier
	 * @return
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
