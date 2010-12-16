package pow.webserver;
import java.util.HashSet;
import java.util.Iterator;

/**
 * represents a ivy or a web user account
 * for the web user the account nest also the list of drones that the user can control
 * the list contains the ivy name of the drone (MJ5, TJ1...). 
 * In Paparazzi a drone is not identified in a unique identifier, that means that if a user
 * can pilot a drone MJ5, he can pilot all 'MJ5' drone 
 * @author genin
 */
public class User implements java.io.Serializable {
  
  private static final long serialVersionUID = 1L;
  private String login = "";
  private String password = "";
  private HashSet<String> dronesNamePermitted;
  private Rights right;
  /**
   * create a user account 
   * @param log the login of the user
   * @param pwd the password of the user
   * @param rght the type of the user
   */
  public User(String log, String pwd,Rights rght) {
    this.login = log;
    this.password = Md5.encode(pwd);
    this.right = rght;
    this.dronesNamePermitted = new HashSet<String>();
  }
  /**
   * @return an iterator containing the list of all drone that the user can control
   */
  public Iterator<String> getSetItr(){
	  return dronesNamePermitted.iterator();
  }
  /**
   * reset the list of drone that the web user can control 
   */
  public void clearListDrone(){
	  dronesNamePermitted.clear();
  }
  /**
   * the list of drone that the web user can control
   * @return a formatted string containing all the drone that the user can control separated by commas
   */
  public String getListDrone(){
	  Iterator<String> itr = dronesNamePermitted.iterator();
	  String res = "";
	  while(itr.hasNext()){
		  res = res + itr.next() + ";";
	  }
	  return res;
  }
  /**
   * add a drone name to the list of drones that the user can control
   * @param d
   */
  public void addDrone(String d){
	  dronesNamePermitted.add(d);
  }
  /**
   * inform whether the user can pilot a drone or not
   * @param droneName the name of the drone
   * @return true if the user can pilot the drones with this name
   */
  public boolean canControl(String droneName)
  {
	  if ((right == Rights.ADMIN)|| ((right==Rights.USER)&&dronesNamePermitted.contains(droneName)))
	  { return true; }
	  else 
	  { return false; }
  }
  /** @return the type of right granted to the user*/
  public Rights getRights(){
	  return this.right;
  }
  /**
   * specify the type of right granted to the user
   * @param r the right granted to the user
   */
  public void setRights(Rights r){
	  this.right=r;
  }
  /**
   * @return the login of the user
   */
  public String getLogin() {
    return login;
  }
  /**
   * specify the login of the user
   * @param log the login of the user
   */
  public void setNom(String log) {
    this.login = log;
  }
   /**
    * @return the password of the user encoded with md5 algorithm
    */
  public String getPassword() {
    return password;
  }
  /**
   * specify the user password, it's not stored in clear (md5 hash)
   * @param pwd the user password in clear
   */
  public void setPwd(String pwd) {
	  this.password = Md5.encode(pwd);
  }
  /**
   * check if the password given in argument corresponds to the password user 
   * @param pwd the password to check
   * @return true if the password is correct
   */
  public boolean IsPwdTrue(String pwd) {
	  return   this.password.equals(Md5.encode(pwd));
  }
}

