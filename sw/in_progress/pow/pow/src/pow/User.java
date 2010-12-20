package pow;
import java.util.HashSet;
import java.util.Iterator;


public class User implements java.io.Serializable {
  private String login = "";
  private String password = "";
  private HashSet<String> dronesNamePermitted;
  private Rights right;
  
  public User(String log, String pwd,Rights rght) {
    this.login = log;
    this.password = Md5.encode(pwd);
    this.right = rght;
    this.dronesNamePermitted = new HashSet<String>();
  }
  
  public Iterator<String> getSetItr(){
	  return dronesNamePermitted.iterator();
  }
  
  public void clearListDrone(){
	  dronesNamePermitted.clear();
  }
  
  public String getListDrone(){
	  Iterator<String> itr = dronesNamePermitted.iterator();
	  String res = "";
	  while(itr.hasNext()){
		  res = res + itr.next() + ";";
	  }
	  return res;
  }
  
  public void addDrone(String d){
	  dronesNamePermitted.add(d);
  }
  
  public boolean canControl(String droneName)
  {
	  if ((right == Rights.ADMIN)|| ((right==Rights.USER)&&dronesNamePermitted.contains(droneName)))
	  { return true; }
	  else 
	  { return false; }
  }
  
  public Rights getRights(){
	  return this.right;
  }
  
  public void setRights(Rights r){
	  this.right=r;
  }
   
  public String getLogin() {
    return login;
  }
   
  public void setNom(String log) {
    this.login = log;
  }
   
  public String getPassword() {
    return password;
  }
   
  public void setPwd(String pwd) {
	  this.password = Md5.encode(pwd);
  }
  
  public boolean IsPwdTrue(String pwd) {
	  return   this.password.equals(Md5.encode(pwd));
  }
}

