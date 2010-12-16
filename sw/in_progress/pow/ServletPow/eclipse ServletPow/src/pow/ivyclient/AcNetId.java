package pow.ivyclient;
/**
 * this object represents the data concerning a drone extracted from Ivy
 * A drone has two identities. One on its ivy drone which can be the same as 
 * another drone on a different ivy bus. The second identifier is the I.D. of the drone 
 * on the web. This I.D. is unique. So a link has to be done between these two kind of id.
 * @author genin
 */
public class AcNetId {
	private int idOfBusIvy;
	private String idOnIvy;
	private String nameOnIvy;
	private String idOnWeb;
	private String flightplan_path;
	private String setting_path;
	private String color;
	private int maxAircrafts; ///
	 /**
	  * 
	  * @param idBusIvy the id of the ivy bus 
	  * @param idIvy the identifier of the drone on ivy
	  * @param n the name of the ivy drone (MJ5,TJ1...)
	  * @param pln the full pathname of the configuration file FlightPlan.xml
	  * @param s  the full pathname of the configuration file Settings.xml
	  * @param c the color of the drone on ivy (in hexa '#FF00FF' or with its common name 'red')
	  * @param maxACOnIvy the maximal number of aircrafts on ivy bus (is no important anymore)
	  */
	public AcNetId(int idBusIvy,String idIvy,String n,String pln,String s,String c,int maxACOnIvy){
		idOfBusIvy= idBusIvy;
		idOnIvy=idIvy;
		nameOnIvy=n;	
		flightplan_path = pln;
		setting_path= s;
		color = c;
		maxAircrafts = maxACOnIvy;
		idOnWeb = generateIdOnWeb();
		
	}
	/**
	 * 
	 * @param idBusIvy the id of the ivy bus 
	 * @param droneWebId the identifier of the drone on the web
	 * @param idIvy the identifier of the drone on ivy
	 * @param n the name of the ivy drone (MJ5,TJ1...)
	 * @param pln the full pathname of the configuration file FlightPlan.xml
	 * @param s  the full pathname of the configuration file Settings.xml
	 * @param c the color of the drone on ivy (in hexa '#FF00FF' or with its common name 'red')
	 * @param maxACOnIvy the maximal number of aircrafts on ivy bus (is no important anymore)
	 */
	public AcNetId(int idBusIvy,String droneWebId,String idIvy,String n,String pln,String s,String c,int maxACOnIvy){
		idOfBusIvy= idBusIvy;
		idOnIvy=idIvy;
		nameOnIvy=n;	
		flightplan_path = pln;
		setting_path= s;
		color = c;
		maxAircrafts = maxACOnIvy;
		idOnWeb = droneWebId;
		
	}
	/**
	 * provides the id of the drone on the ivy bus
	 * @return the id number of the drone on its ivy bus
	 */
	public String getIdOnIvy() {return idOnIvy;}
	/**
	 *  provides the name of the drone on the ivy bus
	 * @return the name of the drone on its ivy bus
	 */
	public String getName() {return nameOnIvy;}
	/**
	 *  provides the id of the drone on the web
	 * @return the single id number of the drone on the web server
	 */
	public String getIdOnWeb() {return idOnWeb;}
	/**
	 *  provides the path of the xml file (on the server) containing 
	 *  the information of the flightplan of the drone
	 *  (waypoints and blocks of instructions)
	 * @return the full path of the flight plan config file on the GCS station
	 */
	public String getPlnPath() {return flightplan_path;}
	/**
	 *  provides the path of the xml file (on the server) containing the settings
	 *  of the drone
	 * @return the full path of the settings file on the GCS station
	 */
	public String getSettingPath() {return setting_path;}
	/**
	 * provides the color of the drone on the GCS view
	 * @return the color of the drone on the GCS station (either in string or in hexadecimal)
	 */
	public String getColor() {return color;}
	/**
	 * generate a unique id for the drone on the web
	 * @deprecated the id is now sent by the server
	 * @return the id of the drone on the server
	 */
	private String generateIdOnWeb() {
		//String newId = "" + (Integer.parseInt(idOnIvy) * ((int)Math.pow(maxAircrafts,idOfBusIvy)));
		String newId = "" + (Integer.parseInt(idOnIvy) +maxAircrafts*idOfBusIvy*10);
		System.out.println("### new Id generated : " + newId + " ###");
		return newId;
	}
	/**
	 * return information about the drone in a string format
	 * @return a string containing information about the drone
	 */
	public String toString(){
		return "id on ivy :\t" + idOnIvy +
				"\nname :\t" + nameOnIvy +
				"\nid on web :\t" + idOnWeb +
				"\nfpl file :\t" + flightplan_path +
				"\nsetting file :\t" + setting_path +
				"\ncolor :\t" + color ;
	}
	
}
