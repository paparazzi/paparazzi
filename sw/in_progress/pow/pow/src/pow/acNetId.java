package pow;
/**
 * this object represents the data concerning a drone extracted from Ivy
 * @author genin
 *
 */
public class acNetId {
	private int idOfBusIvy;
	private String idOnIvy;
	private String nameOnIvy;
	private String idOnWeb;
	private String flightplan_path;
	private String setting_path;
	private String color;
	private int maxAircrafts; ///
	 
	public acNetId(int idBusIvy,String idIvy,String n,String pln,String s,String c,int maxACOnIvy){
		idOfBusIvy= idBusIvy;
		idOnIvy=idIvy;
		nameOnIvy=n;	
		flightplan_path = pln;
		setting_path= s;
		color = c;
		maxAircrafts = maxACOnIvy;
		idOnWeb = generateIdOnWeb();
		
	}
	
	public acNetId(int idBusIvy,String droneWebId,String idIvy,String n,String pln,String s,String c,int maxACOnIvy){
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
	 * 
	 * @return the id number of the drone on its ivy bus
	 */
	public String getIdOnIvy() {return idOnIvy;}
	/**
	 * 
	 * @return the name of the drone on its ivy bus
	 */
	public String getName() {return nameOnIvy;}
	/**
	 * 
	 * @return the single id number of the drone on the web server
	 */
	public String getIdOnWeb() {return idOnWeb;}
	/**
	 * 
	 * @return the full path of the flight plan config file on the GCS station
	 */
	public String getPlnPath() {return flightplan_path;}
	/**
	 * 
	 * @return the full path of the settings file on the GCS station
	 */
	public String getSettingPath() {return setting_path;}
	/**
	 * 
	 * @return the color of the drone on the GCS station (either in string or in hexadecimal)
	 */
	public String getColor() {return color;}
	
	private String generateIdOnWeb() {
		//String newId = "" + (Integer.parseInt(idOnIvy) * ((int)Math.pow(maxAircrafts,idOfBusIvy)));
		String newId = "" + (Integer.parseInt(idOnIvy) +maxAircrafts*idOfBusIvy*10);
		System.out.println("### new Id generated : " + newId + " ###");
		return newId;
	}
	/**
	 * @return information about the drone in a string format
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
