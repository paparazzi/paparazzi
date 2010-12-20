package pow.ivyclient;
/**
 * object which store all information about the url to log on the server
 * @author genin
 *
 */
public class PowUrl {
	private String protocole="http";
	private int port =-1;
	private String serverName;
	private String webAppName;
	private String servletLoginName;
	/**
	 * create all the information requested to connect the ivy module to the server
	 * when all parameters are correct can give an url like
	 * https://paparazzi.fr/ServletPow/Login.srv
	 * @param protocol the protocol used to log to the server (HTTP by default)
	 * @param server the server name (paparazzi.fr) for example
	 * @param port a specific protocol port ( -1 means that this parameter is not used )
	 * @param webapp the name of the application on the server (ServletPow for example)
	 * @param servletName the name of the login servlet (Login.srv)
	 */
	public PowUrl(String protocol,String server,int port,String webapp,String servletName){
		if (protocol.equals("https")) this.protocole="https";
		this.port = port;
		this.serverName =server;
		this.webAppName = webapp;
		this.servletLoginName = servletName;
	}
	/** provides the server name to which the module is connected
	 *  @return server hostname*/
	public String getServerName() {return serverName;}
	/** provides the complete URL to log on the POW server 
	 * @return the complete url to log in as a string */
	public String getWebUrl() {
		if (port==-1) return ""+protocole+"://"+serverName+"/"+webAppName+"/"+servletLoginName;
		else
			return ""+protocole+"://"+serverName+":"+port+"/"+webAppName+"/"+servletLoginName;
	}
}
