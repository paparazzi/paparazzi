package pow;

public class PowUrl {
	private String protocole="http";
	private int port =-1;
	private String serverName;
	private String webAppName;
	private String servletLoginName;
	
	public PowUrl(String protocol,String server,int port,String webapp,String servletName){
		if (protocol.equals("https")) this.protocole="https";
		this.port = port;
		this.serverName =server;
		this.webAppName = webapp;
		this.servletLoginName = servletName;
	}
	
	public String getServerName() {return serverName;}
	public String getWebUrl() {
		if (port==-1) return ""+protocole+"://"+serverName+"/"+webAppName+"/"+servletLoginName;
		else
			return ""+protocole+"://"+serverName+":"+port+"/"+webAppName+"/"+servletLoginName;
	}
}
