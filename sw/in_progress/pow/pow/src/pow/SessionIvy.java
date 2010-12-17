package pow;

import java.net.InetAddress;
import java.net.UnknownHostException;

public class SessionIvy {
	
	private String login;
	private int webId;
	private AES cipherAES;
	private InetAddress IvyHostName;
	private BusIvy_ busIvy;
	
	public SessionIvy(String l,int w, AES c,InetAddress IvyAddr) throws UnknownHostException{
		this.login = l;
		this.webId = w;
		this.cipherAES = c;
		this.IvyHostName = IvyAddr;
		busIvy = null;
	}
	
	public InetAddress getIvyInetAddress(){
		return this.IvyHostName;
	}
	
	public void setIvyInetAddress(InetAddress addr){
		 this.IvyHostName=addr;
	}
	
	public AES getCipher() {return cipherAES;}
	
	public BusIvy_ getBusIvy(){
		return this.busIvy;
	}
	
	public void setBusIvy( BusIvy_ b){
		this.busIvy = b;
	}
}
