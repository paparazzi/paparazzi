package pow.webserver;

import java.net.InetAddress;
import java.net.UnknownHostException;

import pow.AES;
import pow.ivyclient.BusIvy_;
/**
 * store information about a specific ivy bus which is connected 
 * to the server
 * @author genin
 *
 */
public class SessionIvy {
	
	private String login;
	private int webId;
	private AES cipherAES;
	private InetAddress IvyHostName;
	private BusIvy_ busIvy;
	/**
	 * create a session which store information about an ivy bus client connected to the server
	 * main information are its net address to send him orders from web users
	 * and drones which are connected to this ivy bus
	 * @param l the login of the ivy user 
	 * @param w the webid of the ivy bus
	 * @param c the cipher object
	 * @param IvyAddr the IP address of the ivy host machine
	 * @throws UnknownHostException
	 */
	public SessionIvy(String l,int w, AES c,InetAddress IvyAddr) throws UnknownHostException{
		this.login = l;
		this.webId = w;
		this.cipherAES = c;
		this.IvyHostName = IvyAddr;
		busIvy = null;
	}
	/** @return the ip adress of the ivy bus */
	public InetAddress getIvyInetAddress(){
		return this.IvyHostName;
	}
	/** set the ip adress of the ivy bus host*/
	public void setIvyInetAddress(InetAddress addr){
		 this.IvyHostName=addr;
	}
	/** @return the AES object which allows to encrypt and decrypt a message with*/
	public AES getCipher() {return cipherAES;}
	/** @return the object storing data about drones on the bus*/
	public BusIvy_ getBusIvy(){
		return this.busIvy;
	}
	/**
	 * store the information of the ivy bus into the ivy session
	 * @param b the ivy bus object
	 */
	public void setBusIvy( BusIvy_ b){
		this.busIvy = b;
	}
	/** @return the ivy bus identifier */
	public int getWebId() {return webId;}
	/** @return the login of the user who is connected via this ivy bus*/
	public String getLogin(){return login;}
}
