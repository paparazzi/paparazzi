package pow.ivyclient;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.crypto.BadPaddingException;
import javax.crypto.IllegalBlockSizeException;

import pow.AES;

/**
 * thread which send a periodic message to a client or server to say that the connection 
 * is still alive (period has to be less than the socket timeout of the the server)
 * @author genin
 */
public class HeartBeat implements Runnable {

	private InetAddress serveur;
	private int portUdpSend;
	private long period2sendheartbeat;
	private boolean doIt; // to stop thread
	private int webId;
	private SimpleDateFormat dateformat;
	private int num_msg;
	private AES aes;
	
	/**
	 * define the parameter  of the heartbeat message
	 * @param cipher the way to encrypt message to be send on udp channel
	 * @param webId the id of the bus 
	 * @param srv the ip address of the host hosting the ivy bus
	 * @param port2send the udp port on which the heartbeat is sent
	 * @param heartbeatPeriod the period of time to send heartbeat message
	 */
	public HeartBeat(AES cipher,int webId, InetAddress srv,int port2send,long heartbeatPeriod) {
		aes = cipher;
		this.webId = webId;
		serveur = srv;
		
		portUdpSend = port2send;
		period2sendheartbeat = heartbeatPeriod;
		dateformat = new SimpleDateFormat("d:M:y:HH:mm:ss");
		num_msg = 0;
		doIt =true;
	}
	

	/**
	 * send a periodic heartbeat message
	 */
	public void run(){
		
		String msg = "HEARTBEAT "+ webId;
		String full_msg;
		byte[] buffer;
		DatagramPacket dataUdp;
		DatagramSocket socketUdp=null;
		while(doIt){
			try{
				Thread.sleep(period2sendheartbeat);
				full_msg = add_info_msg(msg);
				buffer = aes.encrypt(full_msg.getBytes()); // method should be synchronized ?
				try {
					socketUdp = new DatagramSocket();
					dataUdp = new DatagramPacket(buffer, buffer.length, serveur, portUdpSend);
					socketUdp.send(dataUdp);
					System.out.println("H");
				} catch (IOException e) {
					System.out.println("socket ioexception : can't send heart beat");
				}
				finally{
					if (socketUdp!=null) {socketUdp.close();socketUdp=null;}
				}
			}
			catch(InterruptedException iex){
				System.out.println("heartbeat sleep failed");
			}
			catch(BadPaddingException bpex){System.out.println("error padding heartbeat");}
			catch(IllegalBlockSizeException ex){System.out.println("error illegal block heartbeat");}
			catch(IOException ioex){System.out.println("error io heartbeat");}
		}
		System.out.println("Thread HeartBeat stopped !");
	}
	
	/**
	  * add usefull info to msg for storage in database
	  * its requested because of the format of messages that server is waiting
	  */
	 private String add_info_msg(String msg){
		 String monHeure = this.dateformat.format(new Date());
		 String res = "" + this.webId+" " + this.num_msg + " " + monHeure +" " + msg;
		 this.num_msg++;
		 return res;
	 }
	 /**
	  * stop the heartbeat thread by ending the infinite loop
	  */
	 public void stop_thread(){doIt=false;}
}
