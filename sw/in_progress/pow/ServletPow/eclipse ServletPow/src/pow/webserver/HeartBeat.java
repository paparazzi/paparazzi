package pow.webserver;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import javax.crypto.BadPaddingException;
import javax.crypto.IllegalBlockSizeException;
import javax.servlet.ServletContext;

import pow.AES;

public class HeartBeat implements Runnable{
	/** to retrieve AES cipher and InetAddress for a specific ivy bus*/
	
	private ServletContext srvCtxt;
	private int heartPeriod;
	private boolean doIt;
	private int port;
	/**
	 * 
	 * @param s the object which stores ivysessions
	 * @param h the period of the heartbeats
	 * @param p the udp port to use to send heartbeat
	 */
	public HeartBeat( ServletContext  s,int h,int p){
		srvCtxt = s;
		heartPeriod = h ;
		port  = p;
		doIt= true;
	}
	/**
	 * send a periodic heartbeat message to every connected ivy bus
	 */
	public void run(){
		
		String msg = "SERVER_HEARTBEAT";
		byte[] buffer;
		DatagramSocket socket=null;
		AES cipher ;
		InetAddress ip_addr;
		while(doIt)
		{
			try {
				Thread.sleep(heartPeriod);
				socket = new DatagramSocket();
				HashMap<InetAddress,SessionIvy> tableIvySession =  (HashMap<InetAddress,SessionIvy>) srvCtxt.getAttribute("ivySessionTable");
				if (tableIvySession!=null){
					Iterator<Map.Entry<InetAddress,SessionIvy>> itr = tableIvySession.entrySet().iterator();
					while(itr.hasNext()){
						Map.Entry<InetAddress,SessionIvy> n= itr.next();
						cipher  = n.getValue().getCipher();
						ip_addr = n.getKey();System.out.println("send heart to "+ ip_addr);
						buffer = cipher.encrypt(msg.getBytes());
						DatagramPacket DataEmitted = new DatagramPacket(buffer, buffer.length, ip_addr, port);
						try {
							socket.send(DataEmitted);
							//System.out.println("\nHeartbeat sended to "+ip_addr);
						} catch (IOException e) {
							System.out.println("\tHeartbeat pbm");e.printStackTrace();
						} 
					 }
				}
				
			} catch (InterruptedException e) {System.out.println("\tHeartbeat pbm");e.printStackTrace();	}
			catch (SocketException ex) { System.out.println("\tHeartbeat pbm");ex.printStackTrace();}
			catch(BadPaddingException ex){System.out.println("\tHeartbeat pbm");ex.printStackTrace();}
			catch(IllegalBlockSizeException ex){System.out.println("\tHeartbeat pbm");ex.printStackTrace();}  
			catch(IOException ex){System.out.println("\tHeartbeat pbm");ex.printStackTrace();}
			finally{if(socket!=null){socket.close();socket = null ;} }
		}
	}
	/**
	 * stop the heartbeat process of the server
	 */
	public void stop_thread(){
		doIt=false;
	}
}
