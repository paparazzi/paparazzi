package pow.webserver;

import nl.justobjects.pushlet.core.Dispatcher;
import nl.justobjects.pushlet.core.Event;
import nl.justobjects.pushlet.core.EventSource;
import java.io.*;
import java.net.*;
import java.util.*;

import javax.crypto.BadPaddingException;
import javax.crypto.IllegalBlockSizeException;
import javax.servlet.ServletContext;

import org.apache.commons.codec.binary.Hex;
import org.jdom.*;
import org.jdom.input.*;
import org.jdom.output.Format;
import org.jdom.output.XMLOutputter;

import pow.AES;
import pow.ivyclient.BusIvy_;

import java.util.HashMap;
import java.util.List;
import java.util.Iterator;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Class which receive a datagramm from ivy bus via UDP
 * transforms it into a Pushlet event and send it to the Pushlet server 
 * in order that it will be sent to web clients
 * @author genin
 */
public class Serveur {
static public class IvyEventSource implements EventSource , Runnable  {
	
	private ServletContext srvCtxt;
	private String default_folder = "blabla";
	HeartBeat heartBeatRun;
	Thread thread = null;
	private boolean active = false;
	private int restarts = 1;
    /* *********************** */
	private static Log myLog;
	private static Conf myConf;
	/* *********************** */
	private DatagramSocket socket;
	private String[] myDataSplited;	// Tableau contenant toutes les donnees envoyees
	/* *********************** */
	private static long time2checkDeadBuses = 30*1000; // period of time where dead bus are removed from bus list 
	/* *********************** */
	/* to retrieve AES cipher for a specific ivy bus*/
	private HashMap<InetAddress,SessionIvy> tableIvySession;
	/* ********* */
	private IvyMsg currentIvyMsg=null;
	/* ********* */
	private LinkedBlockingQueue<IvyMsg> fifoToDatabase;
	private SGBDfeeder t_sgbd_feeder;
	private Thread dbThread;
	/* ********* */
	public IvyEventSource() {
	 System.out.println(" Thread de recuperation des données IVY");
	}
	/* *********************************** */
	/* EventSource specific functions      */
	/* *********************************** */
	/**
	 * Activate the event source.
	 * load configuration, create log and create link to database MySQL
	 * ! setServletContext method should be called before activating 
	 * @see nl.justobjects.pushlet.core.EventSourceManager class
	 */
	synchronized public void activate() {
		System.out.println("activating the event source....");
		// creating log
		myLog = new Log(default_folder);
		// loading configuration
		try {loadingConfiguration();} catch (Exception e) {
			System.out.println("erreur loading configuration");
			e.printStackTrace();}
		// reading configuration
		time2checkDeadBuses = myConf.getTime2checkDeadBuses();
		// get database configuration
		// create object to communicate with database
		
		String dbname = myConf.getDataBaseName();
		String dbuser = myConf.getDBUserName();
		String dbpwd  = myConf.getDBPassword();
		srvCtxt.setAttribute("dbName",dbname);
		srvCtxt.setAttribute("dbUser",dbuser);
		srvCtxt.setAttribute("dbPwd",dbpwd);
		t_sgbd_feeder = new SGBDfeeder( dbname, dbuser, dbpwd,myConf.getDbMode());
		fifoToDatabase = t_sgbd_feeder.getQueueFIFO();
		dbThread = new Thread(t_sgbd_feeder);
		dbThread.start();
		myLog.write("SGBD feeder started. mode = " + myConf.getDbMode().toString());
		// launch heartbeat process
		heartBeatRun = new HeartBeat(srvCtxt,myConf.getSocketTimeout()/3,myConf.portWebToIvy());
		Thread heartThread = new Thread(heartBeatRun);
		heartThread.start();
		myLog.write("heartbeat thread started");
		// Stop a possibly running thread
		stopThread();
		// Start new thread and
		thread = new Thread(this, "DATA FROM IVY " + (restarts++));
		active = true;
		thread.start();
		System.out.println("event source activated !!!");
	}

	/**
	 * Desactivate the event source.
	 */
	synchronized public void passivate() {
		
		active = false;
		stopThread();
		// stop data base feeder
		t_sgbd_feeder.kill();
		System.out.println("database feeder stopped.");
		//
		heartBeatRun.stop_thread();
		System.out.println("heartbeat thread stopped.");
	}

	/**
	 * Desactivate the event source.
	 */
	synchronized public void stop() {
	}
	
	/**
	 * stop a eventually runnig thread
	 */
	private void stopThread() {
		if (thread != null) {
			thread.interrupt();
			thread = null;
		}
	}
	/**
	 * load configuration from a specific configuration file on the server 
	 * in tomcat6/../webapps/ServletPow/conf/pow_conf.xml
	 * and initiates a log file to.
	 */
	private void loadingConfiguration() throws Exception
	{
		myConf = new Conf(default_folder,"pow_conf.xml");
		myLog.write("Loading Configuration !");
		myLog.write("");
		myLog.write("Configuration Details :");
		myLog.write("Port : " + myConf.portIvyToWeb());
		myLog.write("Size of Datagrams : " + myConf.getUdpSize());
		myLog.write("DataBase used : " + myConf.getDataBaseName());
		myLog.write("");		
	}
	
	/**
	 * This procedure convert the byte array into a String and returns an array containing all the
	 * the datas separately.
	 * extract also the information concerning the sender to send them to data base   
	 * @param Data The raw of byte containing udp msg which will be processed
	 * @return An array containing all the datas separately
	 */
	private String[] dataProcessing(byte[] data) //throws Exception
	{
		// This array contain all the datas transmitted
		String[] myDataSplited=null;
		currentIvyMsg=null;
		// Number of datas transmitted
		int nbr_datas;
		// Data separator
		String sep_datas = " ";
		// The datagram is converted into String
		String myData = new String(data);
		// check if format of message is correct ( see java.util.regex.Pattern for syntax)
		Pattern p = Pattern.compile("\\d+ \\d+ (\\d+:){5}\\d+ .*"); //regular expression 
		Matcher m = p.matcher(myData);
		boolean dataok = m.matches();
		if (dataok){
			//
			myDataSplited = myData.split(sep_datas,4); // 4 = number of substring
			int webId = Integer.parseInt(myDataSplited[0]);		
			long numMsg = Long.parseLong(myDataSplited[1]);
			String timeMsg = myDataSplited[2];
			String ivyMsg = myDataSplited[3];
			currentIvyMsg  = new IvyMsg(webId,numMsg,timeMsg,ivyMsg); //create a ivy msg
			myDataSplited = myDataSplited[3].split(sep_datas);
			nbr_datas = myDataSplited.length;
			// The last data is trimed in order to erase all the space
			myDataSplited[nbr_datas - 1] = myDataSplited[nbr_datas - 1].trim();	
		}
		// The array containing all the datas is return
		return myDataSplited;		
	}
	
	/* Runnable specific functions*/
	/**
	 *   main loop :
	 *   	listens to the udp channel
	 *   	deciphers the udp message
	 *   	checks if the sender is a new bus or not
	 *   	send the message to database through a fifo
	 *   	turns the ivy message into a pushlet event
	 *   	send the event to web users
	 *   	checks if there are some buses that doesn't send messages anymore
	 *   	removes drone which belonged to dead buses
	 */
	public void run() {
		boolean decrypt_ok;
		long chrono_start;
		long elapsed_time;
		byte[] complete_data ;
		byte[] decrypted_data = null;
    	int true_length_of_msg ;
    	int i;
    	byte[] data_with_no_padding;
    	boolean firsttime=true;
		System.out.println(" Thread lancé");
		//The server is launched and listen on port number 8535
		try{
		socket = new DatagramSocket(myConf.portIvyToWeb());
		socket.setSoTimeout(myConf.getSocketTimeout());
		myLog.write("Server launched !");
		//The module wait the reception of datagrams
		chrono_start = System.currentTimeMillis();
		while(active)
		{
			// deciphering AES 
			// if I receive a udp packet that means that there was a ivy connection procedure before
			// so I can fetch the table containing the ivy sessions
			if (firsttime){
				tableIvySession =  (HashMap<InetAddress,SessionIvy>) this.srvCtxt.getAttribute("ivySessionTable");
			}
			if (tableIvySession!=null){
				firsttime =false;
			}// end if tableSession!=null			
			byte buffer[] = new byte[myConf.getUdpSize()];
			DatagramPacket dataReceived = new DatagramPacket(buffer,buffer.length);
			try
			{
			socket.receive(dataReceived);
			if(dataReceived != null){
				if (firsttime){ // retrieve the table which store the ivy sessions in the servlet context (shared memory)
					tableIvySession =  (HashMap<InetAddress,SessionIvy>) this.srvCtxt.getAttribute("ivySessionTable");
				}
				if (tableIvySession!=null){
					firsttime =false;
					System.out.print("d");
					//The module checks if there is a new Bus Ivy incoming
					SessionIvy clientIvy = tableIvySession.get(dataReceived.getAddress());
					if (clientIvy!=null){
						// decipher message
						decrypt_ok = false;
						AES cipher = clientIvy.getCipher();
						try{
							complete_data = dataReceived.getData();
							// we get the entire data field but as original message is short ,
							// the udp protocol has inserted a lot of zero byte to pad the data field
							// we have to remove them
			            	true_length_of_msg = dataReceived.getLength();
			            	data_with_no_padding = new byte[true_length_of_msg];
			            	// we retrieve real data
			            	for(i=0;i< true_length_of_msg;i++){
			            		data_with_no_padding[i]=complete_data[i];
			            	}
			            	try{
					            decrypted_data = cipher.decrypt(data_with_no_padding);
					            decrypt_ok = true;
			            	}
			            	catch(BadPaddingException ex){
			            		ex.printStackTrace();
			            		System.out.println("message length = " + true_length_of_msg);
			            		System.out.println("msg no padding := "+(new String(Hex.encodeHex(data_with_no_padding))));
			            	}
			            	catch(IllegalBlockSizeException ex){
			            		ex.printStackTrace();
			            		System.out.println("message length = " + true_length_of_msg);
			            		System.out.println("msg no padding := "+(new String(Hex.encodeHex(data_with_no_padding))));
			            	}
						}
						catch(IOException ex){
							System.out.print("\npbm decrypting msg");
							System.out.print(ex.toString());
						}
						if (decrypt_ok)
						{
							// extract data from message and create ivyMsg
							myDataSplited = dataProcessing(decrypted_data);
							if (myDataSplited!=null){
								// check if a new ivy bus is coming
								BusIvy_ current_bus = clientIvy.getBusIvy();
								if (current_bus==null){
									current_bus  = new BusIvy_();
									current_bus.setAddress(dataReceived.getAddress());
									current_bus.updateTime();
									clientIvy.setBusIvy(current_bus);
									myLog.write("New Ivy Connection detected : " + dataReceived.getAddress());
									// send message de connection 
									currentIvyMsg.setOrder(DbOrder.CONNECT);
									send2DateBase(currentIvyMsg);
								}
								else 
								{
									current_bus.updateTime();							
								}
								// according to type of ivy message a specific event is created and sent to web clients
								if((myDataSplited[0].equals("New_Plane"))
										||(myDataSplited[0].equals("Plane_Die"))
										||(myDataSplited[0].equals("Plane_Resurect"))
										||(myDataSplited[0].equals("DL_SETTING_ACK"))
										||(myDataSplited[0].equals("WAYPOINT_MOVED"))
										||(myDataSplited[0].equals("NAV_STATUS"))
										||(myDataSplited[0].equals("SETTINGS_VALUES")))
								{
									myLog.write("Order Received From : " + dataReceived.getAddress() + " ---> " + new String(decrypted_data));
									sendOrderToServer(myDataSplited);
									if(!(current_bus.isOwnBy(Integer.parseInt(myDataSplited[1]))))
									{
										current_bus.addDrones(Integer.parseInt(myDataSplited[1]));
									}		
									currentIvyMsg.setOrder(DbOrder.ADD_ORDER);
									send2DateBase(currentIvyMsg);
									currentIvyMsg = null;
								}	
								else if(myDataSplited[0].equals("HEARTBEAT")){
									myLog.write("HeartBeat From webID : " +myDataSplited[1]  +" ip="+ dataReceived.getAddress());
								}
								else
								{
									sendDataToServer(myDataSplited);
									if(!(current_bus.isOwnBy(Integer.parseInt(myDataSplited[0]))))
									{
										current_bus.addDrones(Integer.parseInt(myDataSplited[0]));
									}		
									currentIvyMsg.setOrder(DbOrder.ADD_DATA);
									send2DateBase(currentIvyMsg);
									currentIvyMsg = null;
								}																
								System.out.print(".");
							}
							else
							{
								System.out.print("\ndata de crypted but does not fit appropriate format, message skipped.");
							}
						}
						else
						{
							System.out.print("\nerror while decrypting data, message skipped.");
						}
						
				}// fin if ivyclient != null
				else
				{
					System.out.print("\nno ivy client stored for this datagramm from "+dataReceived.getAddress());
				}		
				// check dead buses (period of 30sec )
				elapsed_time =System.currentTimeMillis()- chrono_start;
				if (elapsed_time>time2checkDeadBuses){
					chrono_start = System.currentTimeMillis();
					Iterator<Map.Entry<InetAddress,SessionIvy>> itr = tableIvySession.entrySet().iterator();
					BusIvy_ checked_bus;
					SessionIvy checked_session;
					while(itr.hasNext()){
						checked_session = itr.next().getValue();
						checked_bus  = checked_session.getBusIvy();
						if((checked_bus!=null)&&(!checked_bus.isAlive())){
							myLog.write("Ivy Bus not alive is deconnected : " + checked_bus.getAddress());
							for(Integer deadDrone : checked_bus.getDrones())
							{
								deconnectDrone(deadDrone);
							}
							checked_session.setBusIvy(null);
							// deconnect message sent to database
							currentIvyMsg = new IvyMsg(checked_session.getWebId());
							send2DateBase(currentIvyMsg);
							currentIvyMsg = null;
							// 
							tableIvySession.remove(checked_session.getIvyInetAddress());
						}
					}
				}
			} // fin if tablesession != null
			else
			{
				// on a recu un diagramme mais il n'y a pas de session recorded donc on
				// doit envoyer un message au sender comme quoi il doit se relogger
				System.out.println("\nno session recorder "+dataReceived.getAddress()+ "should relog to server");
			}
			}// fin if datareceived!=null
		    }// fin du try socket received
			catch(SocketTimeoutException ex)
			{		
				if(tableIvySession!=null){
					Iterator<Map.Entry<InetAddress,SessionIvy>> itr = tableIvySession.entrySet().iterator();
					BusIvy_ checked_bus;
					SessionIvy checked_session;
					while(itr.hasNext()){
						checked_session = itr.next().getValue();
						checked_bus  = checked_session.getBusIvy();
						if (checked_bus!=null){
							myLog.write("Socket TimeOut : Ivy bus at " + checked_bus.getAddress()+" deconnected");
							for(Integer deadDrone : checked_bus.getDrones())
							{
								deconnectDrone(deadDrone);
							}
							// deconnect message sent to database
							currentIvyMsg = new IvyMsg(checked_session.getWebId());
							send2DateBase(currentIvyMsg);
							currentIvyMsg = null;
							checked_session.setBusIvy(null);
							tableIvySession.remove(checked_session.getIvyInetAddress());
						}
					}
				}
				myLog.write("No Bus Ivy connected...");
				dataReceived = null;
			}
			catch(IOException ex){
				System.out.println("exception socket receive serveur.java");
				ex.printStackTrace();
			}				
		}// fin while true
		} catch (SocketException soe){
			System.out.print("Socket Exception !");
			soe.printStackTrace();
		}		
		finally{
			if (socket!=null) {
				socket.close();
				System.out.println("socket server closed");
				}
		}
	}
	
	/**
	 * send a iskill event when a drone has disappeared from an ivy bus
	 * @param droneId
	 * @throws Exception
	 */
	private void deconnectDrone(int droneId)
	{
		Event event = Event.createDataEvent("/data/drone/iskill");
		event.setField("aircraftId",droneId);
		event.setField("iskill", 1 + "" ); // 1 ==true
		Dispatcher.getInstance().multicast(event);
		//TODO erasing the associated configuration files ????		
	}
	
	/**
	 * send a IvyMsg to database through fifo buffer
	 */
	private void send2DateBase(IvyMsg m)
	{
		try{
			fifoToDatabase.put(m);
		}
		catch (InterruptedException e){
			System.out.println("message was not sent to database");
		}
	}
	/**
	 * send a datagramm containing all useful information on a drone
	 * @param myDataSplited
	 * @throws Exception
	 */
	private void sendDataToServer(String myDataSplited[])// throws Exception
	{
		int i;
		int nbr_datas = myDataSplited.length;
		String[] myDataName = new String[nbr_datas];
		// id of the drone
		myDataName[0] = "id";
		// name of the drone
		myDataName[1] = "dbName";
		// latitude
		myDataName[2] = "dbLatitude";
		// longitude
		myDataName[3] = "dbLongitude";
		// speed
		myDataName[4] = "dbSpeed";//
		// bearing
		myDataName[5] = "dbCourse";
		// amsl
		myDataName[6] =  "dbAmsl";//
		// vertical speed
		myDataName[7] =  "dbVert_speed";//
		// agl
		myDataName[8] =  "dbAgl";//
		// battery load
		myDataName[9] =  "dbStat_battery";//
		// gps_status
		myDataName[10] =  "dbStat_gps";//
		// Engine status
		myDataName[11] = "dbEngine_status";//
		// active block
		myDataName[12] =  "dbActive_block";//
		// id of the setting
		myDataName[13] = "dbId_Setting";//
		// value of the setting
		myDataName[14] = "dbSetting_Value";//
		// color of the drone , rajout thomas
		myDataName[15] = "drone_color";//
		
		Event event = Event.createDataEvent("/data/drones_maj");
		event.setField("aircraftId",myDataSplited[0]);
			for(i=1;i<nbr_datas;i++)
			{	
				if(i==11)
				{
					float myFloatData = Float.parseFloat(myDataSplited[i]);
					int myData = Math.round(myFloatData);
					event.setField(myDataName[i],myData);
				}
				else
				{	
					event.setField(myDataName[i],myDataSplited[i]);
				}
			}
			Dispatcher.getInstance().multicast(event);
	}
	
	/**
	 * This procedure write the orders given by the bus Ivy in the DataBase.
	 * @param myDataSplited The array containing one order
	 */
	
	private void sendOrderToServer(String myDataSplited[]) //throws Exception
	{
		//try{
			if(myDataSplited[0].equals("WAYPOINT_MOVED"))
			{
				Event event = Event.createDataEvent("/data/order/waypoint_moved");
				event.setField("order",myDataSplited[0]);
				event.setField("aircraftId",myDataSplited[1]);
				event.setField("waypointId", myDataSplited[2] );
				event.setField("latitude", myDataSplited[3] );
				event.setField("longitude", myDataSplited[4] );
				event.setField("altitude", myDataSplited[5] );
				updateXML_FPL_file(myDataSplited[1],myDataSplited[2],myDataSplited[3],myDataSplited[4],myDataSplited[5]);
				Dispatcher.getInstance().multicast(event);
			}
			else if(myDataSplited[0].equals("NAV_STATUS"))
			{
				Event event = Event.createDataEvent("/data/order/block_changed");
				event.setField("order",myDataSplited[0]);//System.out.println("1");
				event.setField("aircraftId",myDataSplited[1]);//System.out.println("2");
				event.setField("currentBlockId", myDataSplited[2] );//System.out.println("3");
				Dispatcher.getInstance().multicast(event);//System.out.println("4");
			}
			else if(myDataSplited[0].equals("DL_SETTING_ACK"))
			{
				Event event = Event.createDataEvent("/data/order/change_setting");
				event.setField("order",myDataSplited[0]);
				event.setField("aircraftId",myDataSplited[1]);
				event.setField("settingId", myDataSplited[2]);
				event.setField("settingValue", myDataSplited[3]);
				Dispatcher.getInstance().multicast(event);
			}
			else if(myDataSplited[0].equals("Plane_Die"))
			{
				Event event = Event.createDataEvent("/data/order/plane_die");
				event.setField("order",myDataSplited[0]);
				event.setField("aircraftId",myDataSplited[1]);
				Dispatcher.getInstance().multicast(event);
			}
			else if(myDataSplited[0].equals("Plane_Resurect"))
			{
				Event event = Event.createDataEvent("/data/order/plane_resurect");
				event.setField("order",myDataSplited[0]);
				event.setField("aircraftId",myDataSplited[1]);
				Dispatcher.getInstance().multicast(event);
			}
			else if(myDataSplited[0].equals("New_Plane"))
			{
				Event event = Event.createDataEvent("/data/order/new_plane");
				event.setField("order",myDataSplited[0]);
				event.setField("aircraftId",myDataSplited[1]);
				event.setField("aircraftName",myDataSplited[2]);
				Dispatcher.getInstance().multicast(event);
			}
			else if(myDataSplited[0].equals("SETTINGS_VALUES"))
			{
				Event event = Event.createDataEvent("/data/order/settings");
				event.setField("order",myDataSplited[0]);
				event.setField("aircraftId",myDataSplited[1]);
				event.setField("csv",myDataSplited[2]);
				Dispatcher.getInstance().multicast(event);
			}	
			else
			{   
				Event event = Event.createDataEvent("/data/order/other");
				event.setField("order",myDataSplited[0]);
				event.setField("aircraftId",myDataSplited[1]);
				Dispatcher.getInstance().multicast(event);
			}
	}
	
	/**
	 * method qui renseigne la source sur la servlet qui l'a appelé
	 * modification de la librairie pushlet
	 * Pushlet.java
	 * EventSource.java
	 * EventSourceManager.java
	 */
	public void setServletContext(ServletContext srvCtxt) {
		this.srvCtxt = srvCtxt;
		default_folder = srvCtxt.getRealPath("");
	}
	/**
	 * modify the file FlightPlan.xml
	 * change the (x,y) coordinates of the waypoints with latitude/longitude coordinates
	 * because in the configuration file, the coordinates (x,y) are relatives to a center, but
	 * in the messages sent by ivy the coordinates are absolute in lat/long  
	 * it avoid to make errors with projections 
	 * @link http://cynober.developpez.com/tutoriel/java/xml/jdom/
	 * @param acId the aircraft webid (to find the file on the server)
	 * @param wpId the id of the waypoint to modify
	 * @param lat the latitude of the waypoint
	 * @param lon the longitude of the waypoint
	 * @param alt the altitude of the waypoint if any
	 */
	private void updateXML_FPL_file(String acId,String wpId,String lat,String lon,String alt){
		// change XML file on server
		// http://cynober.developpez.com/tutoriel/java/xml/jdom/
		String default_folder = this.srvCtxt.getRealPath("");
	     try
	      {
	    	 String filepath = default_folder+"/upload/"+acId+"/flight_plan.xml";
	    	 Document document; 
	 		 SAXBuilder sxb = new SAXBuilder(); 
	 		 Element racine;
	    	 document = sxb.build(new File(filepath));// charge le fichier
	    	 //get the root node of the xml tree
		     racine = document.getRootElement();
		     List list_wpt = racine.getChild("flight_plan").getChild("waypoints").getChildren("waypoint");
		     Iterator i = list_wpt.iterator();
		     boolean doIt = true;
		     int cpt_wpt=0;
		     while(i.hasNext()&&doIt) // searching for the waypoint (should be in the correct order)
		     {	       
		        Element courant = (Element)i.next();
		        if (cpt_wpt==(Integer.parseInt(wpId)-1)){ 
		        	String name = courant.getAttributeValue("name");   
		        	System.out.println("wpt changed : "+ name + " for drone "+ acId +" id="+cpt_wpt+" with\tlat="+lat + "\tlon="+lon);
		        	doIt= false;
		        	courant.removeAttribute("x"); // remove the x, y coordinates if they are still here
		        	courant.removeAttribute("y"); 
		        	courant.setAttribute("lat", lat); // inserting the new lat/lon coordinates
		        	courant.setAttribute("long", lon);
		        	courant.setAttribute("alt", alt);
		        }
		        cpt_wpt++;
		     }
		     // save the modified file
		     XMLOutputter sortie = new XMLOutputter(Format.getPrettyFormat());
	         sortie.output(document, new FileOutputStream(filepath));
	      }
	      catch(Exception e){System.out.println("error : unable to load xml file");}
	      
	}
}
}
