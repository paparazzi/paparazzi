package pow;

import nl.justobjects.pushlet.core.Dispatcher;
import nl.justobjects.pushlet.core.Event;
import nl.justobjects.pushlet.core.EventSource;
import java.io.*;
import java.net.*;
/*
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
*/
import java.util.*;

import javax.crypto.BadPaddingException;
import javax.crypto.IllegalBlockSizeException;
import javax.servlet.ServletContext;
//import javax.servlet.*;
//import javax.servlet.http.*;
import pow.Conf;

import org.apache.commons.codec.binary.Hex;
import org.jdom.*;
import org.jdom.input.*;
import org.jdom.output.Format;
import org.jdom.output.XMLOutputter;
import org.jdom.filter.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Iterator;

import java.util.Date;

/**
 * Class which receive a datagramm from ivy bus via udp
 * transforms it into a pushlet event and send it to the pushlet server 
 * in order that it will be sended to web clients
 * @author genin
 *
 */
public class Serveur {
static public class IvyEventSource implements EventSource , Runnable  {
	
	private ServletContext srvCtxt;
	private String default_folder = "blabla";
	
	Thread thread = null;
	volatile boolean active = false;
	private int restarts = 1;
    /* *********************** */
	private static log myLog;
	private static Conf myConf;
	/* *********************** */
	private DatagramSocket socket;
	private int newBusTest;
	private ArrayList <BusIvy_> busesIvy;	// Tableau identifiant les busIvy et les drones
	private ArrayList <BusIvy_> deadBusesIvy;	// Tableau identifiant les busIvy deconnectes
	private String[] myDataSplited;	// Tableau contenant toutes les donnees envoyees
	/* *********************** */
	private static long time2resetFilters = 100000; // on nettoie les filtres ms !!!!!
	/* *********************** */
	private MsgFilter waypointMsgFilter;
	/* to retreive aes cipher for a specific ivy bus*/
	private HashMap<InetAddress,SessionIvy> tableIvySession;
	/* ********* */
	private IvyMsg currentIvyMsg=null;
	/* ********* */
	
	public IvyEventSource() {
	 System.out.println(" Thread de recuperation des données IVY");
	 waypointMsgFilter = new MsgFilter();
	}
	/* *********************************** */
	/* EventSource specific functions      */
	/* *********************************** */
	/**
	 * Activate the event source.
	 */
	synchronized public void activate() {
		System.out.println("activating the event source....");
		// Stop a possibly running thread
		stopThread();
		// Start new thread and
		thread = new Thread(this, "DATA FROM IVY " + (restarts++));
		active = true;
		thread.start();
		System.out.println("event source activated !!!");
	}

	/**
	 * Deactivate the event source.
	 */
	synchronized public void passivate() {
		// inform the other servlet
		this.srvCtxt.removeAttribute("list_bus_ivy");
		//
		active = false;
		stopThread();
	}

	/**
	 * Deactivate the event source.
	 */
	synchronized public void stop() {
	}
	
	private void stopThread() {
		if (thread != null) {
			thread.interrupt();
			thread = null;
		}
	}
	/*
	 * load configuration from a specific configuration file on the server 
	 * and initiates a log file to.
	 */
	private void loadingConfiguration() throws Exception
	{
		myConf = new Conf(default_folder,"pow.conf");
		myLog.write("Loading Configuration !");
		myLog.write("");
		myLog.write("Configuration Details :");
		myLog.write("Port : " + myConf.port());
		myLog.write("Size of Datagrams : " + myConf.taille());
		myLog.write("DataBase used : " + myConf.dataBaseName());
		myLog.write("");		
	}
	
	/**
	 * This procedure convert the byte array into a String and returns an array containing all the
	 * the datas separately.
	 * extract also the information concerning the sender to send them to data base   
	 * @param Data The raw of byte containing udp msg which will be processed
	 * @return An array containing all the datas separately
	 */
	public static String[] dataProcessing(byte[] data) //throws Exception
	{
		// This array contain all the datas transmitted
		String[] myDataSplited;
		// Number of datas transmitted
		int nbr_datas;
		// Data separator
		String sep_datas = " ";
		// The datagram is converted into String
		String myData = new String(data);
		
		myDataSplited = myData.split(sep_datas,4); // 4 = number of substring
		int webid = Integer.parseInt(myDataSplited[0]);
		long num_msg = Long.parseLong(myDataSplited[1]);
		String timeMsg = myDataSplited[2];
		String ivymsg = myDataSplited[3];
		myDataSplited = myDataSplited[3].split(sep_datas);
		//System.out.println("##############################");
		//for(int i =  0; i<myDataSplited.length;i++) System.out.println("***\t"+myDataSplited[i]);
		//System.out.println("???????????????????????????????");
		nbr_datas = myDataSplited.length;
		// The last data is trimed in order to erase all the space
		myDataSplited[nbr_datas - 1] = myDataSplited[nbr_datas - 1].trim();				
		// The array containing all the datas is return
		return myDataSplited;		
	}
	
	/* Runnable specific functions*/
	/**
	 *   main loop 
	 */
	public void run() {
		long chrono_start;
		long elapsed_time;
		byte[] complete_data ;
    	int true_length_of_msg ;
    	int i;
    	byte[] data_with_no_padding;
    	boolean firsttime=true;
		//boolean clearFilter = false ;
		 System.out.println(" Thread lancé");
		//try{
		myLog = new log(default_folder);
		// Tableau identifiant les busIvy et les drones
		busesIvy = new ArrayList <BusIvy_>();
		// Tableau identifiant les busIvy deconnectes
		deadBusesIvy = new ArrayList <BusIvy_>();
		
		try {loadingConfiguration();} catch (Exception e) {
			System.out.println("erreur loading configuration");
			e.printStackTrace();}
		
		// inform the other servlet
		this.srvCtxt.setAttribute("list_bus_ivy", busesIvy);
		//
		
		//The server is launched and listen on port number 8535
		try{
		socket = new DatagramSocket(myConf.port());
		socket.setSoTimeout(myConf.timeout());
		// System.out.println(" ecriture log");
		myLog.write("Server launched !");
		// System.out.println("Server launched !");
		
		//The module wait the reception of datagrams
		chrono_start = System.currentTimeMillis();
		while(true)
		{
			// 
			// decryptage AES 
			// si je recois un paquet via udp c'est qu'il y a eu login d'un client ivy
			// et donc je peux recupere la table des sessions
			if (firsttime){
				tableIvySession =  (HashMap<InetAddress,SessionIvy>) this.srvCtxt.getAttribute("ivySessionTable");
			}
			if (tableIvySession!=null){
				firsttime =false;
			}// fin if tableSession!=null
			newBusTest = 0;
			
			byte buffer[] = new byte[myConf.taille()];
			DatagramPacket dataReceived = new DatagramPacket(buffer,buffer.length);
			try
			{
			socket.receive(dataReceived);
			if(dataReceived != null){
				if (firsttime){
					tableIvySession =  (HashMap<InetAddress,SessionIvy>) this.srvCtxt.getAttribute("ivySessionTable");
				}
				if (tableIvySession!=null){
					firsttime =false;
					System.out.print("d");
					//The module checks if there is a new Bus Ivy incoming
					SessionIvy clientIvy = tableIvySession.get(dataReceived.getAddress());
					if (clientIvy!=null){
						BusIvy_ current_bus = clientIvy.getBusIvy();
						if (current_bus==null){
							current_bus  = new BusIvy_();
							current_bus.setAddress(dataReceived.getAddress());
							current_bus.updateTime();
							clientIvy.setBusIvy(current_bus);
							myLog.write("New Ivy Connection detected : " + dataReceived.getAddress());
						}
						else
						{
							current_bus.updateTime();							
						}
						// check dead buses (after 10sec of inactivity)
						// TODO could be perform less often....
						Iterator<Map.Entry<InetAddress,SessionIvy>> itr = tableIvySession.entrySet().iterator();
						BusIvy_ checked_bus;
						while(itr.hasNext()){
							checked_bus  = itr.next().getValue().getBusIvy();
							if(!(checked_bus.isAlive())){
								myLog.write("Ivy Connection deconnected : " + checked_bus.getAddress());
								for(Integer deadDrone : checked_bus.getDrones())
								{
									deconnectDrone(deadDrone);
								}
							}
						}
						//
						/*
						if(busesIvy.isEmpty())
						{
							BusIvy_ oNewBus = new BusIvy_();
							oNewBus.setAddress(dataReceived.getAddress());
							oNewBus.updateTime();
							busesIvy.add(oNewBus);
							myLog.write("New Ivy Connection detected : " + dataReceived.getAddress());
						}
						else
						{
							for(BusIvy_ myBus : busesIvy)
							{
								if(myBus.getAddress().equals(dataReceived.getAddress()))
								{
									newBusTest = 1;
									myBus.updateTime();	 // to avoid being not alive and be removed
								}
								
								if(!(myBus.isAlive()))
								{
									myLog.write("Ivy Connection deconnected : " + myBus.getAddress());
									for(Integer deadDrone : myBus.getDrones())
									{
										deconnectDrone(deadDrone);
									}
									deadBusesIvy.add(myBus);
								}
								
							}
							if(!(deadBusesIvy.isEmpty()))
							{
								for(BusIvy_ deadBus : deadBusesIvy)
								{
									busesIvy.remove(deadBus);
								}
								deadBusesIvy.removeAll(deadBusesIvy);
							}
							if(newBusTest == 0)
							{
								BusIvy_ newBus = new BusIvy_();
								newBus.setAddress(dataReceived.getAddress());
								newBus.updateTime();
								busesIvy.add(newBus);
								myLog.write("New Ivy Connection detected : " + dataReceived.getAddress());
							}
						}
						*/
						AES cipher = clientIvy.getCipher();
						try{
							complete_data = dataReceived.getData(); // on recupere toute la trame utile
			            	true_length_of_msg = dataReceived.getLength();
			            	data_with_no_padding = new byte[true_length_of_msg]; // on recupere que les infos utiles
			            	for(i=0;i< true_length_of_msg;i++){
			            		data_with_no_padding[i]=complete_data[i];
			            	}
			            	try{
					            byte[] decrypted_data = cipher.decrypt(data_with_no_padding);
					            
								//The datagram is processed
								myDataSplited = dataProcessing(decrypted_data);	
								// switch ....
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
									/*
									for(BusIvy_ myBus : busesIvy)
									{
										if( myBus.getAddress().equals(dataReceived.getAddress()))
										{
											if(!(myBus.isOwnBy(Integer.parseInt(myDataSplited[1]))))
											{
												myBus.addDrones(Integer.parseInt(myDataSplited[1]));
											}
											
										}
									}*/				
								}	
								else
								{
									sendDataToServer(myDataSplited);
									if(!(current_bus.isOwnBy(Integer.parseInt(myDataSplited[0]))))
									{
										current_bus.addDrones(Integer.parseInt(myDataSplited[0]));
									}
									/*
									for(BusIvy_ myBus : busesIvy)
									{
										if( myBus.getAddress().equals(dataReceived.getAddress()))
										{
											if(!(myBus.isOwnBy(Integer.parseInt(myDataSplited[0]))))
											{
												myBus.addDrones(Integer.parseInt(myDataSplited[0]));
											}
										}
									}*/
								}
								
								
								System.out.print(".");
			            	}
			            	catch(BadPaddingException ex){
			            		ex.printStackTrace();
			            		System.out.println("msg no padding := "+(new String(Hex.encodeHex(data_with_no_padding))));
			            	}
			            	catch(IllegalBlockSizeException ex){
			            		ex.printStackTrace();
			            		System.out.println("msg no padding := "+(new String(Hex.encodeHex(data_with_no_padding))));
			            	}
						}
						catch(IOException ex){
							System.out.print("pbm decrypting msg");
							System.out.print(ex.toString());
						}					
				}// fin if ivyclient != null
				else
				{
					System.out.print("no ivy client stored for this datagramm from "+dataReceived.getAddress());
				}
			} // fin if tablesession != null
			else
			{
				// on a recu un diagramme mais il n'y a pas de session recorded donc on
				// doit envoyer un message au sender comme quoi il doit se relogger
				// TODO
				System.out.println("no session recorder "+dataReceived.getAddress()+ "should relog to server");
			}
			}// fin if datareceived!=null
		    }// fin du try socket received
			catch(SocketTimeoutException ex)
			{				
				if(!(busesIvy.isEmpty()))
				{
				for(BusIvy_ myBus : busesIvy)
					{
						myLog.write("Ivy Connection deconnected : " + myBus.getAddress());
						for(Integer deadDrone : myBus.getDrones())
						{
							deconnectDrone(deadDrone);
						}
					}
				
				busesIvy.removeAll(busesIvy);
				myLog.write("No Bus Ivy connected...");
				}
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
		//}
		//catch(Exception e){ 
		//	System.out.println("Non handled exception ... plantage....");
		//	System.out.println("***************************************");
		//	System.out.println(""+e.toString());
		//	System.out.println("***************************************");
		//}
		
	}
	
	/**
	 * send a iskill event when a drone has disappeared from an ivy bus
	 * @param droneId
	 * @throws Exception
	 */
	public void deconnectDrone(int droneId)
	{
		Event event = Event.createDataEvent("/data/drone/iskill");
		event.setField("aircraftId",droneId);
		event.setField("iskill", 1 + "" ); // 1 ==true
		Dispatcher.getInstance().multicast(event);
		//TODO effacer les fichiers de conf associés ????		
	}
	
	
	/**
	 * send a datagramm containing all useful information on a drone
	 * @param myDataSplited
	 * @throws Exception
	 */
	public void sendDataToServer(String myDataSplited[])// throws Exception
	{
		int i;
		int nbr_datas = myDataSplited.length;
		String[] myDataName = new String[nbr_datas];
	//	try{
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
			//System.out.println("acft "+ myDataSplited[0]+" heading :"+myDataSplited[5]);
	/*	}
		catch(Exception ex){
			System.out.println("exception dans sendDataToServer : ");
			System.out.println(""+ex.toString());
			for(int z= 0 ; z<myDataSplited.length;z++){
				System.out.print(myDataSplited[z]+" ");
			}
			System.out.println("\n"+ex.toString());
			throw new Exception(ex);
		}*/
	}
	
	/**
	 * This procedure will check in the DataBase if there is some orders given by
	 * the interface to the bus Ivy concerned.
	 * @param socket The socket through which the datas will be emitted
	 * @param busesIvy The object containing all the informations about the buses Ivy
	 */
	public void sendOrders(int aircraftWebId, String order,ArrayList <BusIvy_> listbusesIvy) throws Exception
	{
		 byte buffer[] = new byte[myConf.taille()];
		for(BusIvy_ myBus : listbusesIvy)
		{
			if(myBus.isOwnBy(aircraftWebId))
			{
				InetAddress address = myBus.getAddress();
				buffer = order.getBytes();
				DatagramPacket DataEmitted = new DatagramPacket(buffer, buffer.length, address, myConf.port());
				socket.send(DataEmitted); 
				myLog.write("Order Emitted To : " + DataEmitted.getAddress() + " ---> " + new String(DataEmitted.getData()));
				System.out.print("*");
			}
		}
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
				//System.out.println(""+myDataSplited.toString());
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
				//TODO TRAITER CE CAS ????
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
		/*}
		catch(Exception ex){
			System.out.println("exception dans sendOrderToServer : ");
			System.out.println(""+ex.toString());
			throw new Exception(ex);
		}*/
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
	
	private void updateXML_FPL_file(String acId,String wpId,String lat,String lon,String alt){
		// change XML file on server
		// http://cynober.developpez.com/tutoriel/java/xml/jdom/
		String default_folder = this.srvCtxt.getRealPath("");
	     try
	      {
	    	 String filepath = default_folder+"/upload/"+acId+"/flight_plan.xml";
	    	 //System.out.println("attempt to modify xml fpl for drone : " +acId);
	    	 Document document; 
	 		 SAXBuilder sxb = new SAXBuilder(); 
	 		 Element racine;
	    	 document = sxb.build(new File(filepath));// charge le fichier
	    	 //On initialise un nouvel élément racine avec l'élément racine du document.
		     racine = document.getRootElement();
		     List list_wpt = racine.getChild("flight_plan").getChild("waypoints").getChildren("waypoint");
		     Iterator i = list_wpt.iterator();
		     boolean doIt = true;
		     int cpt_wpt=0;
		     while(i.hasNext()&&doIt) //parcours des waypoints
		     {	       
		        Element courant = (Element)i.next();
		        if (cpt_wpt==(Integer.parseInt(wpId)-1)){ //TODO verifier que ca coincide bien !!! cf waypoint fictif ivy
		        	String name = courant.getAttributeValue("name");   
		        	// if(name.equals(wpt_name)) {
		        	System.out.println("wpt changed : "+ name + " for drone "+ acId +" id="+cpt_wpt+" with\tlat="+lat + "\tlon="+lon);
		        	doIt= false;
		        	courant.removeAttribute("x"); // on enleve les coordonnées relative en x et y 
		        	courant.removeAttribute("y"); // si elles sont présentes
		        	courant.setAttribute("lat", lat);
		        	courant.setAttribute("long", lon);
		        	courant.setAttribute("alt", alt);
		        }
		        cpt_wpt++;
		     }
		     // enregistre le nouveau fichier 
		     XMLOutputter sortie = new XMLOutputter(Format.getPrettyFormat());
	         sortie.output(document, new FileOutputStream(filepath));
	         //System.out.println("xml filed updated on server for drone : " +acId);
	      }
	      catch(Exception e){System.out.println("error : unable to load xml file");}
	      
	}
}
}
