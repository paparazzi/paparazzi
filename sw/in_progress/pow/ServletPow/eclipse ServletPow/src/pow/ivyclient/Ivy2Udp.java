package pow.ivyclient;

import java.io.*;
import java.net.*;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

//import java.util.Vector;

import fr.dgac.ivy.*;

import org.apache.commons.codec.binary.Hex;
import org.apache.commons.httpclient.HttpClient;
import org.apache.commons.httpclient.HttpException;
import org.apache.commons.httpclient.methods.PostMethod;
import org.apache.commons.httpclient.HttpStatus;
import org.apache.commons.httpclient.protocol.Protocol;
import org.apache.commons.httpclient.protocol.ProtocolSocketFactory;

import java.io.StringReader;

import javax.crypto.BadPaddingException;
import javax.crypto.IllegalBlockSizeException;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
 
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.InputSource;

import pow.AES;
import pow.webserver.StrictSSLProtocolSocketFactory;

/**
 * It contact the server to get an identifier, some configuration parameter and
 * the key to encrypt and decrypt messages.
 * Then it listens messages on the ivy bus and send messages on active drones to the 
 * web server.
 * @author jabln, thomas genin
 * @version 2.0
 */
public class Ivy2Udp implements Runnable{
    private InetAddress serveur;
    private String login; 
    private String password;
    final static int portUdpSend = 8535;
    final static int portUdpReceive = 8536;
    private Ivy bus;
    private Thread heartbeatThread;
    private HeartBeat heartbeatRun;
    final static int taille = 1024;
    static int maxAircrafts = 1000; // TODO a changer !!! mettre une map ou qqch
    static int datagramLength = 17; // modif , on a rajoute la couleur a la fin
    private String datagramMatrix[][] = new String[maxAircrafts][datagramLength];// TODO a changer !!! mettre une map ou qqch
    private int aircraftNumber = 0;
    private Boolean aircraftAlive[] = new Boolean[maxAircrafts];// TODO a changer !!! mettre une map ou qqch
    private int count[] = new int[maxAircrafts];// TODO a changer !!! mettre une map ou qqch
    private int countSettings[]=new int[maxAircrafts];// TODO a changer !!! mettre une map ou qqch
    private HashMap<Integer,HashMap<Integer,Float>> settingsToBeChanged = new HashMap<Integer,HashMap<Integer,Float>>(); 
	private int webId=-999;
	private byte[] aesKey ;
	private AES aesCipher;
	private String url;
	private AcNetIdStorage acNetIds;
	private ConcurrentHashMap<String,AcStatus> dronesStatus;
	/* *********************** */
	private long time2resetWptFilter = 20*60*1000; // on nettoie les filtres tous les 20min !!!!!
	private MsgFilter waypointMsgFilter;
	private long chrono_start;
	private long elapsed_time;
	private long timeToSendValues=60*1000; // on envoie les values d'un drone toutes les 60 secondes
	private  HashMap<Integer,Long> lastvaluesMap;
	private long lastivymessage;
	/* ************************ */
	private long num_msg;
	private SimpleDateFormat dateformat;
	/* ************************** */
	private int socketTimeOut ;
	/* ************************* */
	private boolean continueListening ;
	private boolean reconnect = false;
	private HashSet<Integer> bindSet;
	IvyIHM ihm;
	 /**
	  * create an object which listens on the local ivy bus and send messages to the server 
	  * @param urlObj describes information about the server
	  * @param login 
	  * @param password
	  */
	 public Ivy2Udp(IvyIHM ihm,PowUrl urlObj,String login, String password)
	 {
		 this.ihm = ihm;
		 continueListening = true;
		 num_msg = 0;
			dateformat = new SimpleDateFormat("d:M:y:HH:mm:ss");
		try{
			this.login = login;
			this.password = password;
		 serveur = InetAddress.getByName(urlObj.getServerName());
		 // ********************************************* //
		  this.url =  urlObj.getWebUrl();
		 // ********************************************* //
		 dronesStatus = new ConcurrentHashMap<String,AcStatus>();
		 lastvaluesMap = new HashMap<Integer,Long>();
		 // ********************************************* //
		 waypointMsgFilter = new MsgFilter();
		 chrono_start = System.currentTimeMillis();
		 // ********************************************* //
		 bindSet = new HashSet<Integer>();
		 bus = new Ivy("Ivy2Udp", "Ivy2Udp Ready", null);
	 	} catch (Exception ie) {
	 		System.out.println("Error : cannot connect to server");
	 	}	 	
	 }
	 /* ******************************** */
	 /* *********** structure de correspondance entre webid et ivyid pour chaque drone ************ */
		/** initiate the object which allow to store information about drones */
	 	public void setStorage(){
			acNetIds = new AcNetIdStorage(webId,bus,url,maxAircrafts,dronesStatus);
		}
		/** provides the structure where data about the drone present on the ivy bus are stored  
		 * @return the structure which allow to store information about drones */
		public AcNetIdStorage getStorage(){
			return acNetIds ;
		}
		 /** return the id of the bus for the server
		  * this id is provided by the server and is unique  
		  * @return the unique id of this ivy bus for the server */
		 public int getIvyWebId(){
			 return webId;
		 }
	 /**
	  * request a single id for the bus to the web server
	  * method get
	  * see @link http://hc.apache.org/httpclient-3.x/tutorial.html
	  * see @link blogs.sun.com/gc/entry/unable_to_find_valid_certification
	  * see @link http://blogs.sun.com/andreas/entry/no_more_unable_to_find
	  */
	 public void getWebId() throws IvyConnectionExeption
	 {
		 int statusCode=-1;
		 // http config
		 HttpClient client = new HttpClient();
		 client.getParams().setParameter("http.useragent", "Ivy request");
		 Protocol stricthttps = new Protocol("https", (ProtocolSocketFactory)new StrictSSLProtocolSocketFactory(true), 443);
		 client.getHostConfiguration().setHost("localhost", 443, stricthttps);
		 PostMethod method = new PostMethod(url);
		 method.addParameter("order", "requestWebId");
		 method.addParameter("login", login);
		 method.addParameter("pwd", password);
		try {
			System.out.println("### envoie de la requete requestWebId###");
			statusCode = client.executeMethod(method);
			 if (statusCode != HttpStatus.SC_OK) {
			        System.err.println("Method failed: " + method.getStatusLine());
			        throw new IvyConnectionExeption("Erreur de retour de la requete de connection");			 
			      }
			 else
			 {
				// Read the response body.
				 try {
						String responseBody = method.getResponseBodyAsString();//getResponseBody();//);
						// Deal with the response
						try {
							// parse response in XML tree 
							// @SEE http://java.developpez.com/faq/xml/?page=dom
							DocumentBuilder parser = DocumentBuilderFactory.newInstance().newDocumentBuilder();
							Document document = parser.parse(new InputSource(new StringReader(responseBody)));
							// get login acknowledgment
							Element ack_node = (Element)document.getElementsByTagName("ConnectionAck").item(0);
							int ack = Integer.parseInt(ack_node.getTextContent());
							if (ack==1){
								// get webid node
								Element webid_node = (Element)document.getElementsByTagName("webid").item(0);
								webId = Integer.parseInt(webid_node.getTextContent());
								if (webId==-999) {throw new IvyConnectionExeption("unable to have correct webId");}
								// get key node 
								Element aeskey_node= (Element)document.getElementsByTagName("aeskey").item(0);
								String str_hex_aeskey = aeskey_node.getTextContent();
								// get parameter key node 
								Element aesiv_node= (Element)document.getElementsByTagName("aesiv").item(0);
								String str_hex_aesIvParameter = aesiv_node.getTextContent();
								// get periods of time
								Element time2resetWptFilter_node=(Element)document.getElementsByTagName("time2resetFilter").item(0);
								this.time2resetWptFilter =Long.parseLong(time2resetWptFilter_node.getTextContent());
								Element timeToSendValues_node =(Element)document.getElementsByTagName("timeToSendValues").item(0);
								this.timeToSendValues=Long.parseLong(timeToSendValues_node.getTextContent());
								Element socketTimeOut_node =(Element)document.getElementsByTagName("socketTimeOut").item(0);
								this.socketTimeOut=Integer.parseInt(socketTimeOut_node.getTextContent());
								//
								aesKey = Hex.decodeHex(str_hex_aeskey.toCharArray());
								byte[] iv = Hex.decodeHex(str_hex_aesIvParameter.toCharArray());
								aesCipher = new AES(aesKey,iv);
								//
								System.out.println("##########################");
								System.out.println("### connection ok+\t###");
								System.out.println("### web id = "+ webId   +"\t###");
								System.out.println("### aes key = "+ str_hex_aeskey +"\t###");
								System.out.println("##########################");
							}
							else if (ack==2){
								throw new IvyConnectionExeption("login ok, but aes encryption not supported by server");								
							}
							else if (ack==3){
								throw new IvyConnectionExeption("login ok, but server unable to generate a web id");								
							}
							else
							{
								throw new IvyConnectionExeption("login process failed, you're not allowed to access the server");								
							}							
						}
						catch(Exception e) {
							e.printStackTrace();
							throw new IvyConnectionExeption("unable to read the server response");
						}
				 }
				 catch (IOException e) {
						System.err.println("IOException : echec execution requete post : request webId");
						e.printStackTrace();
						throw new IvyConnectionExeption("No response from the server");
				 } 
			 }		 
		} catch (HttpException e) {
			e.printStackTrace();
			throw new IvyConnectionExeption("HttpException : echec execution requete post : request webId");
		} catch (IOException e) {
			e.printStackTrace();
			throw new IvyConnectionExeption("IOException : echec execution requete post : request webId");
	 	} finally {
	      method.releaseConnection(); // Release the connection.
	      
	    }
	 }
	 	 
	
	 /**
	  * create a thread which send heart beat message to the server
	  */
	 public void startHeartBeat(){
		 heartbeatRun = new HeartBeat(aesCipher,webId,serveur,portUdpSend,socketTimeOut/3);// socket timeout = 10000 for server
		 heartbeatThread = new Thread(heartbeatRun);
		 heartbeatThread.start();
		 System.out.println("process heartbeat started");
	 }
	 
	 /**
     * This method binds the object to some messages
     * of the bus Ivy. Each information that is needed by the web server is put in an
     * array. This array is sent to server.
     */
	 public void bindMsg2Ivy() throws IvyException {
		 for (int i = 0; i < countSettings.length; i++) {
            countSettings[i] = 0;
        }       

		for (int i = 0; i < count.length; i++) {
            count[i] = 0;
        }
        for (int i = 0; i < datagramMatrix.length; i++) {
            for (int j = 0; j < datagramMatrix[i].length - 1; j++) {
                datagramMatrix[i][j] = new String(" ");
            }
        }
        for (int i = 0; i < aircraftAlive.length; i++) {
            aircraftAlive[i] = true;
        }
        // fetch flight parameters
        // see message feature in  paparazzi folder
        // file /conf/message.xml
        // listen a specific message on the ivy bus and define a callback function
        bindSet.add(bus.bindMsg("ground FLIGHT_PARAM ([^ ]*) [^ ]* [^ ]* [^ ]* ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) .*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {
            	lastivymessage = System.currentTimeMillis();
                try {
                    AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE){
                		 a = new aircraftIdSeeker(args[0]);
                		 a.seek();
                		 int acIndex = a.getAcId();
                		 if (acIndex != -1) {
                             datagramMatrix[acIndex][3] = args[1];
                             datagramMatrix[acIndex][4] = args[2];
                             datagramMatrix[acIndex][5] = args[3];
                             datagramMatrix[acIndex][6] = args[4];
                             datagramMatrix[acIndex][7] = args[5];
                             datagramMatrix[acIndex][8] = args[6];
                             datagramMatrix[acIndex][9] = args[7];
                             send(acIndex);
                         }
                		 else // other states UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
                	}             
               } catch (Exception ie) {
            	   
                    System.out.println("Can't process FLIGHT_PARAM information");
                    ie.printStackTrace();
                }

            }
        }));

        bindSet.add(bus.bindMsg("ground ENGINE_STATUS ([^ ]*) [^ ]* [^ ]* [^ ]* [^ ]* ([^ ]*).*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {
            	lastivymessage = System.currentTimeMillis();
                try {
                	
                	AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE){
                		 a = new aircraftIdSeeker(args[0]);
                		 a.seek();
                		 int acIndex = a.getAcId();
                		 if (acIndex != -1) {
                			 datagramMatrix[acIndex][10] = args[1];
                             send(acIndex);
                         }
                		 else // other states UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
                	}            
                } catch (Exception ie) {
                    System.out.println("Can't process ENGINE_STATUS information");
                }

            }
        }));

        bindSet.add(bus.bindMsg("ground ENGINE_STATUS ([^ ]*) ([^ ]*) .*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {
            	lastivymessage = System.currentTimeMillis();
                try {
                	AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE){
                		 a = new aircraftIdSeeker(args[0]);
                		 a.seek();
                		 int acIndex = a.getAcId();
                		 if (acIndex != -1) {
                			 datagramMatrix[acIndex][12] = args[1];
                             send(acIndex);
                         }
                		 else // autres etats UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
                	}        
                } catch (Exception ie) {
                    System.out.println("Can't process AP_STATUS information");
                }

            }
        }));
		

        bindSet.add(bus.bindMsg("ground AP_STATUS ([^ ]*) [^ ]* [^ ]* [^ ]* [^ ]* ([^ ]*) .*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {
            	lastivymessage = System.currentTimeMillis();
                try {
                	AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE){
                		 a = new aircraftIdSeeker(args[0]);
                		 a.seek();
                		 int acIndex = a.getAcId();
                		 if (acIndex != -1) {
                			 datagramMatrix[acIndex][11] = args[1];
                             send(acIndex);
                         }
                		 else // autres etats UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
                	} 
                } catch (Exception ie) {
                    System.out.println("Can't process AP_STATUS information");
                }

            }
        }));

        // when a JUMP_TO_BLOCK message is sent after a web user order
        // we have to wait for this message to have a real acknowledgment of the order
        bindSet.add(bus.bindMsg("ground NAV_STATUS ([^ ]*) ([^ ]*).*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {
            	lastivymessage = System.currentTimeMillis();
                try {
                	AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE){
                		 a = new aircraftIdSeeker(args[0]);
                		 a.seek();
                		 int acIndex = a.getAcId();
                		 if (acIndex != -1) {
                			 datagramMatrix[acIndex][13] = args[1];
                			 // 
 	    					 String j2b[] = new String[1];
 	    					 j2b[0] = args[1];
 	    					 String s = new String("NAV_STATUS");	    			
 	    					 send(acIndex,s,j2b);
                			 
                         }
                		 else // autres etats UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
                	}
               } catch (Exception ie) {
                    System.out.println("Can't process NAV_STATUS information");
               }

            }
        }));
      // when a DL_SETTING message is sent after a web user order
        // we have to wait for this message to have a real acknowledgment of the order
        bindSet.add(bus.bindMsg("ground DL_VALUES ([^ ]*) ([^ ]*)", new IvyMessageListener(){
			public void receive(IvyClient client, String[] args){
				lastivymessage = System.currentTimeMillis();
				try{
					AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE){
                		 a = new aircraftIdSeeker(args[0]);
                		 a.seek();
                		 int acIndex = a.getAcId();
                		 if (acIndex != -1) {
                			 String[] settingSplited = args[1].split(",");
     						String settingId = new String ("");
     						settingId = settingId + countSettings[acIndex];
     						datagramMatrix[acIndex][14] = settingId;
     						datagramMatrix[acIndex][15] = settingSplited[countSettings[acIndex]];
     						send(acIndex);
     						countSettings[acIndex] = (countSettings[acIndex] + 1) % settingSplited.length;
     						// 	gestion de l'A/R de DL_SETTING			
     						// on attend que la valeur du setting soit effectivement change pour envoyer l'A/R
	                		if (settingsToBeChanged.containsKey(acIndex)){
	                			HashMap<Integer,Float> h = settingsToBeChanged.get(acIndex);
	                			Set<Integer> k = h.keySet();
	                			Iterator<Integer> itr = k.iterator();
	                			
	                			while(itr.hasNext()){
	                				int current_settingid = itr.next();
	                				float value = h.get(current_settingid);
	                				if(value==Float.parseFloat(settingSplited[current_settingid])){
	                					h.remove(current_settingid);
	                					String chgSet[] = new String[2];
	             						chgSet[0] = ""+current_settingid;
	             						chgSet[1] = ""+value;	
	        	     					String s = new String("DL_SETTING_ACK");
	        	     					send(acIndex,s,chgSet);			
	        	     					System.out.print("%");
	                				}
	                			}
	                		}
	                		// gestion de l'envoie de l'ensemble des valeurs de 
	                		// setting pour un drone , on envoie la trame par periode timeToSendValues
	                		Long lasttime = lastvaluesMap.get(acIndex);
	                		long nowtime =  System.currentTimeMillis();
	                		boolean firsttime = false;
	                		if (lasttime==null){
	                			lasttime = nowtime; 
	                			lastvaluesMap.put(acIndex,nowtime);
	                			firsttime = true ;
	                		}
	                		
	                		if (firsttime||((nowtime-lasttime)>timeToSendValues)){
	                			// on envoie la value
	                			String csv[] = new String[1];
	                			csv[0] = args[1];
	                			String s = new String("SETTINGS_VALUES");
	                			send(acIndex,s,csv);			
    	     					System.out.print("+");
    	     					lastvaluesMap.put(acIndex,nowtime);
	                		}
	                		
	                		
                		 }
                		 else // autres etats UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
                	}
				} catch(Exception ie) {
					System.out.println("Can't process DL_VALUES information");
					System.out.println("acid= "+args[0]);
					System.out.println("csv=  "+args[1]);
					System.out.println("********************\n"+ie.toString()+"\n********************");
				}
			}
		}));
        // when a MOVE_WAYPOINT message is sent after a web user order
        // we have to wait for this message to have a real acknowledgment of the order
        bindSet.add(bus.bindMsg("ground WAYPOINT_MOVED ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*)",new IvyMessageListener(){
			public void receive(IvyClient client, String[] args){
				lastivymessage = System.currentTimeMillis();
				try {
					AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE)
                		{
	                		a = new aircraftIdSeeker(args[0]);
	                		a.seek();
	                		int i = a.getAcId(); 
	     					String mvWp[] = new String[4];
	     					for (int j = 0;j<4;j++){
	     						mvWp[j] = args[j+1];
	     					}
	     					String s = new String("WAYPOINT_MOVED");
	     					send(i,s,mvWp);
                		 }
                		 else // autres etats UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
				}
				catch (Exception ie){
					System.out.println("Can't process MOVE_WAYPOINT information");
				}
			}
		}));
		// as there are many DL_VALUES messages on ivy bus, when a DL_SETTING order is sent
		// we have to keep a link between the order and the response we are waiting for having a real
		// acknowledgment
        bindSet.add(bus.bindMsg("DL_SETTING ([^ ]*) ([^ ]*) ([^ ]*)", new IvyMessageListener(){

			public void receive(IvyClient client, String[] args){
				lastivymessage = System.currentTimeMillis();
				try{
					AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE)
                		{
	                		a = new aircraftIdSeeker(args[0]);
	                		a.seek();
	                		int indexAircraft = a.getAcId(); 
	                		int indexSetting = Integer.parseInt(args[1]);
	                		float valueSetting = Float.parseFloat(args[2]);
	                		// memorisation qu'une requete DL_SETTING a ete faire
	                		// voir traitement dans ground DL_VALUES
	                		if (!settingsToBeChanged.containsKey(indexAircraft)){
	                			settingsToBeChanged.put(indexAircraft, new HashMap<Integer,Float>());               			
	                		}
	                		settingsToBeChanged.get(indexAircraft).put(indexSetting, valueSetting);
	                		System.out.print("$");            		
                		 }
                		 else // autres etats UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
				}
				catch (Exception ie){
					System.out.println("Can't process DL_SETTING information");
				}
			}
		}));
		// allow to be sure that a drone is still alive
		// this message is sent by ivy server to inform when the last message about a drone 
		// was sent
        bindSet.add(bus.bindMsg("ground TELEMETRY_STATUS ([^ ]*) ([^ ]*)", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {
            	lastivymessage = System.currentTimeMillis();
                try {
                	AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	aircraftIdSeeker a ;
                	if ((droneState==AcStatus.UNKNOWN)||(droneState==AcStatus.CONF_OK))
                	{     
                		a = new aircraftIdSeeker(args[0]);
                        a.seek();
                	}
                	else if (droneState==AcStatus.ALIVE)
                		{
	                		a = new aircraftIdSeeker(args[0]);
	                		a.seek();
	                		int i = a.getAcId(); 
	                        if (i != -1) {
	                            float f = Float.parseFloat(args[1]);
	                            if (aircraftAlive[i] && f > 30) {
	                                aircraftAlive[i] = false;
	                                String s = new String("Plane_Die");
	                                send(i, s);
	                            } else {
	                                if (!aircraftAlive[i] && f < 30) {
	                                    aircraftAlive[i] = true;
	                                    String s = new String("Plane_Resurect");
	                                    send(i, s);
	                                }
	                            }
	                        }
                		}
                } catch (Exception ie) {

                    System.out.println("Can't process TELEMETRY_STATUS information");
                }

            }
        }));

        bus.start(null);
    }
	 /**
	  * stop all the thread Ivy2Udp , HeartBeat and ivy2UdpWriting
	  */
	 public void stop_thread(){
		
		 Iterator<Integer> itr  = bindSet.iterator();
		 while(itr.hasNext()){
			 try {
				bus.unBindMsg(itr.next());
			} catch (IvyException e) {
				System.out.println("unable to unbind ivy msg");
			}
		 }
		 bus.stop();bus=null;
		 System.out.println("link to bus ivy stopped !");
		 if(this.heartbeatRun!=null) this.heartbeatRun.stop_thread();
		 continueListening = false;
	 }
	 /**
	  * add usefull info to msg for storage in database
	  * web_id
	  * number the message if we want to order then on the server (because udp is not safe)
	  */
	 private String add_info_msg(String msg){
		 String monHeure = this.dateformat.format(new Date());
		 String res = "" + this.webId+" " + this.num_msg + " " + monHeure +" " + msg;
		 this.num_msg++;
		 return res;
	 }
	 
    /**
     * This method send to the web server the datagram containing the informations
     * about the aircraft
     * @param acId Line of the matrix containing the informations about the plane
     * @throws java.lang.Exception
     */
    public void send(int acId){
    	DatagramSocket socketUdp = null;
    	String s = "";
    	if (count[acId] == 5) {
          if (aircraftAlive[acId]) {
                    byte buffer[];
                    s = "";
                    for (int i = 1; i < 7; i++) {
						s = s + datagramMatrix[acId][i] + " ";
                    }
					for(int i = 7; i< 10;i++){
					StringBuffer	str = new StringBuffer(datagramMatrix[acId][i]);
						str.setLength(5);
						s = s + str + " ";
					}
					for(int i = 10; i<datagramMatrix[0].length -1;i++){
						s = s + datagramMatrix[acId][i] + " ";
					}
                    s = s + datagramMatrix[acId][datagramMatrix[0].length - 1];
                    s = this.add_info_msg(s);
                    buffer = s.getBytes();
                    try{
	                    buffer = this.aesCipher.encrypt(buffer);
	                    if (buffer.length>taille){System.out.println("Alerte taille trame udp:"+buffer.length+" !!!!");}
	                    DatagramPacket dataUdp = new DatagramPacket(buffer, buffer.length, serveur, portUdpSend);
	                    try{
	                    	socketUdp = new DatagramSocket();
	                    	try{
	                    	socketUdp.send(dataUdp);
		                    System.out.print("#");
		                    count[acId] = 0;
	                    	}
	                    	catch(IOException ex){System.out.print("datagram creation error : msg not sended");}
	                    }
	                    catch (SocketException ex){System.out.print("socket error : msg not sended");}
	                    finally{if(socketUdp!=null){socketUdp.close();socketUdp=null;}}
	                    
                    }
                    catch( IOException ex){System.out.print("cipher error : cannot encrypt msg : msg not sended to server");ex.printStackTrace();}
                    catch(BadPaddingException ex){
        				ex.printStackTrace();
        				System.out.println("order := "+(new String(Hex.encodeHex(buffer))));
        				System.out.print("cannot encrypt msg : msg not sended to server");
        			}
        			catch(IllegalBlockSizeException ex){
        				ex.printStackTrace();
        				System.out.println("order := "+(new String(Hex.encodeHex(buffer))));
        				System.out.print("cannot encrypt msg : msg not sended to server");
        			}
                }
        } else {
            count[acId]++;
        }
    }
	

    /**
     * This method send to the web server a specific order with parameters
     * @param acId : Line of the matrix containing the informations about the plane
     * @param s : order
     * @param as : parameters 
     */

	public void send(int acId, String s, String[] as){
			String order = s;
			DatagramSocket socketUdp=null;
            s = s + " " + datagramMatrix[acId][1]+ " ";
			for (int i =0; i< as.length;i++){
				s = s + as[i] + " ";
			}
            byte buffer[];
	            // gestion de l'ecoute ou non des msg waypoint_moved et des autres msg speciaux					 	
			 	boolean isnew = true;
				// on filtre certains messages
			 	if (order.equals("WAYPOINT_MOVED")
			 			||order.equals("NAV_STATUS"))
			 	{
			 		elapsed_time = System.currentTimeMillis() - chrono_start;
					if (elapsed_time>time2resetWptFilter)
					{
						waypointMsgFilter.resetfilter();
						chrono_start = System.currentTimeMillis();
					}
		            isnew = waypointMsgFilter.isNew(s);
			 	}
	            if(isnew){
			        System.out.print("!");
			        s = this.add_info_msg(s);
		            buffer = s.getBytes();
                    try{
	                    buffer = this.aesCipher.encrypt(buffer);
	                    if (buffer.length>taille){System.out.println("Alerte taille trame udp:"+buffer.length+" !!!!");}
	                    DatagramPacket dataUdp = new DatagramPacket(buffer, buffer.length, serveur, portUdpSend);
	                    try{
	                    	socketUdp = new DatagramSocket();
	                    	try{
	                    	socketUdp.send(dataUdp);
	                    	}
	                    	catch(IOException ex){System.out.print("datagram creation error : msg not sended");}
	                    }
	                    catch (SocketException ex){System.out.print("socket error : msg not sended");}
	                    finally{if(socketUdp!=null){socketUdp.close();socketUdp=null;}}
                    }
                    catch (IOException ex){System.out.print("cipher error : cannot encrypt msg : msg not sended to server");ex.printStackTrace();}
                    catch(BadPaddingException ex){
        				ex.printStackTrace();
        				System.out.println("order := "+(new String(Hex.encodeHex(buffer))));
        				System.out.print("cannot encrypt msg : msg not sended to server");
        			}
        			catch(IllegalBlockSizeException ex){
        				ex.printStackTrace();
        				System.out.println("order := "+(new String(Hex.encodeHex(buffer))));
        				System.out.print("cannot encrypt msg : msg not sended to server");
        			}
                }
    }


	
    /**
     * This method send a particular message about an aircraft
     * @param acId line of the matrix containing the informations about the plane
     * @param s message that is to be sent to send
     */
    public void send(int acId, String s) {
    		DatagramSocket socketUdp= null;
            s = s + " " + datagramMatrix[acId][1];
            byte buffer[] ;
            s = this.add_info_msg(s);
            buffer = s.getBytes();
            System.out.println(s);
            try{
                buffer = this.aesCipher.encrypt(buffer);
                if (buffer.length>taille){System.out.println("Alerte taille trame udp:"+buffer.length+" !!!!");}
                DatagramPacket dataUdp = new DatagramPacket(buffer, buffer.length, serveur, portUdpSend);
                try{
                	socketUdp = new DatagramSocket();
                	try{
                	socketUdp.send(dataUdp);
                	}
                	catch(IOException ex){System.out.print("datagram creation error : msg not sended");}
                }
                catch (SocketException ex){System.out.print("socket error : msg not sended");}
                finally{if(socketUdp!=null){socketUdp.close();socketUdp=null;}}
            }
            catch (IOException ex){System.out.print("cannot encrypt msg : msg not sended to server");ex.printStackTrace();}
            catch(BadPaddingException ex){
				ex.printStackTrace();
				System.out.println("order := "+(new String(Hex.encodeHex(buffer))));
				System.out.print("cannot encrypt msg : msg not sended to server");
			}
			catch(IllegalBlockSizeException ex){
				ex.printStackTrace();
				System.out.println("order := "+(new String(Hex.encodeHex(buffer))));
				System.out.print("cannot encrypt msg : msg not sended to server");
			}
    }

    /**
     * listen the udp socket to get orders from web clients and send them to ivy
     */
    public void run() {
    	byte[] complete_data ;
    	int true_length_of_msg ;
    	int i;
    	long elapsed_ivy;
    	byte[] data_with_no_padding;
    	boolean heartbeat_is_running = false;
    	DatagramSocket socket = null;
		// login procedure
		setStorage();
		try{
			bindMsg2Ivy();
			this.startHeartBeat();
			// start the module which listens to the orders coming from web users 
			try
			{
				lastivymessage = System.currentTimeMillis();
				System.out.println("on lance udp writing ");
				int portUdpReceive = 8536 ;
				socket = new DatagramSocket(portUdpReceive);
				socket.setSoTimeout(this.socketTimeOut);
				while (continueListening) {
					// check last ivy message
					elapsed_ivy = System.currentTimeMillis()-lastivymessage;
					/*
					if(elapsed_ivy>10000) { this.heartbeatRun.stop_thread();heartbeat_is_running = false;}
					else if (!heartbeat_is_running){
						//Reconnection
						}
					*/
					//listening server (hertbeat and true messages
			            byte buffer[] = new byte[taille];
			            DatagramPacket data = new DatagramPacket(buffer, buffer.length);
			            try{
				            socket.receive(data);
				            String[] messageed;
				            try{
				            	complete_data = data.getData();
				            	// we have to remove the padding bytes present in the udp trame if the message is too short
				            	true_length_of_msg = data.getLength();
				            	data_with_no_padding = new byte[true_length_of_msg];
				            	for(i=0;i< true_length_of_msg;i++){
				            		data_with_no_padding[i]=complete_data[i];
				            	}
					            byte[] decrypted_data = aesCipher.decrypt(data_with_no_padding);
					            String s = new String(decrypted_data);
					            if (!s.equals("SERVER_HEARTBEAT")){
					            	System.out.print("data received from server web :  ");
						            System.out.println(s);
						            messageed = s.split(" ");
						            try{
						            	// creer un objet d'envoie qui connait la correspondance ivyid et webid pour chaque drone
						            	new Ivy2UdpWriting(messageed, getStorage()); 
						            }
						            catch(IvyException e1){ System.out.println("erreur sending msg to ivy : " + s);}
						            }
					         //   else {System.out.println("\nHEARTBEAT FROM SERVER");}
				            	}
				            catch(IOException ex){System.out.print("cipher error : cannot decrypt msg : msg not sended to ivy");ex.printStackTrace();}
				            catch(BadPaddingException ex){
								ex.printStackTrace();
								System.out.println("order := "+(new String(Hex.encodeHex(buffer))));
								System.out.print("cannot encrypt msg : msg not sended to server");
							}
							catch(IllegalBlockSizeException ex){
								ex.printStackTrace();
								System.out.println("order := "+(new String(Hex.encodeHex(buffer))));
								System.out.print("cannot encrypt msg : msg not sended to server");
							}
				        }
			            catch(SocketTimeoutException ex){
			            	// no heartbeat
			            	continueListening = false ;
			            	this.stop_thread();
			            	reconnect = true;
			            	ihm.setReLogged(true);
			            }
			            catch(IOException e){ e.printStackTrace();System.out.println("erreur socket when receive message");}
				}
				
				System.out.println("Thread Ivy2Udp stopped !");
			}
			catch(SocketException e){ System.out.println("erreur creation de socket server-->ivy ");}
			finally{if (socket!=null){socket.close();socket=null;}}
		
		}
		catch (IvyException e){
			System.out.println("unable to bind to ivy...connection failed");
			e.printStackTrace();
			System.exit(0);			
		}
		
	}
   
    
    /**
     * Inner class which manage to search in acNetIdStorage, the index of a plane according to its ivyid
     * if the drone is unknown , an identifiant is requested to the server and its conf files are uploaded
     */
    class aircraftIdSeeker {
    	private String droneId;
    	private int res; 
    	
    	public aircraftIdSeeker(String s){
    		
    		droneId = s;
    		res = -1;
    	}
    	
    	public int getAcId() {return res;}
    	
    	public void seek(){
    		// seek if the drone status
    	   	AcStatus droneState = dronesStatus.get(droneId);
    		int i = 0;   	  
    		// ALIVE means that the information about the drone were already fetched and so
    		// we can directly seek them on the datamatrix
    	    if (droneState==AcStatus.ALIVE){
    	    	while (i < maxAircrafts && !droneId.equals(datagramMatrix[i][0])) {
    	            i++;
    	        }
    	        if (i < aircraftNumber) {
    	            res =  (i);
    	        }
    	        else {
    	        	// big error should not arrive
    	        	System.out.println("ERROR , ALIVE BUT NOT IN MATRIX !!!");
    	        }
    	    }//
    	    // if status in unknown we have to seek th information on the ivy bus and
    	    //ask the server a new web id for the drone
    	    else if (droneState==AcStatus.UNKNOWN)
    	    	{
		        	acNetIds.seekAcNetId(droneId);	        	      
		    }// if conf is ok , we can fill the datamatrix and put the drone in ALIVE STATUS
    	    // when the next message for this bus will arrive it will be processed normally
    	    else if (droneState==AcStatus.CONF_OK) {
    	    	AcNetId ac = acNetIds.getAcNetId(droneId);	  
    	    	datagramMatrix[aircraftNumber][0] = ac.getIdOnIvy(); // id on IvyBus
	            datagramMatrix[aircraftNumber][1] = ac.getIdOnWeb(); // id on Web Client
	            datagramMatrix[aircraftNumber][2] = ac.getName(); // name
	            datagramMatrix[aircraftNumber][16] = ac.getColor();
	        	dronesStatus.replace(droneId,AcStatus.ALIVE);
        		//
	            if (datagramMatrix[aircraftNumber][0].equals(" ")) {
	            	 res= (-1);
	            } else {
	                aircraftNumber++;
	                String newPlane = new String("New_Plane");
	                String name[] = new String[1];
	                name[0] = ac.getName();
	                send(aircraftNumber - 1, newPlane,name);
	                res= (aircraftNumber - 1);
	            }		 
    	    }
    	    else { // that means that the drone is in a intermediate status we have to wait that
    	    	// the status becomes CONF_OK to continue ( on the next ivy message)
    	    	// do nothing
    	    }
    	} 	
    } //fin class acId
	
} // fin class ivy2udpreading







