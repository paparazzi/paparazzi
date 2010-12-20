package pow;

import java.io.*;
import java.net.*;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

//import java.util.Vector;

import fr.dgac.ivy.*;
import java.lang.*;


import org.apache.commons.codec.binary.Hex;
import org.apache.commons.httpclient.HttpClient;
import org.apache.commons.httpclient.HttpException;
import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.httpclient.UsernamePasswordCredentials;
import org.apache.commons.httpclient.auth.AuthScope;
import org.apache.commons.httpclient.methods.GetMethod;
import org.apache.commons.httpclient.methods.PostMethod;
import org.apache.commons.httpclient.methods.multipart.FilePart;
import org.apache.commons.httpclient.methods.multipart.MultipartRequestEntity;
import org.apache.commons.httpclient.methods.multipart.Part;
import org.apache.commons.httpclient.methods.multipart.StringPart;
import org.apache.commons.httpclient.HttpStatus;
import org.apache.commons.httpclient.params.HttpMethodParams;
import org.apache.commons.httpclient.DefaultHttpMethodRetryHandler;

import java.util.concurrent.Semaphore;

import java.io.StringReader;

import javax.crypto.BadPaddingException;
import javax.crypto.IllegalBlockSizeException;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
 
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.InputSource;

/**
 * This aims to send Udp datagram containing information on the active drones
 * to the web serveur
 * @author jabln, thomas genin
 * @version 2.0
 */
class Ivy2UdpReading2 {

    static InetAddress serveur;
    final static int portUdpSend = 8535;
    final static int portUdpReceive = 8536;
    private Ivy bus,tempsbus;
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
	private acNetIdStorage acNetIds;
	private ConcurrentHashMap<String,AcStatus> dronesStatus;
	/* *********************** */
	private static long time2resetWptFilter = 20*60*1000; // on nettoie les filtres tous les 20min !!!!!
	private MsgFilter waypointMsgFilter;
	private long chrono_start;
	private long elapsed_time;
	private long chrono_start2;
	private long timeToSendValues=60*1000; // on envoie les values d'un drone toutes les 60 secondes
	private  HashMap<Integer,Long> lastvaluesMap;
	
	/* *********** structure de correspondance entre webid et ivyid pour chaque drone ************ */
	public void setStorage(){
		acNetIds = new acNetIdStorage(webId,bus,url,maxAircrafts,dronesStatus);
	}
	
	public acNetIdStorage getStorage(){
		return acNetIds ;
	}
	/* ************************* */
	 public Ivy2UdpReading2(String hostname)
	 {
		try{
		 serveur = InetAddress.getByName(hostname);
		 // ********************************************* //
		 url = "http://"+hostname+":8080/TestServletPow/Ivy2TomcatHttpServer.srv";
		 // ********************************************* //
		 dronesStatus = new ConcurrentHashMap<String,AcStatus>();
		 lastvaluesMap = new HashMap<Integer,Long>();
		 // ********************************************* //
		 waypointMsgFilter = new MsgFilter();
		 chrono_start = System.currentTimeMillis();
		 chrono_start2 = System.currentTimeMillis();
		 // ********************************************* //
		 bus = new Ivy("Ivy2Udp", "Ivy2Udp Ready", null);
	 	} catch (Exception ie) {
	 		System.out.println("Error : cannot connect to server");
	 	}	 	
	 }
	 
	 public int getIvyWebId(){
		 return webId;
	 }
	 /**
	  * methode qui interroge le serveur web tomcat pour lui demander un identifiant unique
	  * methode get
	  * @SEE http://hc.apache.org/httpclient-3.x/tutorial.html
	  */
	 public void getWebId(String login , String pwd) throws IvyConnectionExeption
	 {
		 int statusCode=-1;
		 HttpClient client = new HttpClient();
		 client.getParams().setParameter("http.useragent", "Ivy request");
		 PostMethod method = new PostMethod(url);
		/* 
		 NameValuePair[] data = {
		          new NameValuePair("requestWebId", "xxx"),
		          new NameValuePair("password", "bloggs")
		        };
		 method.setRequestBody(data);
         */		 
		 
		 //	 static InetAddress 	InetAddress.getByAddress(byte[] addr) ;
		
		 method.addParameter("order", "requestWebId");
		 method.addParameter("login", login);
		 method.addParameter("pwd", pwd);
		try {
			System.out.println("### envoie de la requete requestWebId###");
			statusCode = client.executeMethod(method);
			 if (statusCode != HttpStatus.SC_OK) {
			        System.err.println("Method failed: " + method.getStatusLine());
			       //System.exit(0);// on clot l'appli
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
								// get match key node 
								Element aesiv_node= (Element)document.getElementsByTagName("aesiv").item(0);
								String str_hex_aesIvParameter = aesiv_node.getTextContent();
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
		 
		 /* HttpClient client = new HttpClient();
		  //client.getState().setCredentials(
		  //          new AuthScope("ivybus", 443, "realm"),
		  //          new UsernamePasswordCredentials("username", "password")
		  //      );
		  GetMethod method = new GetMethod(url);
		  // method.setDoAuthentication( true );
          // Provide custom retry handler is necessary
		  method.getParams().setParameter(HttpMethodParams.RETRY_HANDLER, 
		    		new DefaultHttpMethodRetryHandler(3, false));
	 */
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
       // bus = new Ivy("Ivy2Udp", "Ivy2Udp Ready", null);

        bus.bindMsg("ground FLIGHT_PARAM ([^ ]*) [^ ]* [^ ]* [^ ]* ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) .*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {

                try {
                    //int acId = acId(args[0]);
                	AcStatus droneState=dronesStatus.putIfAbsent(args[0],AcStatus.UNKNOWN);
                	//AcStatus droneState = dronesStatus.get(args[0]);
                	aircraftIdSeeker a ;
                	//System.out.println("status drones "+args[0] + ": " + droneState);
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
                		 else // autres etats UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
                	}             
                } catch (Exception ie) {
                    System.out.println("Can't process FLIGHT_PARAM information");
                }

            }
        });

        bus.bindMsg("ground ENGINE_STATUS ([^ ]*) [^ ]* [^ ]* [^ ]* [^ ]* ([^ ]*).*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {

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
                		 else // autres etats UNKNOWN,ASKING_IVY_CONF,ASKING_WEB_ID,
                		 {
                			 //do nothing // skip message
                		 }
                	}            
                } catch (Exception ie) {
                    System.out.println("Can't process ENGINE_STATUS information");
                }

            }
        });

		bus.bindMsg("ground ENGINE_STATUS ([^ ]*) ([^ ]*) .*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {

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
        });
		

        bus.bindMsg("ground AP_STATUS ([^ ]*) [^ ]* [^ ]* [^ ]* [^ ]* ([^ ]*) .*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {

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
        });

        bus.bindMsg("ground NAV_STATUS ([^ ]*) ([^ ]*).*", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {

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
 	    					 //send(acIndex);
 	    					 send(acIndex,s,j2b);
                			 //
                             //send(acIndex);
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
        });


		bus.bindMsg("ground DL_VALUES ([^ ]*) ([^ ]*)", new IvyMessageListener(){

			public void receive(IvyClient client, String[] args){
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
	                				//System.out.println(""+current_settingid+" "+ value +"=?" +settingSplited[current_settingid]);
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
						


		});


		
	//	bus.bindMsg("MOVE_WAYPOINT ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*)",new IvyMessageListener(){
		
		// on surveille les messages WAYPOINT_MOVED pour traquer l'A/R d'un ordre venant d'un client web 
		// qui consisterait a deplacer un waypoint
		bus.bindMsg("ground WAYPOINT_MOVED ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*) ([^ ]*)",new IvyMessageListener(){
			public void receive(IvyClient client, String[] args){
				try {
					//int i = acId(args[0]);
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
		});
		// on traque ce message pour assurer l'A/R d'un ordre venant d'un client web qui viserait
		// a modifier un  parametre d'un drone
		bus.bindMsg("DL_SETTING ([^ ]*) ([^ ]*) ([^ ]*)", new IvyMessageListener(){

			public void receive(IvyClient client, String[] args){
				try{
					//int i = acId(args[0]);
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
		});
		
        bus.bindMsg("ground TELEMETRY_STATUS ([^ ]*) ([^ ]*)", new IvyMessageListener() {

            public void receive(IvyClient client, String[] args) {

                try {
                	//int i = acId(args[0]);
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
        });

        bus.start(null);
    }

    /**
     * This method send to the web server the datagram containing the informations
     * about the aircraft
     * @param acId Line of the matrix containing the informations about the plane
     * @throws java.lang.Exception
     */
    public void send(int acId){// throws Exception {
    	String s = "";
    	if (count[acId] == 5) {
          //  try {
                if (aircraftAlive[acId]) {
                    byte buffer[];// = new byte[taille];
                    s = "";
                    for (int i = 1; i < 7; i++) {
						s = s + datagramMatrix[acId][i] + " ";
                    }
					for(int i = 7; i< 10;i++){
						//StringBuffer str = new StringBuffer(4);
					StringBuffer	str = new StringBuffer(datagramMatrix[acId][i]);
						str.setLength(5);
						s = s + str + " ";
					}
					for(int i = 10; i<datagramMatrix[0].length -1;i++){
						s = s + datagramMatrix[acId][i] + " ";
					}
                    s = s + datagramMatrix[acId][datagramMatrix[0].length - 1];
                    //System.out.println(s);
                    
                    buffer = s.getBytes();
                    try{
	                    buffer = this.aesCipher.encrypt(buffer);
	                    if (buffer.length>taille){System.out.println("Alerte taille trame udp:"+buffer.length+" !!!!");}
	                    DatagramPacket dataUdp = new DatagramPacket(buffer, buffer.length, serveur, portUdpSend);
	                    try{
	                    	DatagramSocket socketUdp = new DatagramSocket();
	                    	try{
	                    	socketUdp.send(dataUdp);
		                    System.out.print("#");
		                    count[acId] = 0;
	                    	}
	                    	catch(IOException ex){System.out.print("datagram creation error : msg not sended");}
	                    }
	                    catch (SocketException ex){System.out.print("socket error : msg not sended");}
	                    
                    }
                    catch( IOException ex){System.out.print("cipher error : cannot encrypt msg : msg not sended to server");ex.printStackTrace();}             
                }
            //} 
            //catch (IOException ioex){ System.out.println("socket send exception, Can't send message " + s);}
            //catch (Exception ie) {
            //    System.out.println("Can't send message on the web");
            //}
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
            s = s + " " + datagramMatrix[acId][1]+ " ";
			for (int i =0; i< as.length;i++){
				s = s + as[i] + " ";
			}
            byte buffer[];//= new byte[taille];
            //System.out.println(s);
	            // gestion de l'ecoute ou non des msg waypoint_moved et des autres msg speciaux			
			 	
			 	boolean isnew = true;
				// on filtre certains messages
			 	if (order.equals("WAYPOINT_MOVED")
			 			||order.equals("NAV_STATUS"))
			 	{
			 		//System.out.println("elapsed time : "+elapsed_time);
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
		            buffer = s.getBytes();
                    try{
	                    buffer = this.aesCipher.encrypt(buffer);
	                    if (buffer.length>taille){System.out.println("Alerte taille trame udp:"+buffer.length+" !!!!");}
	                    DatagramPacket dataUdp = new DatagramPacket(buffer, buffer.length, serveur, portUdpSend);
	                    try{
	                    	DatagramSocket socketUdp = new DatagramSocket();
	                    	try{
	                    	socketUdp.send(dataUdp);
	                    	}
	                    	catch(IOException ex){System.out.print("datagram creation error : msg not sended");}
	                    }
	                    catch (SocketException ex){System.out.print("socket error : msg not sended");}
	                    
                    }
                    catch (IOException ex){System.out.print("cipher error : cannot encrypt msg : msg not sended to server");ex.printStackTrace();}
                }
    }


	
    /**
     * This method send a particular message about an aircraft
     * @param acId line of the matrix containing the informations about the plane
     * @param s message that is to be sent to send
     */
    public void send(int acId, String s) {
            s = s + " " + datagramMatrix[acId][1];
            byte buffer[] ;//= new byte[taille];
            buffer = s.getBytes();
            System.out.println(s);
            try{
                buffer = this.aesCipher.encrypt(buffer);
                if (buffer.length>taille){System.out.println("Alerte taille trame udp:"+buffer.length+" !!!!");}
                DatagramPacket dataUdp = new DatagramPacket(buffer, buffer.length, serveur, portUdpSend);
                try{
                	DatagramSocket socketUdp = new DatagramSocket();
                	try{
                	socketUdp.send(dataUdp);
                	}
                	catch(IOException ex){System.out.print("datagram creation error : msg not sended");}
                }
                catch (SocketException ex){System.out.print("socket error : msg not sended");}
                
            }
            catch (IOException ex){System.out.print("cannot encrypt msg : msg not sended to server");ex.printStackTrace();}
    }

    /*
     * 
     */
    public static void main(String argv[]) throws IvyException {
            //    serveur = InetAddress.getByName(argv[0]);
    	if (argv.length<1){
    	 System.out.println("failure :1 params required : [server name or url]");	
    	}
    	else {
    		byte[] complete_data ;
        	int true_length_of_msg ;
        	int i;
        	byte[] data_with_no_padding;
        	String hostname = argv[0];
        	Ivy2UdpReading2 link = new Ivy2UdpReading2(hostname );
        	try {
        		 // seek user login info 
        		String login="admin_ivy";
				String password = "pwdadmin_ivy";
				link.getWebId(login, password); // login procedure
				//
				link.setStorage();
				link.bindMsg2Ivy();
				// on lance le reading
				try
				{
					System.out.println("on lance udp writing ");
					int portUdpReceive = 8536 ;
					DatagramSocket socket = new DatagramSocket(portUdpReceive);
					while (true) {
				            byte buffer[] = new byte[taille];
				            DatagramPacket data = new DatagramPacket(buffer, buffer.length);
				            try{
					            socket.receive(data);
					            System.out.print("data received from server web :  ");
					            String[] messageed;
					            try{
					            	complete_data = data.getData();
					            	true_length_of_msg = data.getLength();
					            	data_with_no_padding = new byte[true_length_of_msg];
					            	for(i=0;i< true_length_of_msg;i++){
					            		data_with_no_padding[i]=complete_data[i];
					            	}
						            byte[] decrypted_data = link.aesCipher.decrypt(data_with_no_padding);
						            //String s = new String(data.getData());System.out.println(s);
						            String s = new String(decrypted_data);System.out.println(s);
						            messageed = s.split(" ");
						            try{
						            	// creer un objet d'envoie qui connait la correspondance ivyid et webid pour chaque drone
						            	new Ivy2UdpWriting(messageed, link.getStorage()); 
						            }
						            catch(IvyException e1){ System.out.println("erreur sending msg to ivy : " + s);}
						            }
					            catch(IOException ex){System.out.print("cipher error : cannot decrypt msg : msg not sended to ivy");ex.printStackTrace();}
					        }
				            catch(IOException e){ System.out.println("erreur socket when receive message");}
					}				    
				}
				catch(SocketException e){ System.out.println("erreur creation de socket server-->ivy ");}
				//
			} catch (IvyConnectionExeption e) {
				System.out.println(e.toString());
				e.printStackTrace();
				System.exit(0);			}		
    	}
    }
    /*
     * Inner class which manage to find the index of a plane according to its ivyid
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
    	   	AcStatus droneState = dronesStatus.get(droneId);
    		int i = 0;   	  
    	    if (droneState==AcStatus.ALIVE){
    	    	while (i < maxAircrafts && !droneId.equals(datagramMatrix[i][0])) {
    	            i++;
    	        }
    	        if (i < aircraftNumber) {
    	            res =  (i);
    	        }
    	        else {
    	        	// gtrosse erreur 
    	        	System.out.println("ERROR , ALIVE BUT NOT IN MATRIX !!!");
    	        }
    	    }
    	    else if (droneState==AcStatus.UNKNOWN)
    	    	{
		        	acNetIds.seekAcNetId(droneId);	        	      
		    }
    	    else if (droneState==AcStatus.CONF_OK) {
    	    	acNetId ac = acNetIds.getAcNetId(droneId);	  
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
	                //int acWebId = Integer.parseInt(ac.getIdOnWeb());
	                //send(acWebId, newPlane);
	                res= (aircraftNumber - 1);
	            }		 
    	    }
    	    else {
    	    	// do thing
    	    }
    	} 	
    } //fin class acId
} // fin class ivy2udpreading







