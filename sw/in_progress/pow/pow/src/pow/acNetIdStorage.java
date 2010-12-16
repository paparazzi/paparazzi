package pow;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.StringReader;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.concurrent.ConcurrentHashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.apache.commons.httpclient.HttpClient;
import org.apache.commons.httpclient.HttpException;
import org.apache.commons.httpclient.HttpStatus;
import org.apache.commons.httpclient.methods.PostMethod;
import org.apache.commons.httpclient.methods.multipart.FilePart;
import org.apache.commons.httpclient.methods.multipart.MultipartRequestEntity;
import org.apache.commons.httpclient.methods.multipart.Part;
import org.apache.commons.httpclient.methods.multipart.StringPart;
import org.apache.commons.httpclient.protocol.Protocol;
import org.apache.commons.httpclient.protocol.ProtocolSocketFactory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.InputSource;

import fr.dgac.ivy.*;
/**
 * this structure memorize all the drone which exist on the ivy bus
 * @author genin
 *
 */
public class acNetIdStorage {

	private int ivyWebId;
	private Ivy bus;
	String url; // @server to upload
	private HashMap<String,acNetId> acNetIdMap;
	ConcurrentHashMap<String,pow.AcStatus> dronesStates;
	private int maxAircrafts;
	private acNetId resultat;
	
	public acNetIdStorage(int ii,Ivy b,String u,int maxAC, ConcurrentHashMap<String,pow.AcStatus> states ){
			bus = b;
			ivyWebId=ii;
			acNetIdMap= new HashMap<String,acNetId>();
			url = u;
			maxAircrafts = maxAC;
			dronesStates = states;
	}
	/**
	 * Return the identifiant of the drone on the web 
	 * @param idOnIvy
	 * @return
	 */
	public acNetId getAcNetId(String idOnIvy)
	{
		return acNetIdMap.get(idOnIvy);
	}
	/**
	 * Return the identifiant of the drone on the ivy bus 
	 * @param idOnWeb
	 * @return
	 */
	public String getAcIvyId(int idOnWeb)
	{   String res = null;
		boolean doIt = true;
		Collection<acNetId> col = acNetIdMap.values();
		Iterator<acNetId> it = col.iterator();
		while(it.hasNext()&& doIt)
		{
			acNetId ac=it.next();
			if (Integer.parseInt(ac.getIdOnWeb())==idOnWeb){ doIt=false; res = ac.getIdOnIvy();}
		}
		return res;	
	}
	/**
	 * search the drone net id of a drone from its ivy id
	 * if the drone is a new one, a new net id is requested to the server
	 * @param s the ivy id of the drone
	 */
	public void seekAcNetId(String s)  {
		String droneWebId;
		acNetId res = acNetIdMap.get(s);
		if (res==null){
			try {
				AcStatus acs = dronesStates.get(s);
				if (acs==AcStatus.UNKNOWN)
				{
					dronesStates.replace(s,AcStatus.ASKING_WEB_ID);
					droneWebId = getNewDroneWebId(s);
					dronesStates.replace(s,AcStatus.WEB_ID_RECEIVED);
					// lance message sur ivy pour recuperer conf, name et creer un obj acNetId
					 String rqst_id = bus.getWBUId(); 
					 String msgSend2Ivy = ivyWebId + " " + rqst_id + " CONFIG_REQ " + s + "";			
					 String msgtoBind= rqst_id +" (.*) CONFIG (.*) file://(.*) file://(.*) file://(.*) file://(.*) (.*) (.*)";
					 try {
						bus.bindMsgOnce(msgtoBind, new myListener(droneWebId));			
					 	try {
							dronesStates.replace(s,AcStatus.ASKING_IVY_CONF);
							bus.sendMsg(msgSend2Ivy);
						} catch (IvyException e) {
						e.printStackTrace();
						System.out.println("### IvyException : can't send request msg ###");
						}	
					 } catch (IvyException e) {
						 e.printStackTrace();
					 }	
			    }
			} catch (IvyConnectionExeption e) {
				System.out.println("### ERREUR on WEBID REQUEST for drone " +s+" ###");
				e.printStackTrace();
			}
		}
	}
	
	
	 /**
	  * upload de fichier de conf sur le serveur 
	  * verifier que ca marche sur long fichier 
	  * si probleme changer avec :
	  * 	factory.setSizeThreshold(yourMaxMemorySize);
	  * 	factory.setRepository(yourTempDirectory);
	  * 	upload.setSizeMax(yourMaxRequestSize);
	  * @throws InterruptedException 
	  * @throws FileNotFoundException
	  */
	
	public boolean uploadConfFile(acNetId ac) throws FileNotFoundException {
		boolean res=false ;
		String pathFPL = ac.getPlnPath();
		String pathSetting  = ac.getSettingPath();
		String droneWebId = ac.getIdOnWeb();
	    File f1 = new File(pathFPL);
        File f2 = new File(pathSetting);
        FilePart fp1 = new FilePart(f1.getName(), f1);
        FilePart fp2 = new FilePart(f2.getName(), f2);
        
       // System.out.println("File1 Length = " + f1.length());
        //System.out.println("File2 Length = " + f2.length());
        
    	  PostMethod filePost = new PostMethod(url);
    	  
    	 // filePost.addParameter("requestWebId", ""+webId);
    	  
    	  Part[] parts = {
    		  new StringPart("order", "uploadConfFile"),
    	      new StringPart("ivyWebId", ""+ivyWebId),
    	      new StringPart("droneWebId", ""+droneWebId),
    	      fp1,
    	      fp2
    	  };
    	  filePost.setRequestEntity(
    	      new MultipartRequestEntity(parts, filePost.getParams())
    	      );
    	  //filePost.addParameter("requestWebId", ""+webId);
    	  HttpClient client = new HttpClient();
    	  // inutile mais bon pour 1 connection de temps en temps ...
 		  Protocol stricthttps = new Protocol("https", (ProtocolSocketFactory)new StrictSSLProtocolSocketFactory(true), 443);
 		  client.getHostConfiguration().setHost("localhost", 443, stricthttps);
 		  //
    	  try {
    		  int status = client.executeMethod(filePost);
    		  System.out.println("reponse de la methode post uploading " +  HttpStatus.getStatusText(status)); 
    		  if (status != HttpStatus.SC_OK) {
    			  System.err.println("Method failed: ");
    		  }
    		  else{
    			  String responseBody = filePost.getResponseBodyAsString();
    			  try{
	    			  DocumentBuilder parser = DocumentBuilderFactory.newInstance().newDocumentBuilder();
					  Document document = parser.parse(new InputSource(new StringReader(responseBody)));
					  Element ack_node = (Element)document.getElementsByTagName("UploadAck").item(0);
					  int ack = Integer.parseInt(ack_node.getTextContent());
					  if (ack==1){
						 System.out.println("uploading ok for " +ack_node.getAttribute("iddrone")); 
						 res = true;
					  }
					  else{
						  System.out.println("uploading notok for " +ack_node.getAttribute("iddrone")); 
						  res = false;
					  }
    			  }
    			  catch(Exception e){
    				  System.err.println("error during the parsing of xml response to a upload request");
    				  e.printStackTrace();
    			  }
					
    		  }
    	  } catch (HttpException e) {
			 System.err.println("HttpException : echec execution requete post : uploading");
				e.printStackTrace();
		  } catch (IOException e) {
				 System.err.println("IOException : echec execution requete post : uploading");
				e.printStackTrace();
		  } finally {
		      // Release the connection.
			  filePost.releaseConnection();
		  }  
		  return res;
 }
	/**
	 * ask a unique new web id to the server for a new drone detected on the ivy bus
	 * @param IvyDroneId
	 * @return the new web drone id
	 * @throws IvyConnectionExeption
	 */
	public String getNewDroneWebId(String IvyDroneId) throws IvyConnectionExeption
	 {
		 String newDroneId = "#errorWebID#";
		 int statusCode=-1;
		 HttpClient client = new HttpClient();
		 client.getParams().setParameter("http.useragent", "Ivy request");
		 // inutile mais bon pour 1 connection  de temps en temps...
		 Protocol stricthttps = new Protocol("https", (ProtocolSocketFactory)new StrictSSLProtocolSocketFactory(true), 443);
		 client.getHostConfiguration().setHost("localhost", 443, stricthttps);
		 //
		 PostMethod method = new PostMethod(url);
		 method.addParameter("order", "requestNewDroneWebId");
		 method.addParameter("ivyWebId", ""+ivyWebId);
		 //method.addParameter("ivyDroneId",IvyDroneId);
		try {
			statusCode = client.executeMethod(method);
			 if (statusCode != HttpStatus.SC_OK) {
			        System.err.println("Method failed: " + method.getStatusLine());
			      }
			 else
			 {
				String responseBody = method.getResponseBodyAsString();//getResponseBody();//);
				//System.out.println(responseBody);
				if(responseBody.startsWith("<DroneWebId:"))
				{
					newDroneId = responseBody.substring(responseBody.indexOf(":") + 1,responseBody.indexOf(">"));
				}
				else
				{
					throw new IvyConnectionExeption("unable to reach server to get new drone webId");
				}
			 }		 
		} catch (HttpException e) {
			 System.err.println("HttpException : echec execution requete post : request new drone webId");
			e.printStackTrace();
		} catch (IOException e) {
			 System.err.println("IOException : echec execution requete post : request new drone webId");
			e.printStackTrace();
		} finally {
	      // Release the connection.
	      method.releaseConnection();
	    }  
		return newDroneId;
	 }
	/**
	 * inner class 
	 * seems requested so that the ivy send method works
	 * @author genin
	 *
	 */
	class myListener implements IvyMessageListener {
		private String droneWebId;
		
		myListener(String dwi){
			droneWebId=dwi;
		}
		@Override
		public void receive(IvyClient arg0, String[] args) {
			dronesStates.replace(args[1],AcStatus.IVY_CONF_RECEIVED);
			System.out.println("message rqst CONFIG received from ivy for drone "+args[1]);
        	String idOnIvy = args[1];
        	String flightplan_path = args[2];
        	String setting_path = args[5];
        	String drone_name = args[7];
        	String color_drone = checkColor(args[6]);        
        	acNetId ac = new acNetId(ivyWebId,droneWebId,idOnIvy,drone_name,flightplan_path,setting_path,color_drone,maxAircrafts);
        	acNetIdMap.put(idOnIvy, ac);
        	// upload des fichiers de conf sur le serveur
        	try {
        		dronesStates.replace(args[1],AcStatus.UPLOADING_CONF);
				boolean res = uploadConfFile(ac);
				if(res) {dronesStates.replace(args[1],AcStatus.CONF_OK);}
				else {
					dronesStates.replace(args[1],AcStatus.CONF_NOTOK);
					System.out.println("conf files upload of drone "+idOnIvy+" impossible, messages will be skipped");}
				//seekAcNetId(idOnIvy); // on recommence la recherche qui cette fois aboutira
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				System.out.println("### FileNotFoundException ###");
			}
		}
		/*
		 * on ivy drone color can be either in text format or in hexa format
		 * this function send the color in an hex rgb format
		 */
		private String checkColor(String color){
			String res = "";
			if ((color.length()==7)&&(color.charAt(0)=='#')){
				res = color;
			}
			else
			{
				if(color.equals("blue")){ res = "#0000FF"; }
				else if (color.equals("red")){ res = "#FF0000"; }
				else if (color.equals("green")){ res = "#00FF00"; }
				else {res = "#0F0F0F";}
			}
			return res;
		}
		
	}// fin inner class
	
}
