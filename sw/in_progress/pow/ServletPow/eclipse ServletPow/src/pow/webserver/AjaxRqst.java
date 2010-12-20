package pow.webserver;


import java.io.*;
import java.net.*;
import java.util.Iterator;
import java.util.Map;

import javax.crypto.BadPaddingException;
import javax.crypto.IllegalBlockSizeException;
import javax.servlet.ServletConfig;
import javax.servlet.ServletContext;
import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.HttpSession;
// SEE http://java.developpez.com/faq/xml/?page=dom
import javax.xml.parsers.*;
import javax.xml.transform.*;
import javax.xml.transform.dom.*;
import javax.xml.transform.stream.*;

import org.w3c.dom.*;
import org.xml.sax.*;

import org.apache.commons.codec.binary.Hex;

/*import org.jdom.*;
import org.jdom.input.*;
import org.jdom.output.Format;
import org.jdom.output.XMLOutputter;
import org.jdom.filter.*;
*/
import java.util.*;

import pow.AES;
import pow.ivyclient.BusIvy_;
import pow.webserver.Conf;

/**
 * Servlet implementation class ajaxRqst
 * handles the different request from a web client which wants to give orders to a drone
 * and it handles administrator queries to manage users' profiles
 */
public class AjaxRqst extends HttpServlet {
	private static final long serialVersionUID = 1L;
	private String default_folder="blabla";
	private ServletContext srvCtxt;
	private Conf myConf; 
    /**
     * @see HttpServlet#HttpServlet()
     */
    public AjaxRqst() {
        super();
    }
    
    /**
     * init the servlet 
     */
    public void init(ServletConfig config) throws ServletException {
    	super.init(config); // necessaire sinon getServletContext renvoie null dans doPost
    	default_folder = config.getServletContext().getRealPath("");
    	srvCtxt = config.getServletContext();
    	myConf = new Conf(default_folder,"pow_conf.xml");
    }

	/**
	 * handles different request from web users :
	 * orders to send to ivy buses and admin request to deal with users' account as well
	 * @see HttpServlet#doPost(HttpServletRequest request, HttpServletResponse response)
	 */
	protected void doPost(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
		 HttpSession session = request.getSession(true);
		 // recupere les parametres
		 PrintWriter out = response.getWriter();
		 String order = request.getParameter("order"); // fpl_update...
		 if(order!=null) {
			 if (order.equals("fpl_update"))
			 {
				 System.out.println("move waypoint requested !!!");
					 String wpt_name 		= request.getParameter("wpt_name");
					 String aircraft_id 	= request.getParameter("aircraft_id");
					 String wpt_id 			= request.getParameter("wpt_id");
					 String new_alt 		= request.getParameter("new_alt");
					 String new_lat  	    = request.getParameter("new_lat");
					 String new_lon  	    = request.getParameter("new_lon");
					 String new_alt_for_fpl = request.getParameter("new_alt_for_fpl");
					 String dragged         = request.getParameter("dragged");
					 doFplUpdate(wpt_name,aircraft_id, wpt_id,new_alt, new_lat, new_lon, new_alt_for_fpl);
					 response.setContentType("text/xml");
					 out.println("<reponse>");
					 out.println("<waypoint_to_move idwpt='"+wpt_id+"' acid='"+aircraft_id+"' newlat='"+new_lat+"' newlon='"+new_lon+"' dragged='"+dragged+"'/>");
					 out.println("</reponse>");
						
			 }
			 else if (order.equals("activate_block")){
				 System.out.println("block activation requested !!!");
				 String block_id = request.getParameter("block_id");
				 String aircraft_id  = request.getParameter("aircraft_id");
				 doActivateBlock(block_id ,aircraft_id);
				 response.setContentType("text/xml");
				 out.println("<reponse>");
				 out.println("<block_to_activate acid='"+aircraft_id+"' idblock='"+block_id+"' />");
				 out.println("</reponse>");
			 }
			 else if (order.equals("modif_setting")){
				 System.out.println("modif setting requested !!!");
				 String aircraft_id = request.getParameter("aircraft_id");
				 String setting_id = request.getParameter("setting_id");
				 String newsetting_value = request.getParameter("value");
				 doModifSetting(aircraft_id,setting_id,newsetting_value);
				 response.setContentType("text/xml");
				 out.println("<reponse>");
				 out.println("<setting_to_change acid='"+ aircraft_id +"' setting_id='"+setting_id+"' />");
				 out.println("</reponse>");
			 }
			 else if (order.equals("create_user")){
				 String login 		= request.getParameter("login");
				 String pwd 		= request.getParameter("pwd");
				 String right  		= request.getParameter("right");
				 HashSet<String> set_of_drone = null;
				 if (right.equals("user")){
					 String liste_drone = request.getParameter("immats");
					 StringTokenizer st = new StringTokenizer(liste_drone,";");
					 set_of_drone = new HashSet<String>();
					 try{
					     while (st.hasMoreTokens()) {
					    	 set_of_drone.add( st.nextToken());
					     }
					 }
					 catch (NoSuchElementException ex){}
				 }
				 boolean res = doCreateUser(login,pwd,right,set_of_drone);
				 response.setContentType("text/xml");
				 out.println("<reponse>");
				 if(res){
					 out.println("<create_user login='"+login+"' status='OK'/>");
				 }
				 else{
					 out.println("<create_user login='"+login+"' status='NOTOK'/>");
				 }
				 out.println("</reponse>");
			 }
			 else if (order.equals("delete_user")){
				 String login 		= request.getParameter("login");
				 boolean res = doDeleteUser(login);
				 response.setContentType("text/xml");
				 out.println("<reponse>");
				 if(res){
					 out.println("<delete_user login='"+login+"' status='OK'/>");
				 }
				 else{
					 out.println("<delete_user login='"+login+"' status='NOTOK'/>");
				 }
				 out.println("</reponse>");
			 }
			 else if (order.equals("info_user")){
				 String login = request.getParameter("login");
				 UserTab logTab = UserTab.unserialize(default_folder + "/conf/"+"userTable.tbl");
				 User usr = logTab.seek(login);
				 String right = getRight(usr.getRights());
				 String liste = usr.getListDrone();
				 response.setContentType("text/xml");
				 out.println("<reponse>");
				 out.println("<info_user login='"+login+"' right='"+right+"' liste_drone='"+liste+"'/>");
				 out.println("</reponse>"); 
			 }
			 else if (order.equals("modify_user")){
				 String login = request.getParameter("login");
				 String pwd = request.getParameter("pwd");
				 String right = request.getParameter("right");
				 String liste_drone = request.getParameter("liste_drone");
				 StringTokenizer st = new StringTokenizer(liste_drone,";");
				 UserTab logTab = UserTab.unserialize(default_folder + "/conf/"+"userTable.tbl");
				 User usr = logTab.seek(login);
				 if (!pwd.equals("")) {usr.setPwd(pwd);}
				 usr.setRights(getRight(right));
				 usr.clearListDrone();
				 try{
				     while (st.hasMoreTokens()) {
				    	 usr.addDrone(st.nextToken());
				     }
				 }
				 catch (NoSuchElementException ex){}
				 logTab.serialize(default_folder + "/conf/"+"userTable.tbl");
				 response.setContentType("text/xml");
				 out.println("<reponse>");
				 out.println("<modified_user login='"+login+"'/>");
				 out.println("</reponse>"); 
			 }
			 else if (order.equals("delete_drone")){
				 String name = request.getParameter("name");
				 try{
					 DocumentBuilderFactory fabrique = DocumentBuilderFactory.newInstance();
					 DocumentBuilder constructeur = fabrique.newDocumentBuilder();
					 // read an xml file and build DOM structure
					 File xml = new File(default_folder + "/conf/"+"immat.xml");
					 Document document = constructeur.parse(xml);
					 NodeList dronenameList = document.getElementsByTagName("drone");
					 Element root= (Element) document.getElementsByTagName("immat").item(0);
					 int i = 0;
					 boolean doIt = true;
					 // removing node
					 while(i<dronenameList.getLength()&&doIt){
							Element e = (Element)dronenameList.item(i);
							if(e.getAttribute("name").equals(name)){
								doIt=false;
								root.removeChild(e);
							}
							i++;
						}
					 if (!doIt){ 	
						 	writeXmlFile(document, default_folder + "/conf/"+"immat.xml");		    
					    	}
					 response.setContentType("text/xml");
					 out.println("<reponse>");
					 
					 if(!doIt)  out.println("<removed_drone status='OK' name='"+name+"'/>");
					 else 		out.println("<removed_drone status='NOTOK' name='"+name+"'/>");
				 
					 out.println("</reponse>");
				 }catch(ParserConfigurationException pce){
						System.out.println("Erreur de configuration du parseur DOM");
						System.out.println("lors de l'appel à fabrique.newDocumentBuilder();");
					}catch(SAXException se){
						System.out.println("Erreur lors du parsing du document");
						System.out.println("lors de l'appel à construteur.parse(xml)");
					}catch(IOException ioe){
						System.out.println("Erreur d'entrée/sortie");
						System.out.println("lors de l'appel à construteur.parse(xml)");
					}				 				  
			 }
			 else if (order.equals("add_drone")){
				 String name = request.getParameter("name");
				 try{
					 DocumentBuilderFactory fabrique = DocumentBuilderFactory.newInstance();
					 DocumentBuilder constructeur = fabrique.newDocumentBuilder();
					 // read an xml file and build DOM structure
					 File xml = new File(default_folder + "/conf/"+"immat.xml");
					 Document document = constructeur.parse(xml);					 
					 Element root= (Element) document.getElementsByTagName("immat").item(0);
					 // inserting node
					 Element dronename = document.createElement("drone");
					 dronename.setAttribute("name",name);
					 root.appendChild(dronename);
					 // save
					 writeXmlFile(document, default_folder + "/conf/"+"immat.xml");
				     response.setContentType("text/xml");
					 out.println("<reponse>");
					 out.println("<added_drone status='OK' name='"+name+"'/>");					 
					 out.println("</reponse>");
				 } catch(ParserConfigurationException pce){
						System.out.println("Erreur de configuration du parseur DOM");
						System.out.println("lors de l'appel à fabrique.newDocumentBuilder();");
					}catch(SAXException se){
						System.out.println("Erreur lors du parsing du document");
						System.out.println("lors de l'appel à construteur.parse(xml)");
					}catch(IOException ioe){
						System.out.println("Erreur d'entrée/sortie");
						System.out.println("lors de l'appel à construteur.parse(xml)");
					}
				 
			 }
			 else if (order.equals("delete_ivyusr")){
				String name = request.getParameter("login");
				boolean res ;
				UserTab logTab = UserTab.unserialize(default_folder + "/conf/"+"userIvyTable.tbl");
				logTab.remove(name);
				res = !logTab.isInside(name);
				logTab.serialize(default_folder + "/conf/"+"userIvyTable.tbl");					
				
				response.setContentType("text/xml");
				out.println("<reponse>");
				 if(res){
					 out.println("<removed_ivyusr login='"+name+"' status='OK'/>");
				 }
				 else{
					 out.println("<removed_ivyusr login='"+name+"' status='NOTOK'/>");
				 }
				 out.println("</reponse>");	
				 
			 }
			 else if (order.equals("add_ivyusr")){
				 String login = request.getParameter("login");
				 String pwd    = request.getParameter("pwd");
				 UserTab logTab = UserTab.unserialize(default_folder + "/conf/"+"userIvyTable.tbl");
				 User n = new User(login,pwd,Rights.IVY);
				 try{
					 logTab.insert(n);
					 response.setContentType("text/xml");
					 out.println("<reponse>");
					 out.println("<added_ivyusr login='"+login+"' status='OK'/>");
					 out.println("</reponse>");
				} catch(AlreadyRegisteredUserException e){
					 response.setContentType("text/xml");
					 out.println("<reponse>");
					 out.println("<added_ivyusr login='"+login+"' status='NOTOK'/>");
					 out.println("</reponse>");
				}
				logTab.serialize(default_folder + "/conf/"+"userIvyTable.tbl");	
			 }
		 }
		 out.close();
	}
	/**
	 * translate the right from string to a class
	 */
	private Rights getRight(String r)
	{
		if (r.equals("admin")) {return Rights.ADMIN;}
		else if (r.equals("user")) {return Rights.USER;}
		else {return Rights.VISITOR;}
	}
	/**
	 * translate the right from a class to a string
	 */
	private String getRight(Rights r)
	{
		if (r== Rights.ADMIN) {return "admin";}
		else if (r== Rights.USER) {return "user";}
		else {return "visitor";}
	}
	/**
	 * delete an user from the file which stores the users
	 */
	private boolean doDeleteUser(String login){
		boolean res ;
		UserTab logTab = UserTab.unserialize(default_folder + "/conf/"+"userTable.tbl");
		logTab.remove(login);
		res = !logTab.isInside(login);
		logTab.serialize(default_folder + "/conf/"+"userTable.tbl");
		return res;
	}
	/**
	 * create an user in the file which stores the users
	 */
	private boolean doCreateUser(String login, String pwd,String right,Set<String> set_of_drone){
		boolean res;
		UserTab logTab = UserTab.unserialize(default_folder + "/conf/"+"userTable.tbl");
		Rights rght = this.getRight(right);
		User usr = new User(login,pwd,rght);
        if (rght==Rights.USER){
        	Iterator<String> itr = set_of_drone.iterator();
        	while (itr.hasNext()){
        		usr.addDrone(itr.next());
        	}
        }
        try{
        	logTab.insert(usr);
        	res = true;
        	logTab.serialize(default_folder + "/conf/"+"userTable.tbl");
        }
        catch (AlreadyRegisteredUserException ex){ // existe deja
        	res = false;
        }
        return res;
		
	}
	/**
	 * send an order to modif a setting of a drone which belongs to a specific ivy bus
	 */
	private void  doModifSetting(String aircraft_web_id,String setting_id,String newsetting_value){
		String order = "DL_SETTING "+ aircraft_web_id + " " + setting_id + " " + newsetting_value;
		sendOrder2Ivy(Integer.parseInt(aircraft_web_id),order);
	}
	/**
	 * send an order to jump to a new block for a drone which belongs to a specific ivy bus
	 */
	private void doActivateBlock(String block_id,String aircraft_web_id){
		// send msg via  serveur.java  
		String order = "JUMP_TO_BLOCK " + aircraft_web_id + " " + block_id ;
		sendOrder2Ivy(Integer.parseInt(aircraft_web_id),order);
	}
	/**
	 * send an order to move to a new waypoint for a drone which belongs to a specific ivy bus
	 */
	private void doFplUpdate(String wpt_name,String aircraft_web_id, String wpt_id,
			String new_alt, String new_lat, String new_lon,String new_alt_for_fpl) {
		// send msg via  serveur.java  
		String order = "MOVE_WAYPOINT " + aircraft_web_id + " " + wpt_id + " " + new_lat + " " + new_lon + " " + new_alt ;
		sendOrder2Ivy(Integer.parseInt(aircraft_web_id),order);
	}// end method
	
	
	/**This method writes a DOM document to a file 
	 * 
	 * @param doc the xml Document
	 * @param filename the name of the file to write in
	 */
	private static void writeXmlFile(Document doc, String filename)
	{ 
		try { 
			// Prepare the DOM document for writing 
			Source source = new DOMSource(doc); 
			// Prepare the output file 
			File file = new File(filename); 
			Result result = new StreamResult(file); 
			// Write the DOM document to the file 
			Transformer xformer = TransformerFactory.newInstance().newTransformer(); 
			xformer.transform(source, result); } 
		catch (TransformerConfigurationException e) { } 
		catch (TransformerException e) { } 
	} 
	
	
	/**
	 * send an order to a specific ivybus via udp
	 * @param ac_web_id the id of the drone on the web
	 * @param order the message to send
	 */
	private void sendOrder2Ivy(int ac_web_id,String order)
	{   
		DatagramSocket socket=null;
		try {
			System.out.println("attempt to send order to ivy");
			socket = new DatagramSocket();
			socket.setSoTimeout(myConf.getSocketTimeout());
			ServletContext srvCtxt = getServletContext();
			// retrieve in the servlet context (shared memory) the table where the ivy session are stored
			HashMap<InetAddress,SessionIvy> tableIvySession =  (HashMap<InetAddress,SessionIvy>) srvCtxt.getAttribute("ivySessionTable");
			if ((socket!=null)&&(tableIvySession!=null)){
				 byte buffer[] = new byte[myConf.getUdpSize()];
				 Iterator<Map.Entry<InetAddress,SessionIvy>> itr = tableIvySession.entrySet().iterator();
				 BusIvy_ checked_bus=null;
				 boolean doIt=true;
				 // search for the session which owns the drone
				 while(itr.hasNext()&&doIt){
						checked_bus  = itr.next().getValue().getBusIvy();
						if(checked_bus.isOwnBy(ac_web_id)){
							doIt = false ;
						}
				 }
				 if(!doIt){
					 	InetAddress address = checked_bus.getAddress();
						SessionIvy session = tableIvySession.get(address);
						AES aesCipher = session.getCipher();
						try{
							buffer = aesCipher.encrypt(order.getBytes());
							DatagramPacket DataEmitted = new DatagramPacket(buffer, buffer.length, address, myConf.portWebToIvy());
							try {
								socket.send(DataEmitted);	
								System.out.println("\tmsg sended to ivy "+ address.toString()+":"+myConf.portWebToIvy());
								System.out.println("\t"+order);
								//System.out.print("*");
							} catch (IOException e) {
								System.out.println("\terreur msg not sended to " +address.toString()+":"+myConf.portWebToIvy());
								e.printStackTrace();
							} 
						} catch (IOException e) {
							System.out.println("\terreur in encryption, msg not sended to " +address.toString()+":"+myConf.portWebToIvy());
							e.printStackTrace();
						} 
						catch(BadPaddingException ex){
							ex.printStackTrace();
							System.out.println("\torder := "+(new String(Hex.encodeHex(order.getBytes()))));
						}
						catch(IllegalBlockSizeException ex){
							ex.printStackTrace();
							System.out.println("\torder := "+(new String(Hex.encodeHex(order.getBytes()))));
						} 
				 }
			}
			else
			{
				// error
				System.out.println("\tunable to know socket udp or session ");
			}
			socket.close();
		} catch (SocketException e1) {
			System.out.println("\tunable to open socket webtoIvy for an user request");
			e1.printStackTrace();
		}
		finally{if(socket!=null){socket.close();socket=null;}}
	}

}
