package pow.webserver;
import java.io.*;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.SAXException;
/**
 * 
 * store useful data about the server configuration
 * and database connection by 
 * reading a specific file place in the 'conf' folder of
 * the web application
 */
public class Conf {
	/** The mail of the administrator to contact if any problems occurs
	 * (appears in the help.jsp page. */
	private  String mailAdmin;
	/** The administrator login (not used). */
	private  String adminLogin;
	/** The port in which the module listen to get udp trames from ivy buses. */
	private int portIvyToWeb;
	/** The dimension of the datagrams received. */
	private int tailleUdpTrame;
	/** The passWord of the database */
	private  String dbPassword;
	/** The userName of the dataBase */
	private  String dbUserName;
	/** The Name of the dataBase */
	private  String dataBaseName;
	/** The timeout of the connection */
	private  int udpTimeout;
	/** The port in which messages from web are sent to ivy buses */
	private int portWebToIvy ;
	/**  period of time in millisec to  reset the filter of waypoint_moved messages */
	private long time2resetFilter;
	/** period of time in millisec to send all the parameter values for a specific drone */
	private long timeToSendValues;
	/** time to wait for the acknowledgement of an order */
	private int order_response_timeout;
	/** time to wait after a die event to remove the drone */
	private int dieEventTimeoutTime;
	/** time to wait after receiving no data for a drone to remove the drone */
	private int dataEventTimeoutTime;
	/** period of time to check for dead ivy buses in serveur.java*/
	private long time2checkDeadBuses;
	/** how the database work VERBOSE or SILENT*/
	private DbMode dbMode;
	/**
	 * 
	 * @param default_folder folder of the web application on the server
	 * @param conf_filename name of the configuration file placed in 'conf' folder of the web application
	 */
	public Conf(String default_folder,String conf_filename)
	{
	try{
		 DocumentBuilderFactory fabrique = DocumentBuilderFactory.newInstance();
		 // création d'un constructeur de documents
		 DocumentBuilder constructeur = fabrique.newDocumentBuilder();
		 // lecture du contenu d'un fichier XML avec DOM
		 File xml = new File(default_folder + "/conf/"+conf_filename);
		 Document document = constructeur.parse(xml);
		 Element mailAdmin_node = (Element)document.getElementsByTagName("mailWebAdmin").item(0);
		 Element adminLogin_node = (Element)document.getElementsByTagName("webAdminLogin").item(0);
		 Element portIvyToWeb_node = (Element)document.getElementsByTagName("portIvyToWeb").item(0);
		 Element taille_node = (Element)document.getElementsByTagName("sizeUdpTrame").item(0);
		 Element passWord_node = (Element)document.getElementsByTagName("dbPassword").item(0);
		 Element userName_node = (Element)document.getElementsByTagName("dbUserName").item(0);
		 Element dataBaseName_node = (Element)document.getElementsByTagName("dataBaseName").item(0);
		 Element timeout_node = (Element)document.getElementsByTagName("udpSocketTimeout").item(0);
		 Element portWebToIvy_node  = (Element)document.getElementsByTagName("portWebToIvy").item(0);		 
		 Element time2resetFilter_node  = (Element)document.getElementsByTagName("time2resetFilter").item(0);
		 Element timeToSendValuesy_node  = (Element)document.getElementsByTagName("timeToSendValues").item(0);
		 Element order_response_timeout_node  = (Element)document.getElementsByTagName("order_response_timeout").item(0);
		 Element dieEventTimeoutTime_node  = (Element)document.getElementsByTagName("dieEventTimeoutTime").item(0);
		 Element dataEventTimeoutTime_node  = (Element)document.getElementsByTagName("dataEventTimeoutTime").item(0);
		 Element time2checkDeadBuses_node  = (Element)document.getElementsByTagName("time2checkDeadBuses").item(0);
		 
		 Element dbMode_node  = (Element)document.getElementsByTagName("dbMode").item(0);
		 String dbMode_str = dbMode_node.getTextContent();
		 if (dbMode_str.equals("verbose")){dbMode = DbMode.VERBOSE;}
		 else {dbMode = DbMode.SILENT;}
		 mailAdmin=mailAdmin_node.getTextContent();
		 adminLogin=adminLogin_node.getTextContent();
		 portIvyToWeb=Integer.parseInt(portIvyToWeb_node.getTextContent());
		 tailleUdpTrame=Integer.parseInt(taille_node.getTextContent());
		 dbPassword=passWord_node.getTextContent();
		 dbUserName=userName_node.getTextContent();
		 dataBaseName=dataBaseName_node.getTextContent();
		 udpTimeout=Integer.parseInt(timeout_node.getTextContent());
		 portWebToIvy=Integer.parseInt(portWebToIvy_node.getTextContent());	
		 time2resetFilter=Long.parseLong(time2resetFilter_node.getTextContent());
		 timeToSendValues=Long.parseLong(timeToSendValuesy_node.getTextContent());
		 order_response_timeout=Integer.parseInt(order_response_timeout_node.getTextContent());
		 dieEventTimeoutTime=Integer.parseInt(dieEventTimeoutTime_node.getTextContent());
		 dataEventTimeoutTime=Integer.parseInt(dataEventTimeoutTime_node.getTextContent());
		 time2checkDeadBuses =Integer.parseInt(time2checkDeadBuses_node.getTextContent());
	} catch(ParserConfigurationException pce){
		System.out.println("error in loading configuration file : "+default_folder + "/conf/"+conf_filename);
		pce.printStackTrace();
	}catch(SAXException se){
		System.out.println("error in loading configuration file : "+default_folder + "/conf/"+conf_filename);
		se.printStackTrace();
	}catch(IOException ioe){
		System.out.println("error in loading configuration file : "+default_folder + "/conf/"+conf_filename);
		ioe.printStackTrace();
	}
	}
	/** @return the udp port on which the server receive data from ivy buses */
	public int portIvyToWeb()
	{
		return portIvyToWeb;
	}
	/** @return the udp port on which the server send data to ivy buses */
	public int portWebToIvy()
	{
		return portWebToIvy;
	}
	/** @return the max size of an udp trame */
	public int getUdpSize()
	{
		return tailleUdpTrame;
	}
	/** @return the password to connect to the database*/
	public String getDBPassword()
	{
		return dbPassword;
	}
	/** @return the login to connect to the database*/ 
	public String getDBUserName()
	{
		return dbUserName;
	}
	/** @return the database name*/
	public String getDataBaseName()
	{
		return dataBaseName;
	}
	/** @return a time after what an exception is thrown if no message has been received by server*/
	public int getSocketTimeout()
	{
		return udpTimeout;
	}
	/** @return the mail to contact if any problem occurs */
	public String mailAdmin(){return mailAdmin;}
	/** @return the login of the administrator of the site */
	public String adminLogin(){return adminLogin;}
	/**@return  period of time in millisec to  reset the filter of waypoint_moved messages */
	public long getTime2resetFilter(){return time2resetFilter;}
	/**@return period of time in millisec to send all the parameter values for a specific drone */
	public long getTimeToSendValues(){return timeToSendValues;}
	/**@return time to wait for the acknowledgement of an order */
	public int getOrderResponseTimeout(){return order_response_timeout;}
	/**@return time to wait after a die event to remove the drone */
	public int getDieEventTimeoutTime(){return dieEventTimeoutTime;}
	/**@return time to wait after receiving no data for a drone to remove the drone */
	public int getDataEventTimeoutTime(){return dataEventTimeoutTime;}
	/**@return period of time to check for dead ivy buses in serveur.java*/
	public long getTime2checkDeadBuses(){return time2checkDeadBuses;}
	/**@return SILENT or VERBOSE */
	public DbMode getDbMode(){return dbMode;}
}