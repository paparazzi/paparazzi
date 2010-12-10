package pow;
import java.io.*;
/**
 * 
 * store usefull datas about the server configuration
 * reading a specific file on the tomcat server 
 */
public class Conf {
		
	
	private  String mailAdmin;
	private  String adminLogin;
	/**
	 * The port in which the module listen.
	 */
	private int port;
	/**
	 * The dimension of the datagrams received.
	 */
	private int taille;
	/**
	 * The passWord of the database
	 */
	private  String passWord;
	/**
	 * The userName of the dataBase
	 */
	private  String userName;
	/**
	 * The Name of the dataBase
	 */
	private  String dataBaseName;
	/**
	 * The timeout of the connection
	 */
	private  int timeout;
	/** 
	 * The repertory of configuration files
	 */
	private String flightPlanRep;
	/** 
	 * The port in which msg from web are sended to ivy
	 */
	private int portWebToIvy ;
	
	public Conf()
	{
	 File oFile = null;
	 FileReader oFileReader = null;
	 BufferedReader oBufferedReader = null;
	 String fileName = "pow.conf";
	 
	 String line;
	
		try
		{
			 
			oFile = new File("./conf/" + fileName);
			//oFile = new File("./" + fileName);
			oFileReader = new FileReader(oFile);
			oBufferedReader = new BufferedReader(oFileReader);	
		
			while((line = oBufferedReader.readLine())!=null)
			{
				if( (!(line.startsWith("#")))&&(line.length()!= 0) )
				{
					if(line.startsWith("<port:"))
						{
						port = Integer.parseInt(line.substring(line.indexOf(":") + 1,line.indexOf(">")));
						}
					if(line.startsWith("<taille:"))
						{
						taille = Integer.parseInt(line.substring(line.indexOf(":") + 1,line.indexOf(">")));
						}
					if(line.startsWith("<passWord:"))
						{
						passWord = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
						}
					if(line.startsWith("<userName:"))
						{
						userName = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
						}
					if(line.startsWith("<dataBaseName:"))
						{
						dataBaseName = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
						}
					if(line.startsWith("<timeout:"))
						{
						timeout = Integer.parseInt(line.substring(line.indexOf(":") + 1,line.indexOf(">")));
						}
					if(line.startsWith("<flightPlanRep:"))
						{
						flightPlanRep = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
						}
					if(line.startsWith("<mailAdmin:"))
					{
						mailAdmin = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
					}
					if(line.startsWith("<adminLogin:"))
					{
						adminLogin = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
					}
					if(line.startsWith("<portWebToIvy:"))
					{
						portWebToIvy = Integer.parseInt(line.substring(line.indexOf(":") + 1,line.indexOf(">")));
					}
				}
			}
		}
		catch(IOException ex)
		{
			System.out.println("No conf file detected");
			ex.printStackTrace();
		}	
		
	}
	
	public Conf(String default_folder,String conf_filename)
	{
	 File oFile = null;
	 FileReader oFileReader = null;
	 BufferedReader oBufferedReader = null;
	 String fileName = conf_filename;
	 
	 String line;
	
		try
		{
			 
			oFile = new File(default_folder+"/conf/" + fileName);
			//oFile = new File("./" + fileName);
			oFileReader = new FileReader(oFile);
			oBufferedReader = new BufferedReader(oFileReader);	
		
			while((line = oBufferedReader.readLine())!=null)
			{
				if( (!(line.startsWith("#")))&&(line.length()!= 0) )
				{
					if(line.startsWith("<port:"))
						{
						port = Integer.parseInt(line.substring(line.indexOf(":") + 1,line.indexOf(">")));
						}
					if(line.startsWith("<taille:"))
						{
						taille = Integer.parseInt(line.substring(line.indexOf(":") + 1,line.indexOf(">")));
						}
					if(line.startsWith("<passWord:"))
						{
						passWord = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
						}
					if(line.startsWith("<userName:"))
						{
						userName = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
						}
					if(line.startsWith("<dataBaseName:"))
						{
						dataBaseName = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
						}
					if(line.startsWith("<timeout:"))
						{
						timeout = Integer.parseInt(line.substring(line.indexOf(":") + 1,line.indexOf(">")));
						}
					if(line.startsWith("<flightPlanRep:"))
						{
						flightPlanRep = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
						}
					if(line.startsWith("<mailAdmin:"))
					{
						mailAdmin = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
					}
					if(line.startsWith("<adminLogin:"))
					{
						adminLogin = line.substring(line.indexOf(":") + 1,line.indexOf(">"));
					}
					if(line.startsWith("<portWebToIvy:"))
					{
						portWebToIvy = Integer.parseInt(line.substring(line.indexOf(":") + 1,line.indexOf(">")));
					}
				}
			}
		}
		catch(IOException ex)
		{
			System.out.println("No conf file detected");
			ex.printStackTrace();
		}		
	}
	
	public int port()
	{
		return port;
	}
	
	public int portWebToIvy()
	{
		return portWebToIvy;
	}
	public int taille()
	{
		return taille;
	}
	public String passWord()
	{
		return passWord;
	}
	public String userName()
	{
		return userName;
	}
	public String dataBaseName()
	{
		return dataBaseName;
	}
	public int timeout()
	{
		return timeout;
	}
	public String flightPlanRep()
	{
		return flightPlanRep;
	}
	
	public String mailAdmin()
	{
		return mailAdmin;
	}
	
	public String adminLogin(){
		return adminLogin;
	}
}