package pow.webserver;

import java.sql.Connection;
import java.sql.DriverManager;
//import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.Timestamp;
import java.util.concurrent.LinkedBlockingQueue;


/**
 * thread which waits for ivy messages sent by event source (serveur.java) 
 * and stores them into a mysql database
 * the ivy messages are extracted from a blocking queue (fifo) 
 * parameter of the database are extracted from conf file
 * @link http://dev.mysql.com/doc/refman/5.0/fr/sql-syntax.html
 */
public class SGBDfeeder implements Runnable {

	private String dbName;
	private String dbLogin;
	private String dbPwd;
	private String url ;
	private LinkedBlockingQueue<IvyMsg> fifo;
	private boolean doIt;
	private DbMode mode;
	/**
	 * create a thread which fill the database with the message it extracts from the fifo 
	 * @param dbname the name of the database
	 * @param dbuser the user login to connect to the database
	 * @param dbpwd the user password to connect to the database
	 */
	public SGBDfeeder(String dbname,String dbuser,String dbpwd,DbMode mode){
		//fifo = new ConcurrentLinkedQueue<IvyMsg>();
		fifo = new LinkedBlockingQueue<IvyMsg>();		
		dbName = dbname;
		dbLogin= dbuser;
		dbPwd  = dbpwd;
		url = "jdbc:mysql://localhost/"+dbName;
		doIt   = true;
		this.mode = mode;
	}
	/**
	 * 
	 * @return the fifo to communicate with the database feeder
	 */
	public LinkedBlockingQueue<IvyMsg> getQueueFIFO(){
		return fifo;
	}
	/**
	 * stop the thread by ending run method
	 * used caused Thread.stop is now deprecated
	 */
	public void kill(){
		doIt= false;
	}
	/**
	 * listens fifo queue and stores ivy messages in database
	 * if the message has the DECONNECT type it also fills the end field corresponding
	 * to this ivy session into the 'connexion' table 
	 */
	public void run() {
			while(doIt){
				try {
					IvyMsg msg = fifo.take(); // fetch an ivy message in the fifo or wait
					// store it in db
					DbOrder order = msg.getOrder();
					switch(order){
						case DECONNECT :{
							storeDeconnectionMsg(msg.getIvyMsg(), msg.getWebId());
							break;
						}
						default : {
							if (mode == DbMode.VERBOSE) // if mode is silent, we don't store messages in db
							{
								storeMsg(msg.getIvyMsg(), msg.getWebId());
							}
							break;
							}
					}	
				} catch (InterruptedException e) {
					System.out.println("database exception : no receiving data");
					//e.printStackTrace();
				}
			}
			System.out.println("fin thread datafeeder ok");
	}
	
	/**
	 * connecting to db and store a no specific message
	 * @param msg
	 * @param webId
	 */
	private void storeMsg(String msg, int webId){
		Connection connection = null;
		Statement stmt = null;
		// ResultSet rs;
		try {
			Class.forName("com.mysql.jdbc.Driver").newInstance();
			connection = DriverManager.getConnection(url,dbLogin,dbPwd);
			try{
			stmt = connection.createStatement();
			stmt.executeUpdate("INSERT INTO log (msg,webId) VALUES ('"+msg+"',"+webId+")");
			}
			catch (SQLException ex) {
				System.out.println("SQLException: " + ex.getMessage());
				System.out.println("SQLState: " + ex.getSQLState());
				System.out.println("VendorError: " + ex.getErrorCode());
			}			
		}
		catch (SQLException ex) {System.out.println("unable to connect to db");}
		catch (ClassNotFoundException ex) {ex.printStackTrace();}
		catch (IllegalAccessException ex){ex.printStackTrace();}
		catch ( InstantiationException ex){ex.printStackTrace();}
		finally
		{
		   if(connection!=null){try{connection.close();}catch(Exception e){e.printStackTrace();}}		   
		}
	}
	/**
	 * deconnection message , fill the end field in connexion table 
	 * and store the  message in log table
	 */
	private void storeDeconnectionMsg(String msg, int webId){
		Connection connection = null;
		Statement stmt = null;
		Timestamp t = new Timestamp(System.currentTimeMillis());
		String monHeure = t.toString();
		try {
			Class.forName("com.mysql.jdbc.Driver").newInstance();
			connection = DriverManager.getConnection(url,dbLogin,dbPwd);
			try{
			stmt = connection.createStatement();
			stmt.executeUpdate("UPDATE connexion SET end='"+monHeure+"' WHERE webId="+webId);
			stmt = connection.createStatement();
			stmt.executeUpdate("INSERT INTO log (msg,webId) VALUES ('"+msg+"',"+webId+")");
			}
			catch (SQLException ex) {
				System.out.println("SQLException: " + ex.getMessage());
				System.out.println("SQLState: " + ex.getSQLState());
				System.out.println("VendorError: " + ex.getErrorCode());
			}			
		}
		catch (SQLException ex) {System.out.println("unable to connect to db");}
		catch (ClassNotFoundException ex) {ex.printStackTrace();}
		catch (IllegalAccessException ex){ex.printStackTrace();}
		catch ( InstantiationException ex){ex.printStackTrace();}
		finally
		{
		   if(connection!=null){try{connection.close();}catch(Exception e){e.printStackTrace();}}		   
		}
	}
}
