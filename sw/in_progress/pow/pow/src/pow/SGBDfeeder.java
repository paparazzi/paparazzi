package pow;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.LinkedBlockingQueue;

//CREATE TABLE connexion(id INTEGER NOT NULL AUTO_INCREMENT,webId INTEGER,login VARCHAR(50),start DATE NOT NULL,end DATE DEFAULT NULL,PRIMARY KEY(id));


public class SGBDfeeder implements Runnable {

	private String dbName;
	private String dbLogin;
	private String dbPwd;
	private Queue<IvyMsg> fifo;
	
	public SGBDfeeder(String dbname,String dbuser,String dbpwd){
		fifo = new ConcurrentLinkedQueue<IvyMsg>();
		//fifo = new LinkedBlockingQueue<IvyMsg>();
		dbName=dbname;
		dbLogin=dbuser;
		dbPwd=dbpwd;
	}
	
	public Queue<IvyMsg> getQueueFIFO(){
		return fifo;
	}

	
	public void run() {
		Connection connection = null;
		Statement stmt = null;
		ResultSet rs = null;

		String url = "jdbc:mysql://localhost/"+dbName;
		//String login = "root";
		//String password = "pwdroot";
		// connect to SGBD
		try {
			//Connection to the Data Base
			Class.forName("com.mysql.jdbc.Driver").newInstance();
			connection = DriverManager.getConnection(url,dbLogin,dbPwd);
			// receiving msg
			while(true){
				// get msg
				
				// store it in db
				try{
					//connection = DriverManager.getConnection(url,login,password);
					stmt = connection.createStatement();
					int res = stmt.executeUpdate("CREATE DATABASE pow_sql");
					res = stmt.executeUpdate("CREATE TABLE connexion (" +
							"web_id INTEGER," +
							"login VARCHAR(50), " +
							"start DATE NOT NULL, " +
							"end DATE DEFAULT NULL)");
				} catch (SQLException ex) {
					System.out.println("SQLException: " + ex.getMessage());
					System.out.println("SQLState: " + ex.getSQLState());
					System.out.println("VendorError: " + ex.getErrorCode());
				}	 	
			}				
		}
		catch (SQLException ex) {System.out.println("unable to connect to db");}
		catch (ClassNotFoundException ex) {}
		catch (IllegalAccessException ex){}
		catch ( InstantiationException ex){}
		finally
		{
		   if(connection!=null){try{connection.close();}catch(Exception e){e.printStackTrace();}}
		   //etc.
		}
	}
	
	
}
