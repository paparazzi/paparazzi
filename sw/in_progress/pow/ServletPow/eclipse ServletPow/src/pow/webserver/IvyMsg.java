package pow.webserver;

/**
 * represents information which are sended by the source event to the database on the queue fifo
 * @author genin
 *
 */
public class IvyMsg {

	private int webId;
	private long num_msg;
	private String timeMsg;//"d:M:y:HH:mm:ss" add 2000 to d
	private String ivyMsg;
	private DbOrder order;
	
	public IvyMsg(int wId,long num,String date,String msg,DbOrder order){
		webId = wId;
		num_msg = num;
		timeMsg = date;
		ivyMsg = msg; 		
		this.order = order;
	}
	
	public IvyMsg(int wId,long num,String date,String msg){
		webId = wId;
		num_msg = num;
		timeMsg = date;
		ivyMsg = msg; 		
		this.order = null;
	}
	
	public IvyMsg(int wId){
		webId = wId;
		num_msg = -1;
		timeMsg = "-1";
		ivyMsg = "deconnect all drone"; 		
		this.order = DbOrder.DECONNECT;
	}
	/** @return the ivy web ivy which has sent the message */
	public int getWebId(){return webId;}
	/** @return the number of the message */
	public  long getNumMsg() {return num_msg;}
	/** @return the time at which the message was sent*/
	public String getTimeMsg() {return timeMsg;}
	/** @return the ivy message */
	public String getIvyMsg() {return ivyMsg;}
	/** @return an order on how to store the ivy message in database */
	public DbOrder getOrder() {return order;}
	/**
	 * specify how to store the message in database
	 * @see DbOrder
	 * @param o the way to store the message in database
	 */
	public void setOrder(DbOrder o ) { order=o;}
	
	
}
