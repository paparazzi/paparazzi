package pow;


public class IvyMsg {

	private int webId;
	private long num_msg;
	private String timeMsg;//"d:M:y:HH:mm:ss" add 2000 to d
	private String ivyMsg;
	private dbOrder order;
	
	public IvyMsg(int wId,long num,String date,String msg,dbOrder order){
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
	
	public int getWebId(){return webId;}
	public  long getNumMsg() {return num_msg;}
	public String getTimeMsg() {return timeMsg;}
	public String getIvyMsg() {return ivyMsg;}
	public dbOrder getOrder() {return order;}
	public void setOrder(dbOrder o ) { order=o;}
	
	public String getSQLQUERY(){
		return "";
	}
	
}
