package pow.ivyclient;

public class IvyConnectionExeption extends Exception {

	/**
	 * create a specific exception linked with Ivy problem
	 */
	private static final long serialVersionUID = 1805159521752564076L;
	private String reason;
	
	/**
	 * create an ivy exception with a particular message
	 * @param res the message 
	 */
	public IvyConnectionExeption(String res){
		super();
		reason = res;
	}
	/**
	 * display the exception on stdout
	 */
	public String toString() {
		return "IvyConnectionExeption : "+reason;
	}
}
