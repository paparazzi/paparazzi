package pow;

public class IvyConnectionExeption extends Exception {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1805159521752564076L;
	private String reason;
	
	public IvyConnectionExeption(String res){
		super();
		reason = res;
	}
	public String toString() {
		return "IvyConnectionExeption : "+reason;
	}
}
