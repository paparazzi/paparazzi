package pow.webserver;
/**
 * describe the different rights a web user can have
 * @author genin
 *
 */
public enum Rights {
	/** can only see the drones */
	VISITOR,
	/** can pilot all the drones and manage user's account and create others administrators*/
	ADMIN, 
	/** can pilot only specific drones which are specified in its user account */
	USER, 
	/** specify an ivy user which send data to the server via UDP */
	IVY 
}
