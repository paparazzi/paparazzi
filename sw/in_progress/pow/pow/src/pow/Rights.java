package pow;
/**
 * describe the different rights a web user can have
 * @author genin
 *
 */
public enum Rights {
	VISITOR, // can only see the drones
	ADMIN, // can pilot all the drones and manage user's account and create others administrators
	USER,
	IVY // specify an ivy user which send data to the server via UDP
}
