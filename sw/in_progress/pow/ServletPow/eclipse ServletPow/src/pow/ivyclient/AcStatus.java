package pow.ivyclient;
/**
 * implements the life cycle of a drone when it is detected on the ivy bus.
 * these steps are needed to prevent that a ivy message is processed before 
 * that all information about the drone were retrieved
 * these status follow the steps that are followed by the module when a new drone
 * is detected on the bus 
 * @see Ivy2Udp
 * @author genin
 */
public enum AcStatus {
	/** the system does not know this drone, information are requested*/
	UNKNOWN,
	/** a request about the configuration has been sent on ivy*/
	ASKING_IVY_CONF,
	/** the configuration has been received from ivy*/
	IVY_CONF_RECEIVED,
	/** the module sends request to the web server for having a web id for the drone*/
	ASKING_WEB_ID,
	/** the drone web id has been received from the server */
	WEB_ID_RECEIVED,
	/** the module upload the two configuration files of the drone on the server */
	UPLOADING_CONF,
	/** the 2 configuration files have been uploaded successfully */
	CONF_OK,
	/** the module can received ivy message for this drone and send them to the server*/
	ALIVE,
	/** a problem occurred during the process*/
	CONF_NOTOK
}
