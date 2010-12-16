package pow;
/**
 * implements the life cycle of a drone when it is detected on the ivy bus
 * @author genin
 *
 */
public enum AcStatus {
	UNKNOWN,
	ASKING_IVY_CONF,
	IVY_CONF_RECEIVED,
	ASKING_WEB_ID,
	WEB_ID_RECEIVED,
	UPLOADING_CONF,
	CONF_OK,
	ALIVE,
	CONF_NOTOK
}
