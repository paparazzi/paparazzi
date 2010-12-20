package pow;
/**
 * represents 3 kind of message stored in database
 * CONNECT first connection of an ivy bus --> inform db
 * DECONNECT ivy bus deconnected --> inform db 
 * ADD add message to db
 * @author genin
 *
 */
public enum dbOrder {
	CONNECT,
	DECONNECT,
	ADD
}
