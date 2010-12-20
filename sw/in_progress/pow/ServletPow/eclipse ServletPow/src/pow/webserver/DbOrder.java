package pow.webserver;
/**
 * represents 4 kind of message stored in database
 * CONNECT first connection of an ivy bus --> inform db
 * DECONNECT ivy bus deconnected --> inform db 
 * ADD add message to db
 * @author genin
 */
public enum DbOrder {
	/** store the message and create a record in the 'connexion' table */ 
	CONNECT, 
	/** store the message and complete the end field in the 'connexion' table */
	DECONNECT,
	/** just store the data in the log table*/
	ADD_DATA,
	/** the message is an order coming from a web user */
	ADD_ORDER  
}
