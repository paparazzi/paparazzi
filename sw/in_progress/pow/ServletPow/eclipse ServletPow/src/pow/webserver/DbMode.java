package pow.webserver;
/** 
 * specify how the database works
 * @author genin
 */
public enum DbMode {
 /** the database records all ivy messages the server receives */
 VERBOSE,
 /** the database stores only connection activity of ivy client */
 SILENT
}
