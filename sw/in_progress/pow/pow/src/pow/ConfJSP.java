package pow;
/*
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
*/
/**
 * interface which gives access to useful information to a Java Server Page
 */
public class ConfJSP {

	static Conf powConf = new Conf();
	
	public static String getAdminLogin() //throws Exception
	{    
		return powConf.adminLogin();
	}
	
	public static String getAdminMail() //throws Exception
	{    
		return powConf.mailAdmin();
	}
	
}
