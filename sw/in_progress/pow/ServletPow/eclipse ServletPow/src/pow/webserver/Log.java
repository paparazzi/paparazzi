package pow.webserver;
import java.io.*;
import java.util.Calendar;
/**
 * Handles the writing of a log in a file 
 * @author from V1
 * @since V1
 */
public class Log {
	
	private File oFile = null;
	private FileWriter oFileWriter = null;
	private BufferedWriter oBufferedWriter = null;
	/**
	 * create a log file into the log floder of the web application
	 * @param path the complete path of the web application
	 */
	public Log(String path)
	{
		Calendar Today = Calendar.getInstance();
		String fileName = String.valueOf(Today.get(Calendar.DAY_OF_MONTH)) +
		String.valueOf(Today.get(Calendar.MONTH)) +
		String.valueOf(Today.get(Calendar.YEAR)) +
		String.valueOf(Today.get(Calendar.HOUR_OF_DAY)) +
		String.valueOf(Today.get(Calendar.MINUTE)) +
		String.valueOf(Today.get(Calendar.SECOND)) +
		".log";

		try
		{
			oFile = new File(path+"/log/" + fileName);
			oFile.createNewFile();
			oFileWriter = new FileWriter(oFile);
			oBufferedWriter = new BufferedWriter(oFileWriter);
			
			oBufferedWriter.write("##########################################"); oBufferedWriter.newLine();
			oBufferedWriter.write("########## PAPARAZZI ON THE WEB ##########"); oBufferedWriter.newLine();
			oBufferedWriter.write("##########################################"); oBufferedWriter.newLine();
			oBufferedWriter.newLine();
			oBufferedWriter.write("Beginning of log : "); oBufferedWriter.newLine();
			oBufferedWriter.newLine();
			oBufferedWriter.flush();
		}
		catch(IOException ex)
		{
			System.out.println("No log file created ");
			ex.printStackTrace();
		}
	}
	/**
	 * write a string in log file and on stdout
	 * @param writing the string to write in the log file
	 */
	public void write(String writing)
	{
		Calendar Now = Calendar.getInstance();
		String Time = String.valueOf(Now.get(Calendar.HOUR_OF_DAY)) + ":" +
		String.valueOf(Now.get(Calendar.MINUTE)) + ":" +
		String.valueOf(Now.get(Calendar.SECOND));
		writing = writing.trim();
		try
		{
			oBufferedWriter.write(Time + " -> " + writing); oBufferedWriter.newLine();
			oBufferedWriter.flush();
			System.out.println("\nlog at : " + Time + " -> " + writing);
		}
		catch(IOException ex)
		{
			System.out.println(ex.getMessage());
		}
		
	}
}