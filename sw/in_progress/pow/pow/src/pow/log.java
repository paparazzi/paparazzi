package pow;
import java.io.*;
import java.util.Calendar;
/**
 * Handles the writing of a log in a file 
 * @author genin
 *
 */
public class log {
	
	private File oFile = null;
	private FileWriter oFileWriter = null;
	private BufferedWriter oBufferedWriter = null;
	
	public log(String path)
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
			//System.out.println("log file to be created  ");
			oFile.createNewFile();
			//System.out.println("log file created  ");
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
			System.out.println("log at : " + Time + " -> " + writing);
		}
		catch(IOException ex)
		{
			System.out.println(ex.getMessage());
		}
		
	}
}