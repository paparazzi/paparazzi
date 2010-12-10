package pow;
import java.io.*;
import java.net.*;
import java.util.*;
import java.util.Calendar;
/**
 * 
 * inner representation of an ivy bus 
 * store the drones' information about the drones which belong to this ivy bus
 * store the IP adresse of the machine where the ivy bus is working
 */
public class BusIvy_
{
	private Calendar oCalendar;
	private long oTime;
	private InetAddress busAddress;
	private ArrayList<Integer>  DronesId = new ArrayList<Integer>();
	
	public void setAddress(InetAddress myInetAddress)
	{
		busAddress = myInetAddress;
	}
	public void updateTime()
	{
		oCalendar=Calendar.getInstance();
		oTime = oCalendar.getTimeInMillis();
	}
	public void addDrones(int newDroneId)
	{
		DronesId.add(newDroneId);
	}
	public ArrayList<Integer> getDrones()
	{
		return DronesId;
	}
	public void displayDrones()
	{
		System.out.println("Bus Ivy : ");
		System.out.println(busAddress);
		System.out.println("Drones : ");
		for(Integer myDrone : DronesId)
		{
			System.out.println(DronesId);
		}
	}
	public InetAddress getAddress()
	{
		return busAddress;
	}
	public boolean isOwnBy(int myDroneId)
	{
		return DronesId.contains(myDroneId);
	}
	public boolean isAlive()
	{
		Calendar iCalendar = Calendar.getInstance();
		long iTime = iCalendar.getTimeInMillis();
		
		return ((iTime - oTime)<10000);		// 10 secondes
	}
}