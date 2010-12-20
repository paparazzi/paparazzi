package pow.ivyclient;

import java.net.*;
import java.util.*;
import java.util.Calendar;
/**
 * inner representation of an ivy bus 
 * store the drones' information about the drones which belong to this ivy bus
 * store the IP adress of the machine where the ivy bus is working
 */
public class BusIvy_
{
	private Calendar oCalendar;
	private long oTime;
	private InetAddress busAddress;
	private ArrayList<Integer>  DronesId = new ArrayList<Integer>();
	/**
	 * 
	 * @param myInetAddress the ip address of the ivy host
	 */
	public void setAddress(InetAddress myInetAddress)
	{
		busAddress = myInetAddress;
	}
	/**
	 * update the 'inner clock' of the object when a message for this bus is detected
	 * useful to know if the bus is still alive 
	 */
	public void updateTime()
	{
		oCalendar=Calendar.getInstance();
		oTime = oCalendar.getTimeInMillis();
	}
	/**
	 * inform the bus that a new drone send information on this bus
	 * @param newDroneId (the ivy id)
	 */
	public void addDrones(int newDroneId)
	{
		DronesId.add(newDroneId);
	}
	/**
	 * get a array list containing the ivy id of the drones present on the bus 
	 * @return the array list containing the ivy id of the drones present on the bus 
	 */
	public ArrayList<Integer> getDrones()
	{
		return DronesId;
	}
	/**
	 * display information on stdout about the drones present on the bus
	 */
	public void displayDrones()
	{
		System.out.println("Bus Ivy : ");
		System.out.println(busAddress);
		System.out.println("Drones : ");
		for(Integer myDrone : DronesId)
		{
			System.out.println(myDrone);
		}
	}
	/**
	 * provides the IP adress of the host hosting the bus
	 * @return the IP adress of the host hosting the bus
	 */
	public InetAddress getAddress()
	{
		return busAddress;
	}
	/**
	 * inform if a drone is present or not on the bus
	 * @param myDroneId the ivy id
	 * @return true if the drone is present on the bus
	 */
	public boolean isOwnBy(int myDroneId)
	{
		return DronesId.contains(myDroneId);
	}
	/**
	 * inform if the bus is alive
	 * @return true if the last message was received less than 10 seconds ago
	 */
	public boolean isAlive()
	{
		Calendar iCalendar = Calendar.getInstance();
		long iTime = iCalendar.getTimeInMillis();
		
		return ((iTime - oTime)<10000);		// 10 secondes
	}
}