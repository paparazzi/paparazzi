package pow;

import java.io.*;
import java.net.*;
import fr.dgac.ivy.*;

/**
 * This class aims to give order to the drones according to the datagrams it
 * receive from the web
 * @author jabln
 */
class Ivy2UdpWriting implements IvyMessageListener {

    //static InetAddress serveur;
    // final static int port = 8535;
    //final static int taille = 1024;
    private Ivy bus;
    private acNetIdStorage netIdStorage;
    private String[] message; //Array which contains the informations are to be sent on the bus
    private String IvyAcId;  //Identity of the aircraft on the bus Ivy

    /**
     * This constructor of the class binds to the bus Ivy and implements the
     * method receive to send on the bus the informations given in parametres
     * in an array of string
     * @param message Informations that is to be sent on the bus Ivy
     * @param storage the object which to do the link between webid and ivyid for each drone
     * @throws java.lang.Exception
     */
    Ivy2UdpWriting(String[] message, acNetIdStorage storage) throws IvyException {
    	this.message = message;
        netIdStorage = storage;
        int acwebid = Integer.parseInt(message[1]);
        this.IvyAcId = netIdStorage.getAcIvyId(acwebid);
        if ( this.IvyAcId != null) {
	        bus = new Ivy("Web Writing", "Web Writing ready", null);
	        bus.start(null);
	        bus.bindMsg("FLIGHT_PARA.*", this);
	   }
        else
        {
        	System.out.println("enable to make the link between webid on ivyidy");
        }
    }

    /**
     * This method implements the method receive of the bus Ivy class. It sends
     * the informations on the bus and stop the bind
     * @param client
     * @param args
     */
    public void receive(IvyClient client, String args[]) {
        try {
            String s = "WEB_MSG ";
            s = s + message[0] + " " + IvyAcId + " ";
            for (int i = 2; i < message.length; i++) {
                s = s + message[i] + " ";
            }
            bus.sendMsg(s);
            bus.stop();
        } catch (IvyException ie) {
            System.out.println("can't send my message on the bus");
        }
    }
}
