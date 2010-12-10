// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.test;

import nl.justobjects.pushlet.client.PushletClient;
import nl.justobjects.pushlet.client.PushletClientListener;
import nl.justobjects.pushlet.core.Event;
import nl.justobjects.pushlet.core.Protocol;
import nl.justobjects.pushlet.util.PushletException;

import java.util.HashMap;
import java.util.Map;

/**
 * Demonstrates join-listen service.
 *
 * The pushlet protocol supports the "join-listen" service
 * which allows stateless (e.g. RESTful) clients to join/subscribe/listen
 * using a single HTTP request.
 *
 * @version $Id: SimpleListener.java,v 1.3 2005/03/14 14:07:23 justb Exp $
 * @author Just van den Broecke - Just Objects &copy;
 **/
public class SimpleListener implements PushletClientListener, Protocol {
	private static String SUBJECT = "/temperature";
	private static final String MODE = MODE_STREAM;

	public SimpleListener(String aHost, int aPort) {
		// Create and start a Pushlet client; we receive callbacks
		// through onHeartbeat() and onData().
		try {
			PushletClient pushletClient = new PushletClient(aHost, aPort);
			pushletClient.setDebug(false);
			pushletClient.join();
			pushletClient.listen(this, MODE, SUBJECT);
			p("pushletClient started");
		} catch (PushletException pe) {
			p("Error in setting up pushlet session pe=" + pe);
		}
	}

	/** Error occurred. */
	public void onError(String message) {
		p(message);
	}

	/** Abort event from server. */
	public void onAbort(Event theEvent) {
		p("onAbort received: " + theEvent);
	}

	/** Data event from server. */
	public void onData(Event theEvent) {
		// Calculate round trip delay
		System.out.println(theEvent.toXML());
	}

	/** Heartbeat event from server. */
	public void onHeartbeat(Event theEvent) {
		p("onHeartbeat received: " + theEvent);
	}

	/** Generic print. */
	public void p(String s) {
		System.out.println("[SimpleListener] " + s);
	}

	/** Main program. */
	public static void main(String args[]) {
		if (args.length == 0) {
			new SimpleListener("localhost", 8080);
		}
		else if (args.length == 1) {
			SUBJECT = args[0];
			new SimpleListener("localhost", 8080);
		} else {
			SUBJECT = args[0];
			// args[1] and [2] should be host and port
			new SimpleListener(args[1], Integer.parseInt(args[2]));
		}
	}
}


/*
 * $Log: SimpleListener.java,v $
 * Revision 1.3  2005/03/14 14:07:23  justb
 * addded subject arg
 *
 * Revision 1.2  2005/02/28 21:21:32  justb
 * no chg
 *
 * Revision 1.1  2005/02/28 15:58:05  justb
 * added SimpleListener example
 *
 *
 */
