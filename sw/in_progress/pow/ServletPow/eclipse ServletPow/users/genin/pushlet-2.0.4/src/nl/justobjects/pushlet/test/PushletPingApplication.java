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
 * Tester to demonstrate Pushlet use in Java applications.
 *
 * This program does two things:
 * (1) it subscribes to the subject "test/ping"
 * (2) it publishes an Event with subject "/test/ping" every few seconds.
 *
 * @version $Id: PushletPingApplication.java,v 1.15 2005/02/21 16:59:17 justb Exp $
 * @author Just van den Broecke - Just Objects &copy;
 **/
public class PushletPingApplication extends Thread implements PushletClientListener, Protocol {
	private PushletClient pushletClient;
	private String host;
	private int port;
	private static final String SUBJECT = "/test/ping";
	private static final long PUBLISH_INTERVAL_MILLIS = 3000;

	public PushletPingApplication(String aHost, int aPort) {
		host = aHost;
		port = aPort;
	}

	public void run() {
		// Create and start a Pushlet client; we receive callbacks
		// through onHeartbeat() and onData().
		try {
			pushletClient = new PushletClient(host, port);
			pushletClient.setDebug(true);
			pushletClient.join();
			pushletClient.listen(this, Protocol.MODE_STREAM);

			// Test subscribe/unsubscribe
			String subscriptionId = pushletClient.subscribe(SUBJECT);
			pushletClient.unsubscribe(subscriptionId);

			// The real subscribe
			pushletClient.subscribe(SUBJECT);
			p("pushletClient started");
		} catch (PushletException pe) {
			p("Error in setting up pushlet session pe=" + pe);
			return;
		}

		// Publish an event to the server every N seconds.
		Map eventData = new HashMap(2);
		int seqNr = 1;
		while (true) {
			try {
				// Create event data
				eventData.put("seqNr", "" + seqNr++);
				eventData.put("time", "" + System.currentTimeMillis());

				// POST event to pushlet server
				pushletClient.publish(SUBJECT, eventData);

				p("published ping # " + (seqNr - 1) + " - sleeping...");
				Thread.sleep(PUBLISH_INTERVAL_MILLIS);
			} catch (Exception e) {
				p("Postlet exception: " + e);
				System.exit(-1);
			}
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
		long then = Long.parseLong(theEvent.getField("time"));
		long delay = System.currentTimeMillis() - then;
		p("onData: ping #" + theEvent.getField("seqNr") + " in " + delay + " ms");
	}

	/** Heartbeat event from server. */
	public void onHeartbeat(Event theEvent) {
		p("onHeartbeat received: " + theEvent);
	}

	/** Generic print. */
	public void p(String s) {
		System.out.println("[PushletPing] " + s);
	}

	/** Main program. */
	public static void main(String args[]) {
		for (int i = 0; i < 1; i++) {
			if (args.length == 0) {
				new PushletPingApplication("localhost", 8080).start();
			} else {
				// Supply a host and port
				new PushletPingApplication(args[0], Integer.parseInt(args[1])).start();
			}
		}

	}
}


/*
 * $Log: PushletPingApplication.java,v $
 * Revision 1.15  2005/02/21 16:59:17  justb
 * SessionManager and session lease introduced
 *
 * Revision 1.14  2005/02/21 11:50:48  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *
 * Revision 1.13  2005/02/20 13:05:33  justb
 * removed the Postlet (integrated in Pushlet protocol)
 *
 * Revision 1.12  2005/02/18 09:54:15  justb
 * refactor: rename Publisher Dispatcher and single Subscriber class
 *
 * Revision 1.11  2005/02/15 15:46:37  justb
 * client API improves
 *
 * Revision 1.10  2005/02/15 13:28:08  justb
 * use new protocol lib and publish with PushletClient
 *
 * Revision 1.9  2004/10/24 13:52:52  justb
 * small fixes in client lib
 *
 * Revision 1.8  2004/10/24 12:58:19  justb
 * revised client and test classes for new protocol
 *
 * Revision 1.7  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.6  2004/08/12 13:18:54  justb
 * cosmetic changes
 *
 * Revision 1.5  2004/03/10 14:53:10  justb
 * new
 *
 * Revision 1.4  2003/08/17 20:30:20  justb
 * cosmetic changes
 *
 * Revision 1.3  2003/08/15 08:37:41  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.2  2003/05/18 16:15:08  justb
 * support for XML encoded Events
 *
 * Revision 1.1.1.1  2002/09/24 21:02:33  justb
 * import to sourceforge
 *
 * Revision 1.1.1.1  2002/09/20 22:48:19  justb
 * import to SF
 *
 * Revision 1.1.1.1  2002/09/20 14:19:02  justb
 * first import into SF
 *
 * Revision 1.3  2000/08/31 12:49:50  just
 * added CVS comment tags for log and copyright
 *
 *
 */
