// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.test;

import nl.justobjects.pushlet.client.PushletClient;
import nl.justobjects.pushlet.client.PushletClientListener;
import nl.justobjects.pushlet.core.Event;
import nl.justobjects.pushlet.core.Protocol;
import nl.justobjects.pushlet.util.PushletException;
import nl.justobjects.pushlet.util.Rand;

import java.util.HashMap;
import java.util.Map;

/**
 * Tester to demonstrate Pushlet use in Java applications.
 * <p/>
 * This program does two things:
 * (1) it subscribes to the subject "test/ping"
 * (2) it publishes an Event with subject "/test/ping" every few seconds.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: StressTester.java,v 1.2 2007/11/09 13:16:57 justb Exp $
 */
public class StressTester implements Protocol {
	static private String host = "localhost";
	static private int port = 8080;
	static private int TESTER_COUNT = 10;
	private static final String SUBJECT = "/test/ping";
	private static final long MIN_PUBLISH_INTERVAL_MILLIS = 200;
	private static final long MAX_PUBLISH_INTERVAL_MILLIS = 1000;
	private static final long MIN_SUBSCRIBER_INTERVAL_MILLIS = 500;
	private static final long MAX_SUBSCRIBER_INTERVAL_MILLIS = 1000;

	public StressTester() {
	}

	public void run() {
		new EventPublisher().start();
		new EventSubscriber().start();
	}

	/**
	 * Generic print.
	 */
	public void err(String s) {
		System.out.println("[StressTester] ERROR" + s);
	}

	/**
	 * Generic print.
	 */
	public void p(String s) {
		System.out.println("[StressTester] " + s);
	}

	private class EventSubscriber extends Thread implements PushletClientListener {
		private PushletClient pushletClient;

		public void run() {
			while (true) {
				// Create and start a Pushlet client; we receive callbacks
				// through onHeartbeat() and onData().
				try {
					pushletClient = new PushletClient(host, port);
					// pushletClient.setDebug(true);
					pushletClient.join();
					pushletClient.listen(this, Protocol.MODE_STREAM);
					//p("listening");
					// Test subscribe/unsubscribe
					String subscriptionId = pushletClient.subscribe(SUBJECT);
					pushletClient.unsubscribe(subscriptionId);

					// The real subscribe
					subscriptionId = pushletClient.subscribe(SUBJECT);
					//p("sleeping");
					sleepRandom();
					//p("leaving");
					pushletClient.unsubscribe(subscriptionId);
					pushletClient.leave();

				} catch (Throwable t) {
					err("Error in EventSubscriber t=" + t);
					return;
				}
			}
		}

		/**
		 * Error occurred.
		 */
		public void onError(String message) {
			// p(message);
		}

		/**
		 * Abort event from server.
		 */
		public void onAbort(Event theEvent) {
			//p("onAbort received: " + theEvent);
		}

		/**
		 * Data event from server.
		 */
		public void onData(Event theEvent) {
			// Calculate round trip delay
			long then = Long.parseLong(theEvent.getField("time"));
			long delay = System.currentTimeMillis() - then;
			//p("onData: ping #" + theEvent.getField("seqNr") + " in " + delay + " ms");
		}

		/**
		 * Heartbeat event from server.
		 */
		public void onHeartbeat(Event theEvent) {
			//p("onHeartbeat received: " + theEvent);
		}

		private void sleepRandom() throws InterruptedException {
			Thread.sleep(Rand.randomLong(MIN_SUBSCRIBER_INTERVAL_MILLIS, MAX_SUBSCRIBER_INTERVAL_MILLIS));
		}
	}

	private class EventPublisher extends Thread {
		private PushletClient pushletClient;

		public void run() {
			// Create and start a Pushlet client; we receive callbacks
			// through onHeartbeat() and onData().
			try {
				pushletClient = new PushletClient(host, port);
				pushletClient.join();

				// p("pushletClient started");
			} catch (PushletException pe) {
				err("Error in EventPublisher pe=" + pe);
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

					Thread.sleep(Rand.randomLong(MIN_PUBLISH_INTERVAL_MILLIS, MAX_PUBLISH_INTERVAL_MILLIS));
				} catch (Exception e) {
					p("EventPublisher exception: " + e);
					return;
				}
			}
		}

	}

	/**
	 * Main program.
	 */
	public static void main(String args[]) {
		if (args.length > 0) {
			TESTER_COUNT = Integer.parseInt(args[0]);
		}
		if (args.length == 3) {
			host = args[1];
			port = Integer.parseInt(args[2]);
		}

		for (int i = 0; i < TESTER_COUNT; i++) {
			new StressTester().run();
		}

	}
}

/*
 * $Log: StressTester.java,v $
 * Revision 1.2  2007/11/09 13:16:57  justb
 * use Rand from util package (and and Rand.java to pushlet client jar
 *
 * Revision 1.1  2005/02/28 17:16:58  justb
 * simple stress tester
 *
 *
 */
