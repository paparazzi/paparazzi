// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.PushletException;
import nl.justobjects.pushlet.util.Rand;
import nl.justobjects.pushlet.util.Sys;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.net.URLEncoder;

/**
 * Handles data channel between dispatcher and client.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Subscriber.java,v 1.26 2007/11/23 14:33:07 justb Exp $
 */
public class Subscriber implements Protocol, ConfigDefs {
	private Session session;

	/**
	 * Blocking queue.
	 */
	private EventQueue eventQueue = new EventQueue(Config.getIntProperty(QUEUE_SIZE));

	/**
	 * URL to be used in refresh requests in pull/poll modes.
	 */
	private long queueReadTimeoutMillis = Config.getLongProperty(QUEUE_READ_TIMEOUT_MILLIS);
	private long queueWriteTimeoutMillis = Config.getLongProperty(QUEUE_WRITE_TIMEOUT_MILLIS);
	private long refreshTimeoutMillis = Config.getLongProperty(PULL_REFRESH_TIMEOUT_MILLIS);
	volatile long lastAlive = Sys.now();

	/**
	 * Map of active subscriptions, keyed by their subscription id.
	 */
	private Map subscriptions = Collections.synchronizedMap(new HashMap(3));

	/**
	 * Are we able to accept/send events ?.
	 */
	private volatile boolean active;

	/**
	 * Transfer mode (stream, pull, poll).
	 */
	private String mode;


	/**
	 * Protected constructor as we create through factory method.
	 */
	protected Subscriber() {
	}

	/**
	 * Create instance through factory method.
	 *
	 * @param aSession the parent Session
	 * @return a Subscriber object (or derived)
	 * @throws PushletException exception, usually misconfiguration
	 */
	public static Subscriber create(Session aSession) throws PushletException {
		Subscriber subscriber;
		try {
			subscriber = (Subscriber) Config.getClass(SUBSCRIBER_CLASS, "nl.justobjects.pushlet.core.Subscriber").newInstance();
		} catch (Throwable t) {
			throw new PushletException("Cannot instantiate Subscriber from config", t);
		}

		subscriber.session = aSession;
		return subscriber;
	}

	public void start() {
		active = true;
	}

	public void stop() {
		removeSubscriptions();
		active = false;
	}

	public void bailout() {
		session.stop();
	}

	/**
	 * Are we still active to handle events.
	 */
	public boolean isActive() {
		return active;
	}

	/**
	 * Return client session.
	 */
	public Session getSession() {
		return session;
	}

	/**
	 * Get (session) id.
	 */
	public String getId() {
		return session.getId();
	}

	/**
	 * Return subscriptions.
	 */
	public Subscription[] getSubscriptions() {
		// todo: Optimize
		return (Subscription[]) subscriptions.values().toArray(new Subscription[0]);
	}

	/**
	 * Add a subscription.
	 */
	public Subscription addSubscription(String aSubject, String aLabel) throws PushletException {
		Subscription subscription = Subscription.create(aSubject, aLabel);
		subscriptions.put(subscription.getId(), subscription);
		info("Subscription added subject=" + aSubject + " sid=" + subscription.getId() + " label=" + aLabel);
		return subscription;
	}

	/**
	 * Remove a subscription.
	 */
	public Subscription removeSubscription(String aSubscriptionId) {
		Subscription subscription = (Subscription) subscriptions.remove(aSubscriptionId);
		if (subscription == null) {
			warn("No subscription found sid=" + aSubscriptionId);
			return null;
		}
		info("Subscription removed subject=" + subscription.getSubject() + " sid=" + subscription.getId() + " label=" + subscription.getLabel());
		return subscription;
	}

	/**
	 * Remove all subscriptions.
	 */
	public void removeSubscriptions() {
		subscriptions.clear();
	}

	public String getMode() {
		return mode;
	}

	public void setMode(String aMode) {
		mode = aMode;
	}

	public long getRefreshTimeMillis() {
		String minWaitProperty = PULL_REFRESH_WAIT_MIN_MILLIS;
		String maxWaitProperty = PULL_REFRESH_WAIT_MAX_MILLIS;
		if (mode.equals((MODE_POLL))) {
			minWaitProperty = POLL_REFRESH_WAIT_MIN_MILLIS;
			maxWaitProperty = POLL_REFRESH_WAIT_MAX_MILLIS;

		}
		return Rand.randomLong(Config.getLongProperty(minWaitProperty),
				Config.getLongProperty(maxWaitProperty));
	}

	/**
	 * Get events from queue and push to client.
	 */
	public void fetchEvents(Command aCommand) throws PushletException {

		String refreshURL = aCommand.httpReq.getRequestURI() + "?" + P_ID + "=" + session.getId() + "&" + P_EVENT + "=" + E_REFRESH;

		// This is the only thing required to support "poll" mode
		if (mode.equals(MODE_POLL)) {
			queueReadTimeoutMillis = 0;
			refreshTimeoutMillis = Config.getLongProperty(POLL_REFRESH_TIMEOUT_MILLIS);
		}

		// Required for fast bailout (tomcat)
		aCommand.httpRsp.setBufferSize(128);

		// Try to prevent caching in any form.
		aCommand.sendResponseHeaders();

		// Let clientAdapter determine how to send event
		ClientAdapter clientAdapter = aCommand.getClientAdapter();
		Event responseEvent = aCommand.getResponseEvent();
		try {
			clientAdapter.start();

			// Send first event (usually hb-ack or listen-ack)
			clientAdapter.push(responseEvent);

			// In pull/poll mode and when response is listen-ack or join-listen-ack,
			// return and force refresh immediately
			// such that the client recieves response immediately over this channel.
			// This is usually when loading the browser app for the first time
			if ((mode.equals(MODE_POLL) || mode.equals(MODE_PULL))
					&& responseEvent.getEventType().endsWith(Protocol.E_LISTEN_ACK)) {
				sendRefresh(clientAdapter, refreshURL);

				// We should come back later with refresh event...
				return;
			}
		} catch (Throwable t) {
			bailout();
			return;
		}


		Event[] events = null;

		// Main loop: as long as connected, get events and push to client
		long eventSeqNr = 1;
		while (isActive()) {
			// Indicate we are still alive
			lastAlive = Sys.now();

			// Update session time to live
			session.kick();

			// Get next events; blocks until timeout or entire contents
			// of event queue is returned. Note that "poll" mode
			// will return immediately when queue is empty.
			try {
				// Put heartbeat in queue when starting to listen in stream mode
				// This speeds up the return of *_LISTEN_ACK
				if (mode.equals(MODE_STREAM) && eventSeqNr == 1) {
					eventQueue.enQueue(new Event(E_HEARTBEAT));
				}

				events = eventQueue.deQueueAll(queueReadTimeoutMillis);
			} catch (InterruptedException ie) {
				warn("interrupted");
				bailout();
			}

			// Send heartbeat when no events received
			if (events == null) {
				events = new Event[1];
				events[0] = new Event(E_HEARTBEAT);
			}

			// ASSERT: one or more events available

			// Send events to client using adapter
			// debug("received event count=" + events.length);
			for (int i = 0; i < events.length; i++) {
				// Check for abort event
				if (events[i].getEventType().equals(E_ABORT)) {
					warn("Aborting Subscriber");
					bailout();
				}

				// Push next Event to client
				try {
					// Set sequence number
					events[i].setField(P_SEQ, eventSeqNr++);

					// Push to client through client adapter
					clientAdapter.push(events[i]);
				} catch (Throwable t) {
					bailout();
					return;
				}
			}

			// Force client refresh request in pull or poll modes
			if (mode.equals(MODE_PULL) || mode.equals(MODE_POLL)) {
				sendRefresh(clientAdapter, refreshURL);

				// Always leave loop in pull/poll mode
				break;
			}
		}
	}

	/**
	 * Determine if we should receive event.
	 */
	public Subscription match(Event event) {
		Subscription[] subscriptions = getSubscriptions();
		for (int i = 0; i < subscriptions.length; i++) {
			if (subscriptions[i].match(event)) {
				return subscriptions[i];
			}
		}
		return null;
	}

	/**
	 * Event from Dispatcher: enqueue it.
	 */
	public void onEvent(Event theEvent) {
		if (!isActive()) {
			return;
		}

		// p("send: queue event: "+theEvent.getSubject());

		// Check if we had any active continuation for at
		// least 'timeOut' millisecs. If the client has left this
		// instance there would be no way of knowing otherwise.
		long now = Sys.now();
		if (now - lastAlive > refreshTimeoutMillis) {
			warn("not alive for at least: " + refreshTimeoutMillis + "ms, leaving...");
			bailout();
			return;
		}

		// Put event in queue; leave if queue full
		try {
			if (!eventQueue.enQueue(theEvent, queueWriteTimeoutMillis)) {
				warn("queue full, bailing out...");
				bailout();
			}

			// ASSERTION : Event in queue.
			// see fetchEvents() where Events are dequeued and pushed to the client.
		} catch (InterruptedException ie) {
			bailout();
		}

	}

	/**
	 * Send refresh command to pull/poll clients.
	 */
	protected void sendRefresh(ClientAdapter aClientAdapter, String aRefreshURL) {
		Event refreshEvent = new Event(E_REFRESH);

		// Set wait time and url for refresh
		refreshEvent.setField(P_WAIT, "" + getRefreshTimeMillis());
		refreshEvent.setField(P_URL, aRefreshURL);

		try {
			// Push to client through client adapter
			aClientAdapter.push(refreshEvent);

			// Stop this round until refresh event
			aClientAdapter.stop();
		} catch (Throwable t) {
			// Leave on any exception
			bailout();
		}
	}

	/**
	 * Info.
	 */
	protected void info(String s) {
		session.info("[Subscriber] " + s);
	}

	/**
	 * Exceptional print util.
	 */
	protected void warn(String s) {
		session.warn("[Subscriber] " + s);
	}

	/**
	 * Exceptional print util.
	 */
	protected void debug(String s) {
		session.debug("[Subscriber] " + s);
	}


	public String toString() {
		return session.toString();
	}
}

/*
 * $Log: Subscriber.java,v $
 * Revision 1.26  2007/11/23 14:33:07  justb
 * core classes now configurable through factory
 *
 * Revision 1.25  2007/11/10 15:53:15  justb
 * put heartbeat in queue when start fetching events in stream-mode
 *
 * Revision 1.24  2006/10/19 12:33:40  justb
 * add atomic join-listen support (one request)
 *
 * Revision 1.22  2006/05/06 00:06:28  justb
 * first rough version AJAX client
 *
 * Revision 1.21  2005/02/28 12:45:59  justb
 * introduced Command class
 *
 * Revision 1.20  2005/02/21 16:59:09  justb
 * SessionManager and session lease introduced
 *
 * Revision 1.19  2005/02/21 12:32:28  justb
 * fixed publish event in Controller
 *
 * Revision 1.18  2005/02/21 11:50:46  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *
 * Revision 1.17  2005/02/20 13:05:32  justb
 * removed the Postlet (integrated in Pushlet protocol)
 *
 * Revision 1.16  2005/02/18 12:36:47  justb
 * changes for renaming and configurability
 *
 * Revision 1.15  2005/02/18 10:07:23  justb
 * many renamings of classes (make names compact)
 *
 * Revision 1.14  2005/02/18 09:54:15  justb
 * refactor: rename Publisher Dispatcher and single Subscriber class
 *
 * Revision 1.13  2005/02/16 14:39:34  justb
 * fixed leave handling and added "poll" mode
 *
 * Revision 1.12  2005/01/24 13:42:00  justb
 * new protocol changes (p_listen)
 *
 * Revision 1.11  2005/01/13 14:47:15  justb
 * control evt: send response on same (control) connection
 *
 * Revision 1.10  2004/10/24 20:50:35  justb
 * refine subscription with label and sending sid and label on events
 *
 * Revision 1.9  2004/10/24 12:58:18  justb
 * revised client and test classes for new protocol
 *
 * Revision 1.8  2004/09/26 21:39:43  justb
 * allow multiple subscriptions and out-of-band requests
 *
 * Revision 1.7  2004/09/20 22:01:38  justb
 * more changes for new protocol
 *
 * Revision 1.6  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.5  2004/08/13 23:36:05  justb
 * rewrite of Pullet into Pushlet "pull" mode
 *
 * Revision 1.4  2004/03/10 14:01:55  justb
 * formatting and *Subscriber refactoring
 *
 * Revision 1.3  2003/08/15 08:37:40  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.2  2003/05/18 16:15:08  justb
 * support for XML encoded Events
 *
 * Revision 1.1.1.1  2002/09/24 21:02:32  justb
 * import to sourceforge
 *
 * Revision 1.1.1.1  2002/09/20 22:48:18  justb
 * import to SF
 *
 * Revision 1.1.1.1  2002/09/20 14:19:04  justb
 * first import into SF
 *
 * Revision 1.3  2002/04/15 20:42:41  just
 * reformatting and renaming GuardedQueue to EventQueue
 *
 * Revision 1.2  2000/08/21 20:48:29  just
 * added CVS log and id tags plus copyrights
 *
 *
 */
