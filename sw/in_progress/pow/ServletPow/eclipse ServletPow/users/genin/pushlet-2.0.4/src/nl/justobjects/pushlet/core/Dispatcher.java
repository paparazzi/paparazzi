// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.Log;
import nl.justobjects.pushlet.util.PushletException;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

/**
 * Routes Events to Subscribers.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Dispatcher.java,v 1.9 2007/12/04 13:55:53 justb Exp $
 */
public class Dispatcher implements Protocol, ConfigDefs {
	/**
	 * Singleton pattern:  single instance.
	 */
	private static Dispatcher instance;
	protected SessionManagerVisitor sessionManagerVisitor;

	static {
		try {
			instance = (Dispatcher) Config.getClass(DISPATCHER_CLASS, "nl.justobjects.pushlet.core.Dispatcher").newInstance();
			Log.info("Dispatcher created className=" + instance.getClass());
		} catch (Throwable t) {
			Log.fatal("Cannot instantiate Dispatcher from config", t);
		}
	}

	/**
	 * Singleton pattern with factory method: protected constructor.
	 */
	protected Dispatcher() {

	}

	/**
	 * Singleton pattern: get single instance.
	 */
	public static Dispatcher getInstance() {
		return instance;
	}

	/**
	 * Send event to all subscribers.
	 */
	public synchronized void broadcast(Event anEvent) {
		try {
			// Let the SessionManager loop through Sessions, calling
			// our Visitor Method for each Session. This is done to guard
			// synchronization with SessionManager and to optimize by
			// not getting an array of all sessions.
			Object[] args = new Object[2];
			args[1] = anEvent;
			Method method = sessionManagerVisitor.getMethod("visitBroadcast");
			SessionManager.getInstance().apply(sessionManagerVisitor, method, args);
		} catch (Throwable t) {
			Log.error("Error calling SessionManager.apply: ", t);
		}
	}

	/**
	 * Send event to subscribers matching Event subject.
	 */
	public synchronized void multicast(Event anEvent) {
		try {
			// Let the SessionManager loop through Sessions, calling
			// our Visitor Method for each Session. This is done to guard
			// synchronization with SessionManager and to optimize by
			// not getting an array of all sessions.
			Method method = sessionManagerVisitor.getMethod("visitMulticast");
			Object[] args = new Object[2];
			args[1] = anEvent;
			SessionManager.getInstance().apply(sessionManagerVisitor, method, args);
		} catch (Throwable t) {
			Log.error("Error calling SessionManager.apply: ", t);
		}
	}


	/**
	 * Send event to specific subscriber.
	 */
	public synchronized void unicast(Event event, String aSessionId) {
		// Get subscriber to send event to
		Session session = SessionManager.getInstance().getSession(aSessionId);
		if (session == null) {
			Log.warn("unicast: session with id=" + aSessionId + " does not exist");
			return;
		}

		// Send Event to subscriber.
		session.getSubscriber().onEvent((Event) event.clone());
	}

	/**
	 * Start Dispatcher.
	 */
	public void start() throws PushletException {
		Log.info("Dispatcher started");

		// Create callback for SessionManager visits.
		sessionManagerVisitor = new SessionManagerVisitor();
	}

	/**
	 * Stop Dispatcher.
	 */
	public void stop() {
		// Send abort control event to all subscribers.
		Log.info("Dispatcher stopped: broadcast abort to all subscribers");
		broadcast(new Event(E_ABORT));
	}

	/**
	 * Supplies Visitor methods for callbacks from SessionManager.
	 */
	private class SessionManagerVisitor {
		private final Map visitorMethods = new HashMap(2);

		SessionManagerVisitor() throws PushletException {

			try {
				// Setup Visitor Methods for callback from SessionManager
				// This is a slight opitmization over creating Method objects
				// on each invokation.
				Class[] argsClasses = {Session.class, Event.class};
				visitorMethods.put("visitMulticast", this.getClass().getMethod("visitMulticast", argsClasses));
				visitorMethods.put("visitBroadcast", this.getClass().getMethod("visitBroadcast", argsClasses));
			} catch (NoSuchMethodException e) {
				throw new PushletException("Failed to setup SessionManagerVisitor", e);
			}
		}

		/**
		 * Return Visitor Method by name.
		 */
		public Method getMethod(String aName) {
			return (Method) visitorMethods.get(aName);

		}

		/**
		 * Visitor method called by SessionManager.
		 */
		public void visitBroadcast(Session aSession, Event event) {
			aSession.getSubscriber().onEvent((Event) event.clone());
		}

		/**
		 * Visitor method called by SessionManager.
		 */
		public void visitMulticast(Session aSession, Event event) {
			Subscriber subscriber = aSession.getSubscriber();
			Event clonedEvent;
			Subscription subscription;

			// Send only if the subscriber's criteria
			// match the event.
			if ((subscription = subscriber.match(event)) != null) {
				// Personalize event
				clonedEvent = (Event) event.clone();

				// Set subscription id and optional label
				clonedEvent.setField(P_SUBSCRIPTION_ID, subscription.getId());
				if (subscription.getLabel() != null) {
					event.setField(P_SUBSCRIPTION_LABEL, subscription.getLabel());
				}

				subscriber.onEvent(clonedEvent);
			}
		}
	}
}

/*
 * $Log: Dispatcher.java,v $
 * Revision 1.9  2007/12/04 13:55:53  justb
 * reimplement SessionManager concurrency (prev version was not thread-safe!)
 *
 * Revision 1.8  2007/11/23 14:33:07  justb
 * core classes now configurable through factory
 *
 * Revision 1.7  2005/02/28 12:45:59  justb
 * introduced Command class
 *
 * Revision 1.6  2005/02/28 09:14:55  justb
 * sessmgr/dispatcher factory/singleton support
 *
 * Revision 1.5  2005/02/21 16:59:06  justb
 * SessionManager and session lease introduced
 *
 * Revision 1.4  2005/02/21 11:50:46  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *
 * Revision 1.3  2005/02/18 12:36:47  justb
 * changes for renaming and configurability
 *
 * Revision 1.2  2005/02/18 10:07:23  justb
 * many renamings of classes (make names compact)
 *
 * Revision 1.1  2005/02/18 09:54:15  justb
 * refactor: rename Publisher Dispatcher and single Subscriber class
 *
 * Revision 1.14  2005/02/16 14:39:34  justb
 * fixed leave handling and added "poll" mode
 *
 * Revision 1.13  2004/10/24 20:50:35  justb
 * refine subscription with label and sending sid and label on events
 *
 * Revision 1.12  2004/10/24 12:58:18  justb
 * revised client and test classes for new protocol
 *
 * Revision 1.11  2004/09/26 21:39:43  justb
 * allow multiple subscriptions and out-of-band requests
 *
 * Revision 1.10  2004/09/20 22:01:38  justb
 * more changes for new protocol
 *
 * Revision 1.9  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.8  2004/08/13 23:36:05  justb
 * rewrite of Pullet into Pushlet "pull" mode
 *
 * Revision 1.7  2004/08/12 13:18:54  justb
 * cosmetic changes
 *
 * Revision 1.6  2004/03/10 15:45:55  justb
 * many cosmetic changes
 *
 * Revision 1.5  2004/03/10 13:59:28  justb
 * rewrite using Collection classes and finer synchronization
 *
 * Revision 1.4  2003/08/15 08:37:40  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.3  2003/08/12 08:54:40  justb
 * added getSubscriberCount() and use Log
 *
 * Revision 1.2  2003/05/18 16:15:08  justb
 * support for XML encoded Events
 *
 * Revision 1.1.1.1  2002/09/24 21:02:31  justb
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
