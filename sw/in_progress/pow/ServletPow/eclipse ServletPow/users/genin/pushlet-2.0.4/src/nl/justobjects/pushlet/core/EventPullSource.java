// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.Log;

/**
 * Abstract Event source from which Events are pulled.
 *
 * @version $Id: EventPullSource.java,v 1.15 2007/11/23 14:33:07 justb Exp $
 * @author Just van den Broecke - Just Objects &copy;
 **/

/**
 * ABC for specifc EventPullSources.
 */
abstract public class EventPullSource implements EventSource, Runnable {
	private volatile boolean alive = false;
	private volatile boolean active = false;
	private static int threadNum = 0;
	private Thread thread;

	public EventPullSource() {
	}

	abstract protected long getSleepTime();

	abstract protected Event pullEvent();

	public void start() {
		thread = new Thread(this, "EventPullSource-" + (++threadNum));
		thread.setDaemon(true);
		thread.start();
	}

	public boolean isAlive() {
		return alive;
	}

	/**
	 * Stop the event generator thread.
	 */
	public void stop() {
		alive = false;

		if (thread != null) {
			thread.interrupt();
			thread = null;
		}

	}

	/**
	 * Activate the event generator thread.
	 */
	synchronized public void activate() {
		if (active) {
			return;
		}
		active = true;
		if (!alive) {
			start();
			return;
		}
		Log.debug(getClass().getName() + ": notifying...");
		notifyAll();
	}

	/**
	 * Deactivate the event generator thread.
	 */
	public void passivate() {
		if (!active) {
			return;
		}
		active = false;
	}

	/**
	 * Main loop: sleep, generate event and publish.
	 */
	public void run() {
		Log.debug(getClass().getName() + ": starting...");
		alive = true;
		while (alive) {
			try {

				Thread.sleep(getSleepTime());

				// Stopped during sleep: end loop.
				if (!alive) {
					break;
				}

				// If passivated wait until we get
				// get notify()-ied. If there are no subscribers
				// it wasts CPU to remain producing events...
				synchronized (this) {
					while (!active) {
						Log.debug(getClass().getName() + ": waiting...");
						wait();
					}
				}

			} catch (InterruptedException e) {
				break;
			}

			try {
				// Derived class should produce an event.
				Event event = pullEvent();

				// Let the publisher push it to subscribers.
				Dispatcher.getInstance().multicast(event);
			} catch (Throwable t) {
				Log.warn("EventPullSource exception while multicasting ", t);
				t.printStackTrace();
			}
		}
		Log.debug(getClass().getName() + ": stopped");
	}
}

/*
  * $Log: EventPullSource.java,v $
  * Revision 1.15  2007/11/23 14:33:07  justb
  * core classes now configurable through factory
  *
  * Revision 1.14  2005/02/28 09:14:55  justb
  * sessmgr/dispatcher factory/singleton support
  *
  * Revision 1.13  2005/02/21 16:59:08  justb
  * SessionManager and session lease introduced
  *
  * Revision 1.12  2005/02/21 11:50:46  justb
  * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
  *
  * Revision 1.11  2005/02/18 10:07:23  justb
  * many renamings of classes (make names compact)
  *
  * Revision 1.10  2005/02/18 09:54:15  justb
  * refactor: rename Publisher Dispatcher and single Subscriber class
  *
  * Revision 1.9  2004/09/20 22:01:38  justb
  * more changes for new protocol
  *
  * Revision 1.8  2004/09/03 22:35:37  justb
  * Almost complete rewrite, just checking in now
  *
  * Revision 1.7  2004/08/15 16:00:15  justb
  * enhancements to pull mode
  *
  * Revision 1.6  2004/08/13 23:36:05  justb
  * rewrite of Pullet into Pushlet "pull" mode
  *
  * Revision 1.5  2004/03/10 14:01:55  justb
  * formatting and *Subscriber refactoring
  *
  * Revision 1.4  2003/08/15 08:37:40  justb
  * fix/add Copyright+LGPL file headers and footers
  *
  * Revision 1.3  2003/08/12 09:57:05  justb
  * replaced all print statements to Log.*() calls
  *
  * Revision 1.2  2003/05/18 16:15:08  justb
  * support for XML encoded Events
  *
  * Revision 1.1.1.1  2002/09/24 21:02:31  justb
  * import to sourceforge
  *
  * Revision 1.1.1.1  2002/09/20 22:48:17  justb
  * import to SF
  *
  * Revision 1.1.1.1  2002/09/20 14:19:03  justb
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
