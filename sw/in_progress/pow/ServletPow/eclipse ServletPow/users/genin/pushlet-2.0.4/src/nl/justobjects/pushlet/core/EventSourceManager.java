// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.Log;
import nl.justobjects.pushlet.util.Sys;

import java.util.Enumeration;
import java.util.Properties;
import java.util.Vector;
import java.io.File;

/**
 * Maintains lifecycle of event sources.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: EventSourceManager.java,v 1.14 2007/11/10 13:44:02 justb Exp $
 */
public class EventSourceManager {
	private static Vector eventSources = new Vector(0);
	private static final String PROPERTIES_FILE = "sources.properties";

	/**
	 * Initialize event sources from properties file.
	 */
	public static void start(String aDirPath) {
		// Load Event sources using properties file.
		Log.info("EventSourceManager: start");

		Properties properties = null;

		try {
			properties = Sys.loadPropertiesResource(PROPERTIES_FILE);
		} catch (Throwable t) {
			// Try from provided dir (e.g. WEB_INF/pushlet.properties)
			String filePath = aDirPath + File.separator + PROPERTIES_FILE;
			Log.info("EventSourceManager: cannot load " + PROPERTIES_FILE + " from classpath, will try from " + filePath);

			try {
				properties = Sys.loadPropertiesFile(filePath);
			} catch (Throwable t2) {
				Log.fatal("EventSourceManager: cannot load properties file from " + filePath, t);

				// Give up
				Log.warn("EventSourceManager: not starting local event sources (maybe that is what you want)");
				return;
			}
		}

		// Create event source collection
		eventSources = new Vector(properties.size());

		// Add the configured sources
		for (Enumeration e = properties.keys(); e.hasMoreElements();) {
			String nextKey = (String) e.nextElement();
			String nextClass = properties.getProperty(nextKey);
			EventSource nextEventSource = null;
			try {
				nextEventSource = (EventSource) Class.forName(nextClass).newInstance();
				Log.info("created EventSource: key=" + nextKey + " class=" + nextClass);
				eventSources.addElement(nextEventSource);
			} catch (Exception ex) {
				Log.warn("Cannot create EventSource: class=" + nextClass, ex);
			}
		}

		activate();
	}

	/**
	 * Activate all event sources.
	 */
	public static void activate() {
		Log.info("Activating " + eventSources.size() + " EventSources");
		for (int i = 0; i < eventSources.size(); i++) {
			((EventSource) eventSources.elementAt(i)).activate();
		}
		Log.info("EventSources activated");
	}

	/**
	 * Deactivate all event sources.
	 */
	public static void passivate() {
		Log.info("Passivating " + eventSources.size() + " EventSources");
		for (int i = 0; i < eventSources.size(); i++) {
			((EventSource) eventSources.elementAt(i)).passivate();
		}
		Log.info("EventSources passivated");
	}

	/**
	 * Halt event sources.
	 */
	public static void stop() {
		Log.info("Stopping " + eventSources.size() + " EventSources...");
		for (int i = 0; i < eventSources.size(); i++) {
			((EventSource) eventSources.elementAt(i)).stop();
		}
		Log.info("EventSources stopped");
	}

}

/*
 * $Log: EventSourceManager.java,v $
 * Revision 1.14  2007/11/10 13:44:02  justb
 * pushlet.properties and sources.properties can now also be put under WEB-INF
 *
 * Revision 1.13  2005/02/21 11:50:46  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *
 * Revision 1.12  2005/02/18 12:36:47  justb
 * changes for renaming and configurability
 *
 * Revision 1.11  2005/02/18 10:07:23  justb
 * many renamings of classes (make names compact)
 *
 * Revision 1.10  2005/02/15 13:29:49  justb
 * use Sys.loadPropertiesResource()
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
 * Revision 1.5  2004/08/12 13:18:54  justb
 * cosmetic changes
 *
 * Revision 1.4  2003/08/15 08:37:40  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.3  2003/08/12 09:41:35  justb
 * replace static initalizer with explicit init()
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
 * Revision 1.5  2002/04/15 20:42:41  just
 * reformatting and renaming GuardedQueue to EventQueue
 *
 * Revision 1.4  2000/10/30 14:15:47  just
 * no message
 *
 * Revision 1.3  2000/08/31 08:26:54  just
 * Changed classloader that loads eventsources.properties to use EventSourceManager's classloader
 *
 * Revision 1.2  2000/08/21 20:48:29  just
 * added CVS log and id tags plus copyrights
 *
 *
 */
