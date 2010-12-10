// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.servlet;

import nl.justobjects.pushlet.core.*;
import nl.justobjects.pushlet.util.Log;
import nl.justobjects.pushlet.util.Servlets;
import nl.justobjects.pushlet.util.PushletException;
import nl.justobjects.pushlet.Version;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Enumeration;

/**
 * Servlet runs a Subscriber per request.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Pushlet.java,v 1.23 2007/12/04 13:55:53 justb Exp $
 */
public class Pushlet extends HttpServlet implements Protocol {

	public void init() throws ServletException {
		try {
			// Load configuration (from classpath or WEB-INF root path)
			String webInfPath = getServletContext().getRealPath("/") + "/WEB-INF";
			Config.load(webInfPath);

			Log.init();

			// Start
			Log.info("init() Pushlet Webapp - version=" + Version.SOFTWARE_VERSION + " built=" + Version.BUILD_DATE);

			// Start session manager
			SessionManager.getInstance().start();

			// Start event Dispatcher
			Dispatcher.getInstance().start();


			if (Config.getBoolProperty(Config.SOURCES_ACTIVATE)) {
				EventSourceManager.start(webInfPath);
			} else {
				Log.info("Not starting local event sources");
			}
		} catch (Throwable t) {
			throw new ServletException("Failed to initialize Pushlet framework " + t, t);
		}
	}

	public void destroy() {
		Log.info("destroy(): Exit Pushlet webapp");

		if (Config.getBoolProperty(Config.SOURCES_ACTIVATE)) {
			// Stop local event sources
			EventSourceManager.stop();
		} else {
			Log.info("No local event sources to stop");
		}

		// Should abort all subscribers
		Dispatcher.getInstance().stop();

		// Should stop all sessions
		SessionManager.getInstance().stop();
	}

	/**
	 * Servlet GET request: handles event requests.
	 */
	public void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
		Event event = null;

		try {
			// Event parm identifies event type from the client
			String eventType = Servlets.getParameter(request, P_EVENT);

			// Always must have an event type
			if (eventType == null) {
				Log.warn("Pushlet.doGet(): bad request, no event specified");
				response.sendError(HttpServletResponse.SC_BAD_REQUEST, "No eventType specified");
				return;
			}

			// Create Event and set attributes from parameters
			event = new Event(eventType);
			for (Enumeration e = request.getParameterNames(); e.hasMoreElements();) {
				String nextAttribute = (String) e.nextElement();
				event.setField(nextAttribute, request.getParameter(nextAttribute));
			}


		} catch (Throwable t) {
			// Error creating event
			Log.warn("Pushlet: Error creating event in doGet(): ", t);
			response.setStatus(HttpServletResponse.SC_BAD_REQUEST);
			return;
		}

		// Handle parsed request
		doRequest(event, request, response);

	}

	/**
	 * Servlet POST request: extracts event data from body.
	 */
	public void doPost(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
		Event event = null;
		try {
			// Create Event by parsing XML from input stream.
			event = EventParser.parse(new InputStreamReader(request.getInputStream()));

			// Always must have an event type
			if (event.getEventType() == null) {
				Log.warn("Pushlet.doPost(): bad request, no event specified");
				response.sendError(HttpServletResponse.SC_BAD_REQUEST, "No eventType specified");
				return;
			}


		} catch (Throwable t) {
			// Error creating event
			Log.warn("Pushlet:  Error creating event in doPost(): ", t);
			response.setStatus(HttpServletResponse.SC_BAD_REQUEST);
			return;
		}

		// Handle parsed request
		doRequest(event, request, response);

	}

	/**
	 * Generic request handler (GET+POST).
	 */
	protected void doRequest(Event anEvent, HttpServletRequest request, HttpServletResponse response) {
		// Must have valid event type.
		String eventType = anEvent.getEventType();
		try {

			// Get Session: either by creating (on Join eventType)
			// or by id (any other eventType, since client is supposed to have joined).
			Session session = null;
			if (eventType.startsWith(Protocol.E_JOIN)) {
				// Join request: create new subscriber
				session = SessionManager.getInstance().createSession(anEvent);

				String userAgent = request.getHeader("User-Agent");
				if (userAgent != null) {
					userAgent = userAgent.toLowerCase();
				} else {
					userAgent = "unknown";
				}
				session.setUserAgent(userAgent);

			} else {
				// Must be a request for existing Session

				// Get id
				String id = anEvent.getField(P_ID);

				// We must have an id value
				if (id == null) {
					response.sendError(HttpServletResponse.SC_BAD_REQUEST, "No id specified");
					Log.warn("Pushlet: bad request, no id specified event=" + eventType);
					return;
				}

				// We have an id: get the session object
				session = SessionManager.getInstance().getSession(id);

				// Check for invalid id
				if (session == null) {
					response.sendError(HttpServletResponse.SC_BAD_REQUEST, "Invalid or expired id: " + id);
					Log.warn("Pushlet:  bad request, no session found id=" + id + " event=" + eventType);
					return;
				}
			}

			// ASSERTION: we have a valid Session

			// Let Controller handle request further
			// including exceptions
			Command command = Command.create(session, anEvent, request, response);
			session.getController().doCommand(command);
		} catch (Throwable t) {
			// Hmm we should never ever get here
			Log.warn("Pushlet:  Exception in doRequest() event=" + eventType, t);
			t.printStackTrace();
			response.setStatus(HttpServletResponse.SC_INTERNAL_SERVER_ERROR);
		}

	}
}

/*
 * $Log: Pushlet.java,v $
 * Revision 1.23  2007/12/04 13:55:53  justb
 * reimplement SessionManager concurrency (prev version was not thread-safe!)
 *
 * Revision 1.22  2007/11/24 10:29:36  justb
 * add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
 *
 * Revision 1.21  2007/11/23 21:10:17  justb
 * add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
 *
 * Revision 1.20  2007/11/10 13:44:02  justb
 * pushlet.properties and sources.properties can now also be put under WEB-INF
 *
 * Revision 1.19  2006/05/15 11:52:53  justb
 * updates mainly for AJAX client
 *
 * Revision 1.18  2005/02/28 15:58:05  justb
 * added SimpleListener example
 *
 * Revision 1.17  2005/02/28 13:06:01  justb
 * introduced join-listen protocol service
 *
 * Revision 1.16  2005/02/28 12:45:59  justb
 * introduced Command class
 *
 * Revision 1.15  2005/02/28 09:14:56  justb
 * sessmgr/dispatcher factory/singleton support
 *
 * Revision 1.14  2005/02/25 15:13:04  justb
 * session id generation more robust
 *
 * Revision 1.13  2005/02/21 17:19:21  justb
 * move init()/destroy() to Pushlet servlet
 *
 * Revision 1.12  2005/02/21 16:59:17  justb
 * SessionManager and session lease introduced
 *
 * Revision 1.11  2005/02/21 11:50:47  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *
 * Revision 1.10  2005/02/20 13:05:32  justb
 * removed the Postlet (integrated in Pushlet protocol)
 *
 * Revision 1.9  2005/02/18 10:07:23  justb
 * many renamings of classes (make names compact)
 *
 * Revision 1.8  2005/01/13 14:47:15  justb
 * control evt: send response on same (control) connection
 *
 * Revision 1.7  2004/10/24 12:58:18  justb
 * revised client and test classes for new protocol
 *
 * Revision 1.6  2004/09/26 21:39:44  justb
 * allow multiple subscriptions and out-of-band requests
 *
 * Revision 1.5  2004/09/20 22:01:40  justb
 * more changes for new protocol
 *
 * Revision 1.4  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.3  2004/08/13 23:36:06  justb
 * rewrite of Pullet into Pushlet "pull" mode
 *
 * Revision 1.2  2003/08/15 08:37:40  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.1  2003/08/13 13:26:57  justb
 * moved all servlets to servlet package
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

