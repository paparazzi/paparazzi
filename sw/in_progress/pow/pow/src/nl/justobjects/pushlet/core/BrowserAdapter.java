// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.Log;

import javax.servlet.http.HttpServletResponse;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Iterator;

/**
 * Generic implementation of ClientAdapter for browser clients.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: BrowserAdapter.java,v 1.6 2007/11/09 13:15:35 justb Exp $
 */
public class BrowserAdapter implements ClientAdapter, Protocol {

	public static final String START_DOCUMENT =
			"<html><head><meta http-equiv=\"Pragma\" content=\"no-cache\"><meta http-equiv=\"Expires\" content=\"Tue, 31 Dec 1997 23:59:59 GMT\"></head>"
					+ "<body>"
					+ "\n<script language=\"JavaScript\"> var url=\" \"; \nfunction refresh() { document.location.href=url; }</script>";
	public static final String END_DOCUMENT = "</body></html>";

	private PrintWriter servletOut;
	private HttpServletResponse servletRsp;
	private int bytesSent;

	/**
	 * Constructor.
	 */
	public BrowserAdapter(HttpServletResponse aServletResponse) {
		servletRsp = aServletResponse;
	}

	/**
	 * Generic init.
	 */
	public void start() throws IOException {
		// Keep servlet request/response objects until page ends in stop()
		// Content type as HTML
		servletRsp.setStatus(HttpServletResponse.SC_OK);
		servletRsp.setContentType("text/html;charset=UTF-8");

		// http://www.junlu.com/msg/45902.html
		// Log.debug("bufsize=" + aRsp.getBufferSize());
		servletOut = servletRsp.getWriter();
		send(START_DOCUMENT);
	}

	/**
	 * Push Event to client.
	 */
	public void push(Event anEvent) throws IOException {
		Log.debug("BCA event=" + anEvent.toXML());

		// Check if we should refresh
		if (anEvent.getEventType().equals(Protocol.E_REFRESH)) {
			// Append refresh and tail of HTML document
			// Construct the JS callback line to be sent as last line of doc.
			// This will refresh the request using the unique id to determine
			// the subscriber instance on the server. The client will wait for
			// a number of milliseconds.
			long refreshWaitMillis = Long.parseLong(anEvent.getField(P_WAIT));

			// Create servlet request for requesting next events (refresh)
			String url = anEvent.getField(P_URL);
			String jsRefreshTrigger = "\n<script language=\"JavaScript\">url=\'" + url + "\';\n setTimeout(\"refresh()\", " + refreshWaitMillis + ");\n</script>";


			send(jsRefreshTrigger + END_DOCUMENT);
		} else {
			send(event2JavaScript(anEvent));
		}
	}

	/**
	 * End HTML page in client browser.
	 */
	public void stop() {
		// To be garbage collected if adapter remains active
		servletOut = null;
	}

	/**
	 * Send any string to browser.
	 */
	protected void send(String s) throws IOException {
		// Send string to browser.
		// Log.debug("Adapter: sending: " + s);
		if (servletOut == null) {
			throw new IOException("Client adapter was stopped");
		}

		servletOut.print(s);

		servletOut.flush();

		// Note: this doesn't seem to have effect
		// in Tomcat 4/5 if the client already disconnected.
		servletRsp.flushBuffer();

		bytesSent += s.length();
		Log.debug("bytesSent= " + bytesSent);
		// Log.debug("BCA sent event: " + s);
	}

	/**
	 * Converts the Java Event to a JavaScript function call in browser page.
	 */
	protected String event2JavaScript(Event event) throws IOException {

		// Convert the event to a comma-separated string.
		String jsArgs = "";
		for (Iterator iter = event.getFieldNames(); iter.hasNext();) {
			String name = (String) iter.next();
			String value = event.getField(name);
			String nextArgument = (jsArgs.equals("") ? "" : ",") + "'" + name + "'" + ", \"" + value + "\"";
			jsArgs += nextArgument;
		}

		// Construct and return the function call */
		return "<script language=\"JavaScript\">parent.push(" + jsArgs + ");</script>";
	}

}

/*
 * $Log: BrowserAdapter.java,v $
 * Revision 1.6  2007/11/09 13:15:35  justb
 * add charset=UTF-8 in returned HTTP content types
 *
 * Revision 1.5  2006/05/15 11:52:53  justb
 * updates mainly for AJAX client
 *
 * Revision 1.4  2006/05/06 00:10:11  justb
 * various chgs but not too serious...
 *
 * Revision 1.3  2005/02/28 12:45:59  justb
 * introduced Command class
 *
 * Revision 1.2  2005/02/21 11:50:44  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *
 * Revision 1.1  2005/02/18 10:07:23  justb
 * many renamings of classes (make names compact)
 *
 * Revision 1.12  2005/02/15 13:30:23  justb
 * changes for Tomcat buffering (now working in tc4 and 5.0)
 *
 * Revision 1.11  2005/01/24 22:45:58  justb
 * getting safari to work
 *
 * Revision 1.10  2005/01/18 16:46:27  justb
 * buffer size setting ignored by tomcat workings
 *
 * Revision 1.9  2004/10/24 12:58:18  justb
 * revised client and test classes for new protocol
 *
 * Revision 1.8  2004/09/20 22:01:38  justb
 * more changes for new protocol
 *
 * Revision 1.7  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.6  2004/08/15 16:00:15  justb
 * enhancements to pull mode
 *
 * Revision 1.5  2004/08/13 23:36:05  justb
 * rewrite of Pullet into Pushlet "pull" mode
 *
 * Revision 1.4  2003/08/15 08:37:40  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.3  2003/08/12 09:57:05  justb
 * replaced all print statements to Log.*() calls
 *
 * Revision 1.2  2003/05/18 16:15:07  justb
 * support for XML encoded Events
 *
 * Revision 1.1.1.1  2002/09/24 21:02:30  justb
 * import to sourceforge
 *
 * Revision 1.1.1.1  2002/09/20 22:48:17  justb
 * import to SF
 *
 * Revision 1.1.1.1  2002/09/20 14:19:02  justb
 * first import into SF
 *
 * Revision 1.5  2002/04/15 20:42:41  just
 * reformatting and renaming GuardedQueue to EventQueue
 *
 * Revision 1.4  2000/12/27 22:39:35  just
 * no message
 *
 * Revision 1.3  2000/10/30 14:15:47  just
 * no message
 *
 *
 */

