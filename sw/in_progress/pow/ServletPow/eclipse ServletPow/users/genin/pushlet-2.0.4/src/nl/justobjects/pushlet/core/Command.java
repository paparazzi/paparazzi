// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.PushletException;
import nl.justobjects.pushlet.util.Servlets;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

/**
 * Wraps pushlet request/response data.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Command.java,v 1.4 2007/11/23 14:33:07 justb Exp $
 */
public class Command implements Protocol {

	/**
	 * Pushlet request event.
	 */
	public final Event reqEvent;

	/**
	 * Pushlet response  event.
	 */
	private Event rspEvent;

	/**
	 * HTTP Servlet GET/POST request.
	 */
	public final HttpServletRequest httpReq;

	/**
	 * HTTP Servlet GET/POST response.
	 */
	public final HttpServletResponse httpRsp;

	/**
	 * Pushlet session.
	 */
	public final Session session;

	/**
	 * Per-response client adapter.
	 */
	private ClientAdapter clientAdapter;

	/**
	 * Constructor.
	 */
	private Command(Session aSession, Event aRequestEvent, HttpServletRequest aHTTPReq, HttpServletResponse aHTTPRsp) {
		session = aSession;
		reqEvent = aRequestEvent;
		httpReq = aHTTPReq;
		httpRsp = aHTTPRsp;
	}

	/**
	 * Create new Command object.
	 */
	public static Command create(Session aSession, Event aReqEvent, HttpServletRequest aHTTPReq, HttpServletResponse aHTTPRsp) {
		return new Command(aSession, aReqEvent, aHTTPReq, aHTTPRsp);
	}

	/**
	 * Set pushlet response event.
	 */
	public void setResponseEvent(Event aResponseEvent) {
		rspEvent = aResponseEvent;
	}

	/**
	 * Get pushlet response event.
	 */
	public Event getResponseEvent() {
		return rspEvent;
	}

	/**
	 * Get client adapter for request.
	 */
	public ClientAdapter getClientAdapter() throws PushletException {
		if (clientAdapter == null) {
			// Create and initialize client-specific adapter.
			clientAdapter = createClientAdapter();
		}
		return clientAdapter;
	}

	/**
	 * Create client notifier based on "format" parameter passed in request.
	 */
	protected ClientAdapter createClientAdapter() throws PushletException {

		// Assumed to be set by parent.
		String outputFormat = session.getFormat();

		// Determine client adapter to create.
		if (outputFormat.equals(FORMAT_JAVASCRIPT)) {
			// Client expects to receive Events as JavaScript dispatch calls..
			return new BrowserAdapter(httpRsp);
		} else if (outputFormat.equals(FORMAT_SERIALIZED_JAVA_OBJECT)) {
			// Client expects to receive Events as Serialized Java Objects.
			return new SerializedAdapter(httpRsp);
		} else if (outputFormat.equals(FORMAT_XML)) {
			// Client expects to receive Events as stream of XML docs.
			return new XMLAdapter(httpRsp);
		} else if (outputFormat.equals(FORMAT_XML_STRICT)) {
			// Client expects to receive Events embedded in single XML doc.
			return new XMLAdapter(httpRsp, true);
		} else {
			throw new PushletException("Null or invalid output format: " + outputFormat);
		}
	}

	/**
	 * Sends HTTP response headers.
	 */
	protected void sendResponseHeaders() {
		// Just to try to prevent caching in any form.
		Servlets.setNoCacheHeaders(httpRsp);

		// Close connection for Java enabled browsers
		if (session.getUserAgent().indexOf("java") > 0) {
			// The connection should be closed after this request
			// NB: this allows sending a "long response". Some clients
			// in particular java.net.URL in VMs > 1.1 that use HTTP/1.1
			// will block if
			// - the content length is not sent
			// - if Connection: close HTTP header is not sent.
			//
			// Since we don't know the content length we will assume
			// the underlying servlet engine will use chunked encoding.
			httpRsp.setHeader("Connection", "close");
		}
	}


}

/*
  * $Log: Command.java,v $
  * Revision 1.4  2007/11/23 14:33:07  justb
  * core classes now configurable through factory
  *
  * Revision 1.3  2005/05/06 19:44:00  justb
  * added xml-strict format
  *
  * Revision 1.2  2005/02/28 17:25:15  justb
  * commented
  *
  * Revision 1.1  2005/02/28 12:45:59  justb
  * introduced Command class
  *
  *
  */
