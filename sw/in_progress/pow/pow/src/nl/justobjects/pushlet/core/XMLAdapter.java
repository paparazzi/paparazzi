// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.Log;

import javax.servlet.ServletOutputStream;
import javax.servlet.http.HttpServletResponse;
import java.io.IOException;

/**
 * ClientAdapter that sends Events as XML.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: XMLAdapter.java,v 1.7 2007/11/09 13:15:35 justb Exp $
 */
class XMLAdapter implements ClientAdapter {
	/**
	 * Header for strict XML
	 */
	// public static final String XML_HEAD = "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n";
	private String contentType = "text/plain;charset=UTF-8";
	private ServletOutputStream out = null;
	private HttpServletResponse servletRsp;
	private boolean strictXML;

	/**
	 * Initialize.
	 */
	public XMLAdapter(HttpServletResponse aServletResponse) {
		this(aServletResponse, false);
	}

	/**
	 * Initialize.
	 */
	public XMLAdapter(HttpServletResponse aServletResponse, boolean useStrictXML) {
		servletRsp = aServletResponse;

		// Strict XML implies returning a complete XML document
		strictXML = useStrictXML;
		if (strictXML) {
			contentType = "text/xml;charset=UTF-8";
		}
	}

	public void start() throws IOException {

		// If content type is plain text
		// then this is not a complete XML document, but rather
		// a stream of XML documents where each document is
		// an Event. In strict XML mode a complete document is returned.
		servletRsp.setContentType(contentType);

		out = servletRsp.getOutputStream();

		// Don't need this further
		servletRsp = null;

		// Start XML document if strict XML mode
		if (strictXML) {
			out.print("<pushlet>");
		}
	}

	/**
	 * Force client to refresh the request.
	 */
	public void push(Event anEvent) throws IOException {
		debug("event=" + anEvent);

		// Send the event as XML to the client and flush.
		out.print(anEvent.toXML(strictXML));
		out.flush();
	}

	/**
	 * No action.
	 */
	public void stop() throws IOException {
		// Close XML document if strict XML mode
		if (strictXML) {
			out.print("</pushlet>");
			out.flush();
		}
	}

	private void debug(String s) {
		Log.debug("[XMLAdapter]" + s);
	}
}

/*
 * $Log: XMLAdapter.java,v $
 * Revision 1.7  2007/11/09 13:15:35  justb
 * add charset=UTF-8 in returned HTTP content types
 *
 * Revision 1.6  2006/05/15 11:52:53  justb
 * updates mainly for AJAX client
 *
 * Revision 1.5  2006/05/06 00:06:28  justb
 * first rough version AJAX client
 *
 * Revision 1.4  2005/05/06 19:44:00  justb
 * added xml-strict format
 *
 * Revision 1.3  2005/02/28 12:45:59  justb
 * introduced Command class
 *
 * Revision 1.2  2005/02/21 11:50:47  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *
 * Revision 1.1  2005/02/18 10:07:23  justb
 * many renamings of classes (make names compact)
 *
 * Revision 1.7  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.6  2004/03/10 14:01:55  justb
 * formatting and *Subscriber refactoring
 *
 * Revision 1.5  2003/08/15 08:37:40  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.4  2003/08/13 14:00:00  justb
 * some testing for applets; no real change
 *
 * Revision 1.3  2003/08/12 09:57:06  justb
 * replaced all print statements to Log.*() calls
 *
 * Revision 1.2  2003/05/19 21:56:29  justb
 * various fixes for applet clients
 *
 * Revision 1.1  2003/05/18 16:12:28  justb
 * adding support for XML encoded Events
 */
