// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import javax.servlet.http.HttpServletResponse;
import java.io.IOException;
import java.io.ObjectOutputStream;

/**
 * Implementation of ClientAdapter that sends Events as serialized objects.
 * <p/>
 * NOTE: You are discouraged to use this adapter, since it is Java-only
 * and may have JVM-specific problems. Far better choice is to use XML
 * and the XMLAdapter.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: SerializedAdapter.java,v 1.4 2007/11/23 14:33:07 justb Exp $
 */
class SerializedAdapter implements ClientAdapter {
	private ObjectOutputStream out = null;
	public static final String CONTENT_TYPE = "application/x-java-serialized-object";
	private HttpServletResponse servletRsp;

	/**
	 * Initialize.
	 */
	public SerializedAdapter(HttpServletResponse aServletResponse) {
		servletRsp = aServletResponse;
	}

	public void start() throws IOException {

		servletRsp.setContentType(CONTENT_TYPE);

		// Use a serialized object output stream
		out = new ObjectOutputStream(servletRsp.getOutputStream());

		// Don't need this further
		servletRsp = null;
	}

	/**
	 * Push Event to client.
	 */
	public void push(Event anEvent) throws IOException {
		out.writeObject(anEvent);

		out.flush();
	}


	public void stop() throws IOException {
	}
}

/*
 * $Log: SerializedAdapter.java,v $
 * Revision 1.4  2007/11/23 14:33:07  justb
 * core classes now configurable through factory
 *
 * Revision 1.3  2005/02/28 12:45:59  justb
 * introduced Command class
 *
 * Revision 1.2  2005/02/21 11:50:46  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *
 * Revision 1.1  2005/02/18 10:07:23  justb
 * many renamings of classes (make names compact)
 *
 * Revision 1.4  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.3  2003/08/15 08:37:40  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.2  2003/05/18 16:13:48  justb
 * fixed blocking for java.net.URL with HTTP/1.1 (JVMs > 1.1)
 *
 * Revision 1.1.1.1  2002/09/24 21:02:31  justb
 * import to sourceforge
 *
 * Revision 1.1.1.1  2002/09/20 22:48:18  justb
 * import to SF
 *
 * Revision 1.1.1.1  2002/09/20 14:19:03  justb
 * first import into SF
 *
 * Revision 1.5  2002/04/15 20:42:41  just
 * reformatting and renaming GuardedQueue to EventQueue
 *
 * Revision 1.4  2000/12/27 22:39:35  just
 * no message
 *
 * Revision 1.3  2000/08/21 20:48:29  just
 * added CVS log and id tags plus copyrights
 *
 *
 */
