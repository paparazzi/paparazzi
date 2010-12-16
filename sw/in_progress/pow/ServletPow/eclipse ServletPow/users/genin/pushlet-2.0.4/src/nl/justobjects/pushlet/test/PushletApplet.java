// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.test;

import nl.justobjects.pushlet.client.PushletClient;
import nl.justobjects.pushlet.client.PushletClientListener;
import nl.justobjects.pushlet.core.Event;
import nl.justobjects.pushlet.core.Protocol;
import nl.justobjects.pushlet.util.PushletException;

import java.applet.Applet;
import java.awt.*;

/**
 * Tester for applet clients; displays incoming events in text area.
 *
 * @version $Id: PushletApplet.java,v 1.16 2005/02/18 09:54:15 justb Exp $
 * @author Just van den Broecke - Just Objects &copy;
 **/
public class PushletApplet extends Applet implements PushletClientListener, Protocol {
	private TextArea textArea;
	private String host = "localhost";
	private int port = 8080;
	private String subject;
	private PushletClient pushletClient;
	private String VERSION = "15.feb.05 #5";
	private String PUSH_MODE = Protocol.MODE_PULL;

	/** One-time setup. */
	public void init() {
		// Subject to subscribe to
		subject = getParameter(P_SUBJECT);

		host = getDocumentBase().getHost();
		port = getDocumentBase().getPort();

		// Hmm sometimes this value is -1...(Mozilla with Java 1.3.0 on Win)
		if (port == -1) {
			port = 80;
		}

		setLayout(new GridLayout(1, 1));
		textArea = new TextArea(15, 40);
		textArea.setForeground(Color.yellow);
		textArea.setBackground(Color.gray);
		textArea.setEditable(false);
		add(textArea);
		p("PushletApplet - " + VERSION);
	}

	public void start() {
		dbg("start()");
		bailout();

		try {
			pushletClient = new PushletClient(host, port);
			p("Created PushletClient");

			pushletClient.join();
			p("Joined server");

			pushletClient.listen(this, PUSH_MODE);
			p("Listening in mode=" + PUSH_MODE);

			pushletClient.subscribe(subject);
			p("Subscribed to=" + subject);
		} catch (PushletException pe) {
			p("Error exception=" + pe);
			bailout();
		}
	}

	public void stop() {
		dbg("stop()");
		bailout();
	}

	/** Abort event from server. */
	public void onAbort(Event theEvent) {
		p(theEvent.toXML());
		bailout();
	}

	/** Data event from server. */
	public void onData(Event theEvent) {
		p(theEvent.toXML());
	}

	/** Heartbeat event from server. */
	public void onHeartbeat(Event theEvent) {
		p(theEvent.toXML());
	}

	/** Error occurred. */
	public void onError(String message) {
		p(message);
		bailout();
	}

	private void bailout() {
		if (pushletClient != null) {
			p("Stopping PushletClient");
			try {
				pushletClient.leave();
			} catch (PushletException ignore) {
				p("Error during leave pe=" + ignore);

			}
			pushletClient = null;
		}
	}

	/** Generic print. */
	private void p(String s) {
		dbg("event: " + s);
		synchronized (textArea) {
			textArea.append(s + "\n");
		}
	}

	/** Generic print. */
	private void dbg(String s) {
		System.out.println("[PushletApplet] " + s);
	}
}

/*
 * $Log: PushletApplet.java,v $
 * Revision 1.16  2005/02/18 09:54:15  justb
 * refactor: rename Publisher Dispatcher and single Subscriber class
 *
 * Revision 1.15  2005/02/15 15:46:36  justb
 * client API improves
 *
 * Revision 1.14  2005/02/15 13:28:33  justb
 * use new PushletClient lib
 *
 * Revision 1.13  2004/10/25 21:22:26  justb
 * *** empty log message ***
 *
 * Revision 1.12  2004/10/24 13:52:52  justb
 * small fixes in client lib
 *
 * Revision 1.11  2004/10/24 12:58:19  justb
 * revised client and test classes for new protocol
 *
 * Revision 1.10  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.9  2004/08/12 13:18:54  justb
 * cosmetic changes
 *
 * Revision 1.8  2003/08/17 21:06:37  justb
 * version info change
 *
 * Revision 1.7  2003/08/15 08:37:41  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.6  2003/08/14 21:43:10  justb
 * improved Java client lifecycle; notably stopping listener thread
 *
 * Revision 1.5  2003/08/13 14:00:00  justb
 * some testing for applets; no real change
 *
 * Revision 1.4  2003/05/19 22:53:33  justb
 * more fixes for applets
 *
 * Revision 1.3  2003/05/19 21:56:29  justb
 * various fixes for applet clients
 *
 * Revision 1.2  2003/05/18 16:15:08  justb
 * support for XML encoded Events
 *
 * Revision 1.1.1.1  2002/09/24 21:02:33  justb
 * import to sourceforge
 *
 */
