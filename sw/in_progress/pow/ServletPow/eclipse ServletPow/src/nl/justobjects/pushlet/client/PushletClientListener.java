// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.client;

import nl.justobjects.pushlet.core.Event;
import nl.justobjects.pushlet.core.Protocol;

/**
 * Interface for listener of the PushletClient object.
 *
 * @version $Id: PushletClientListener.java,v 1.5 2005/02/21 11:50:37 justb Exp $
 * @author Just van den Broecke - Just Objects &copy;
 **/
public interface PushletClientListener extends Protocol {
	/** Abort event from server. */
	public void onAbort(Event theEvent);

	/** Data event from server. */
	public void onData(Event theEvent);

	/** Heartbeat event from server. */
	public void onHeartbeat(Event theEvent);

	/** Error occurred. */
	public void onError(String message);
}

/*
* $Log: PushletClientListener.java,v $
* Revision 1.5  2005/02/21 11:50:37  justb
* ohase1 of refactoring Subscriber into Session/Controller/Subscriber
*
* Revision 1.4  2005/02/15 15:46:31  justb
* client API improves
*
* Revision 1.3  2004/10/24 12:58:18  justb
* revised client and test classes for new protocol
*
* Revision 1.2  2004/09/03 22:35:37  justb
* Almost complete rewrite, just checking in now
*
* Revision 1.1  2004/03/10 20:14:17  justb
* renamed all *JavaPushletClient* to *PushletClient*
*
* Revision 1.3  2003/08/15 08:37:40  justb
* fix/add Copyright+LGPL file headers and footers
*
*
*/