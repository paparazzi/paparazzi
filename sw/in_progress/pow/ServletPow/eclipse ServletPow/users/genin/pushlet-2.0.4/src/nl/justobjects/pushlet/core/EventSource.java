// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

/**
 * Abstract Event source from which Events are pulled.
 *
 * @version $Id: EventSource.java,v 1.7 2007/11/23 14:33:07 justb Exp $
 * @author Just van den Broecke - Just Objects &copy;
 **/

/**
 * Interface for specifc Event(Pull/Push)Sources.
 */
public interface EventSource {
	/**
	 * Activate the event source.
	 */
	public void activate();

	/**
	 * Deactivate the event source.
	 */
	public void passivate();

	/**
	 * Halt the event source.
	 */
	public void stop();
}

/*
  * $Log: EventSource.java,v $
  * Revision 1.7  2007/11/23 14:33:07  justb
  * core classes now configurable through factory
  *
  * Revision 1.6  2005/02/21 11:50:46  justb
  * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
  *
  * Revision 1.5  2005/02/18 10:07:23  justb
  * many renamings of classes (make names compact)
  *
  * Revision 1.4  2004/09/03 22:35:37  justb
  * Almost complete rewrite, just checking in now
  *
  * Revision 1.3  2003/08/15 08:37:40  justb
  * fix/add Copyright+LGPL file headers and footers
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

