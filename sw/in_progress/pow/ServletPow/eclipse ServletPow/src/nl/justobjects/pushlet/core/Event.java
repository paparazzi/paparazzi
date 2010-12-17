// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.Sys;

import java.io.Serializable;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * Represents the event data.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Event.java,v 1.13 2007/11/23 14:33:07 justb Exp $
 */
public class Event implements Protocol, Serializable {

	protected Map attributes = new HashMap(3);

	public Event(String anEventType) {
		this(anEventType, null);
	}

	public Event(String anEventType, Map theAttributes) {

		if (theAttributes != null) {
			setAttrs(theAttributes);
		}

		// Set required field event type
		setField(P_EVENT, anEventType);

		// Set time in seconds since 1970
		setField(P_TIME, System.currentTimeMillis() / 1000);
	}

	public Event(Map theAttributes) {
		if (!theAttributes.containsKey(P_EVENT)) {
			throw new IllegalArgumentException(P_EVENT + " not found in attributes");
		}
		setAttrs(theAttributes);
	}

	public static Event createDataEvent(String aSubject) {
		return createDataEvent(aSubject, null);
	}

	public static Event createDataEvent(String aSubject, Map theAttributes) {
		Event dataEvent = new Event(E_DATA, theAttributes);
		dataEvent.setField(P_SUBJECT, aSubject);
		return dataEvent;
	}

	public String getEventType() {
		return getField(P_EVENT);
	}

	public String getSubject() {
		return getField(P_SUBJECT);
	}

	public void setField(String name, String value) {
		attributes.put(name, value);
	}

	public void setField(String name, int value) {
		attributes.put(name, value + "");
	}

	public void setField(String name, long value) {
		attributes.put(name, value + "");
	}

	public String getField(String name) {
		return (String) attributes.get(name);
	}

	/**
	 * Return field; if null return default.
	 */
	public String getField(String name, String aDefault) {
		String result = getField(name);
		return result == null ? aDefault : result;
	}

	public Iterator getFieldNames() {
		return attributes.keySet().iterator();
	}

	public String toString() {
		return attributes.toString();
	}

	/**
	 * Convert to HTTP query string.
	 */
	public String toQueryString() {
		String queryString = "";
		String amp = "";
		for (Iterator iter = getFieldNames(); iter.hasNext();) {
			String nextAttrName = (String) iter.next();
			String nextAttrValue = getField(nextAttrName);
			queryString = queryString + amp + nextAttrName + "=" + nextAttrValue;
			// After first add "&".
			amp = "&";
		}

		return queryString;
	}

	public String toXML(boolean strict) {
		String xmlString = "<event ";
		for (Iterator iter = getFieldNames(); iter.hasNext();) {
			String nextAttrName = (String) iter.next();
			String nextAttrValue = getField(nextAttrName);
			xmlString = xmlString + nextAttrName + "=\"" + (strict ? Sys.forHTMLTag(nextAttrValue) : nextAttrValue) + "\" ";
		}

		xmlString += "/>";
		return xmlString;
	}

	public String toXML() {
		return toXML(false);
	}

	public Object clone() {
		// Clone the Event by using copy constructor
		return new Event(attributes);
	}

	/**
	 * Copy given attributes into event attributes
	 */
	private void setAttrs(Map theAttributes) {
		attributes.putAll(theAttributes);
	}
}

/*
  * $Log: Event.java,v $
  * Revision 1.13  2007/11/23 14:33:07  justb
  * core classes now configurable through factory
  *
  * Revision 1.12  2006/05/15 11:52:53  justb
  * updates mainly for AJAX client
  *
  * Revision 1.11  2006/05/06 00:06:28  justb
  * first rough version AJAX client
  *
  * Revision 1.10  2005/02/21 11:50:46  justb
  * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
  *
  * Revision 1.9  2005/02/20 13:05:32  justb
  * removed the Postlet (integrated in Pushlet protocol)
  *
  * Revision 1.8  2005/02/15 13:29:24  justb
  * add toQueryString()
  *
  * Revision 1.7  2005/01/18 16:47:10  justb
  * protocol changes for v2 and publishing from pushlet client
  *
  * Revision 1.6  2005/01/13 14:47:15  justb
  * control evt: send response on same (control) connection
  *
  * Revision 1.5  2004/09/03 22:35:37  justb
  * Almost complete rewrite, just checking in now
  *
  * Revision 1.4  2004/08/15 16:00:15  justb
  * enhancements to pull mode
  *
  * Revision 1.3  2003/08/15 08:37:40  justb
  * fix/add Copyright+LGPL file headers and footers
  *
  * Revision 1.2  2003/05/18 16:15:08  justb
  * support for XML encoded Events
  *
  * Revision 1.1.1.1  2002/09/24 21:02:30  justb
  * import to sourceforge
  *
  */
