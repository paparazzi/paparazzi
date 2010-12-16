// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.util;

/**
 * Generic exception wrapper.
 *
 * @author Just van den Broecke
 * @version $Id: PushletException.java,v 1.1 2005/02/15 15:14:34 justb Exp $
 */
public class PushletException extends Exception {

	private PushletException() {
	}

	public PushletException(String aMessage, Throwable t) {
		super(aMessage + "\n embedded exception=" + t.toString());
	}

	public PushletException(String aMessage) {
		super(aMessage);
	}

	public PushletException(Throwable t) {
		this("PushletException: ", t);
	}

	public String toString() {
		return "PushletException: " + getMessage();
	}
}

/*
 * $Log: PushletException.java,v $
 * Revision 1.1  2005/02/15 15:14:34  justb
 * *** empty log message ***
 *

 *
 */