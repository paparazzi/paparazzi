// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.util;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;


/**
 * Servlet utilities.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Servlets.java,v 1.2 2007/11/23 21:10:17 justb Exp $
 */
public class Servlets {

	/**
	 * Get parameter; if not set or empty return null.
	 */
	public static String getParameter(HttpServletRequest aRequest, String aName) {
		return getParameter(aRequest, aName, null);
	}

	/**
	 * Get parameter; if not set or empty return specified default value.
	 */
	public static String getParameter(HttpServletRequest aRequest, String aName, String aDefault) {
		String value = aRequest.getParameter(aName);
		if (value == null || value.length() == 0) {
			value = aDefault;
		}
		return value;
	}

	/**
	 * Set HTTP headers to prevent caching.
	 */
	public static void setNoCacheHeaders(HttpServletResponse aResponse) {
		// Set to expire far in the past.
		aResponse.setHeader("Expires", "Sat, 6 May 1995 12:00:00 GMT");

		// Set standard HTTP/1.1 no-cache headers.
		aResponse.setHeader("Cache-Control", "no-store, no-cache, must-revalidate");

		// Set IE extended HTTP/1.1 no-cache headers (use addHeader).
		aResponse.addHeader("Cache-Control", "post-check=0, pre-check=0");

		// Set standard HTTP/1.0 no-cache header.
		aResponse.setHeader("Pragma", "no-cache");

	}

}

/*
 * $Log: Servlets.java,v $
 * Revision 1.2  2007/11/23 21:10:17  justb
 * add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
 *
 * Revision 1.1  2004/09/20 22:01:40  justb
 * more changes for new protocol
 *
 *
 */


