// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.util;

import nl.justobjects.pushlet.core.Config;
import nl.justobjects.pushlet.core.ConfigDefs;

/**
 * Logging wrapper.
 * <p/>
 * Provides a hook to direct logging to your own logging library.  Override the DefaultLogger class by setting
 * "logger.class" in pushlet.properties to your own logger
 * to integrate your own logging library.
 *
 * @author Just van den Broecke
 * @version $Id: Log.java,v 1.5 2007/12/07 12:57:40 justb Exp $
 */
public class Log implements ConfigDefs {
	/**
	 * Init with default to have at least some logging.
	 */
	private static PushletLogger logger = new DefaultLogger();

	/**
	 * General purpose initialization.
	 */
	static public void init() {
		try {
			logger = (PushletLogger) Config.getClass(LOGGER_CLASS, "nl.justobjects.pushlet.util.DefaultLogger").newInstance();
		} catch (Throwable t) {
			// Hmmm cannot log this since we don't have a log...
			System.out.println("Cannot instantiate Logger from config ex=" + t);
			return;
		}

		logger.init();

		// Set log level
		logger.setLevel(Config.getIntProperty(Config.LOG_LEVEL));

		logger.info("Logging intialized logger class=" + logger.getClass());
	}

	/**
	 * Log message for trace level.
	 *
	 * @param aMessage the message to be logged
	 */
	static public void trace(String aMessage) {
		logger.debug(aMessage);
	}

	/**
	 * Log message for debug level.
	 *
	 * @param aMessage the message to be logged
	 */
	static public void debug(String aMessage) {
		logger.debug(aMessage);
	}

	/**
	 * Log message for info level.
	 *
	 * @param aMessage the message to be logged
	 */
	static public void info(String aMessage) {
		logger.info(aMessage);
	}

	/**
	 * Log message for warning level.
	 *
	 * @param aMessage the message to be logged
	 */
	static public void warn(String aMessage) {
		logger.warn(aMessage);
	}

	/**
	 * Log message for warning level with exception.
	 *
	 * @param aMessage   the message to be logged
	 * @param aThrowable the exception
	 */
	static public void warn(String aMessage, Throwable aThrowable) {
		logger.warn(aMessage, aThrowable);
	}

	/**
	 * Log message for error level.
	 *
	 * @param aMessage the message to be logged
	 */
	static public void error(String aMessage) {
		logger.error(aMessage);
	}

	/**
	 * Log message (error level with exception).
	 *
	 * @param aMessage   the message to be logged
	 * @param aThrowable the exception
	 */
	static public void error(String aMessage, Throwable aThrowable) {
		logger.error(aMessage, aThrowable);
	}

	/**
	 * Log message for fatal level.
	 *
	 * @param aMessage the message to be logged
	 */
	static public void fatal(String aMessage) {
		logger.fatal(aMessage);
	}

	/**
	 * Log message (fatal level with exception).
	 *
	 * @param aMessage   the message to be logged
	 * @param aThrowable the exception
	 */
	static public void fatal(String aMessage, Throwable aThrowable) {
		logger.fatal(aMessage, aThrowable);
	}

	/**
	 * Set log level
	 *
	 * @param aLevel the message to be logged
	 */
	static public void setLevel(int aLevel) {
		logger.setLevel(aLevel);
	}
}

/*
* $Log: Log.java,v $
* Revision 1.5  2007/12/07 12:57:40  justb
* added log4j and make it the default logging method
*
* Revision 1.4  2007/11/23 21:29:43  justb
* add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
*
* Revision 1.3  2007/11/23 21:10:17  justb
* add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
*
* Revision 1.2  2005/02/21 11:15:59  justb
* support log levels
*
* Revision 1.1  2005/02/18 10:07:23  justb
* many renamings of classes (make names compact)
*
* Revision 1.7  2004/09/03 22:35:37  justb
* Almost complete rewrite, just checking in now
*
* Revision 1.6  2004/08/12 13:16:08  justb
* make debug flag false
*
* Revision 1.5  2004/03/10 14:01:55  justb
* formatting and *Subscriber refactoring
*
* Revision 1.4  2003/08/15 09:54:46  justb
* fix javadoc warnings
*
* Revision 1.3  2003/08/15 08:37:40  justb
* fix/add Copyright+LGPL file headers and footers
*
* Revision 1.2  2003/08/12 09:42:47  justb
* enhancements
*
* Revision 1.1  2003/08/12 08:46:00  justb
* cvs comment tags added
*
*
*/