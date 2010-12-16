// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.util;

/**
 * Default logger.
 * <p/>
 * Logs to stdout. Override this class by setting "logger.class" in pushlet.properties to your own logger
 * to integrate your own logging library.
 *
 * @author Just van den Broecke
 * @version $Id: DefaultLogger.java,v 1.2 2007/12/07 12:57:40 justb Exp $
 */
public class DefaultLogger implements PushletLogger {


	/**
	 * Level intialized with default.
	 */
	private int level = LOG_LEVEL_INFO;

	public DefaultLogger() {
	}

	public void init() {

	}

	/**
	 * Log message for trace level.
	 *
	 * @param aMessage the message to be logged
	 */
	public void trace(String aMessage) {
		if (level < LOG_LEVEL_TRACE) {
			return;
		}
		print("TRACE", aMessage);
	}

	/**
	 * Log message for debug level.
	 *
	 * @param aMessage the message to be logged
	 */
	public void debug(String aMessage) {
		if (level < LOG_LEVEL_DEBUG) {
			return;
		}
		print("DEBUG", aMessage);
	}

	/**
	 * Log message for info level.
	 *
	 * @param aMessage the message to be logged
	 */
	public void info(String aMessage) {
		if (level < LOG_LEVEL_INFO) {
			return;
		}
		print("INFO", aMessage);
	}

	/**
	 * Log message for warning level.
	 *
	 * @param aMessage the message to be logged
	 */
	public void warn(String aMessage) {
		if (level < LOG_LEVEL_WARN) {
			return;
		}
		print("WARN", aMessage);
	}

	/**
	 * Log message for warning level with exception.
	 *
	 * @param aMessage   the message to be logged
	 * @param aThrowable the exception
	 */
	public void warn(String aMessage, Throwable aThrowable) {
		warn(aMessage + " exception=" + aThrowable);
	}

	/**
	 * Log message for error level.
	 *
	 * @param aMessage the message to be logged
	 */
	public void error(String aMessage) {
		if (level < LOG_LEVEL_ERROR) {
			return;
		}
		print("FATAL", aMessage);
	}

	/**
	 * Log message (error level with exception).
	 *
	 * @param aMessage   the message to be logged
	 * @param aThrowable the exception
	 */
	public void error(String aMessage, Throwable aThrowable) {
		error(aMessage + " exception=" + aThrowable);
	}

	/**
	 * Log message for fatal level.
	 *
	 * @param aMessage the message to be logged
	 */
	public void fatal(String aMessage) {
		if (level < LOG_LEVEL_FATAL) {
			return;
		}
		print("FATAL", aMessage);
	}

	/**
	 * Log message (fatal level with exception).
	 *
	 * @param aMessage   the message to be logged
	 * @param aThrowable the exception
	 */
	public void fatal(String aMessage, Throwable aThrowable) {
		fatal(aMessage + " exception=" + aThrowable);
	}

	/**
	 * Set log level
	 *
	 * @param aLevel the message to be logged
	 */
	public void setLevel(int aLevel) {
		level = aLevel;
	}

	/**
	 * Print message.
	 *
	 * @param aTag	 the log type
	 * @param aMessage the message to be logged
	 */
	private void print(String aTag, String aMessage) {
		// SImple std out e.g. to catalina.out in Tomcat
		System.out.println("Pushlet[" + aTag + "] " + aMessage);
	}

}

/*
* $Log: DefaultLogger.java,v $
* Revision 1.2  2007/12/07 12:57:40  justb
* added log4j and make it the default logging method
*
* Revision 1.1  2007/11/23 21:10:17  justb
* add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
*
*
*
*/