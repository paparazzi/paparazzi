// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.util;

import org.apache.log4j.Level;
import org.apache.log4j.LogManager;
import org.apache.log4j.Logger;

/**
 * Logger to use Log4j for logging.
 * <p/>
 * Logs using Log4j.
 * This class will require a log4j library in the classpath of the Pushlet.
 *
 * @author Uli Romahn
 * @version $Id: Log4jLogger.java,v 1.1 2007/12/07 12:57:40 justb Exp $
 */
public class Log4jLogger implements PushletLogger {

	/**
	 * Level intialized with default.
	 */
	private Logger logger = LogManager.getLogger("pushlet");


	/* (non-Javadoc)
		 * @see nl.justobjects.pushlet.util.PushletLogger#init()
		 */
	public void init() {
		setLevel(LOG_LEVEL_INFO);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#debug(java.lang.String)
	 */
	public void debug(String aMessage) {
		if (!logger.isDebugEnabled()) {
			return;
		}
		logger.debug(aMessage);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#error(java.lang.String)
	 */
	public void error(String aMessage) {
		logger.error(aMessage);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#error(java.lang.String, java.lang.Throwable)
	 */
	public void error(String aMessage, Throwable aThrowable) {
		logger.error(aMessage, aThrowable);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#fatal(java.lang.String)
	 */
	public void fatal(String aMessage) {
		logger.fatal(aMessage);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#fatal(java.lang.String, java.lang.Throwable)
	 */
	public void fatal(String aMessage, Throwable aThrowable) {
		logger.fatal(aMessage, aThrowable);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#info(java.lang.String)
	 */
	public void info(String aMessage) {
		if (!logger.isInfoEnabled()) {
			return;
		}
		logger.info(aMessage);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#trace(java.lang.String)
	 */
	public void trace(String aMessage) {
		logger.trace(aMessage);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#warn(java.lang.String)
	 */
	public void warn(String aMessage) {
		logger.warn(aMessage);
	}

	/* (non-Javadoc)
	 * @see nl.justobjects.pushlet.util.PushletLogger#warn(java.lang.String, java.lang.Throwable)
	 */
	public void warn(String aMessage, Throwable aThrowable) {
		logger.warn(aMessage, aThrowable);
	}

	/* (non-Javadoc)
		 * @see nl.justobjects.pushlet.util.PushletLogger#setLevel(int)
		 */
	public void setLevel(int aLevel) {
		if (aLevel < LOG_LEVEL_FATAL) {
			logger.setLevel(Level.OFF);
		} else {
			switch (aLevel) {
				case LOG_LEVEL_FATAL:
					logger.setLevel(Level.FATAL);
					break;
				case LOG_LEVEL_ERROR:
					logger.setLevel(Level.ERROR);
					break;
				case LOG_LEVEL_WARN:
					logger.setLevel(Level.WARN);
					break;
				case LOG_LEVEL_INFO:
					logger.setLevel(Level.INFO);
					break;
				case LOG_LEVEL_DEBUG:
					logger.setLevel(Level.DEBUG);
					break;
				case LOG_LEVEL_TRACE:
					logger.setLevel(Level.TRACE);
					break;
				default:
					logger.setLevel(Level.INFO);
			}
		}
	}
}
