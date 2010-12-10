// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;


/**
 * Definition of config property strings.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: ConfigDefs.java,v 1.9 2007/12/07 12:57:40 justb Exp $
 */
public interface ConfigDefs {
	/**
	 * Class factory definitions, used to insert your custom classes.
	 */
	public static final String CONTROLLER_CLASS = "controller.class";
	public static final String DISPATCHER_CLASS = "dispatcher.class";
	public static final String LOGGER_CLASS = "logger.class";
	public static final String SESSION_MANAGER_CLASS = "sessionmanager.class";
	public static final String SESSION_CLASS = "session.class";
	public static final String SUBSCRIBER_CLASS = "subscriber.class";
	public static final String SUBSCRIPTION_CLASS = "subscription.class";

	/**
	 * Session management.
	 */
	public static final String SESSION_ID_SIZE = "session.id.size";
	public static final String SESSION_ID_GENERATION = "session.id.generation";
	public static final String SESSION_ID_GENERATION_UUID = "uuid";
	public static final String SESSION_ID_GENERATION_RANDOMSTRING = "randomstring";
	public static final String SESSION_TIMEOUT_MINS = "session.timeout.mins";

	public static final String SOURCES_ACTIVATE = "sources.activate";

	/**
	 * Logging
	 */
	public static final String LOG_LEVEL = "log.level";
	public static final int LOG_LEVEL_FATAL = 1;
	public static final int LOG_LEVEL_ERROR = 2;
	public static final int LOG_LEVEL_WARN = 3;
	public static final int LOG_LEVEL_INFO = 4;
	public static final int LOG_LEVEL_DEBUG = 5;
	public static final int LOG_LEVEL_TRACE = 6;

	/**
	 * Queues
	 */
	public static final String QUEUE_SIZE = "queue.size";
	public static final String QUEUE_READ_TIMEOUT_MILLIS = "queue.read.timeout.millis";
	public static final String QUEUE_WRITE_TIMEOUT_MILLIS = "queue.write.timeout.millis";

	/**
	 * Listening modes.
	 */
	public static final String LISTEN_FORCE_PULL_ALL = "listen.force.pull.all";
	public static final String LISTEN_FORCE_PULL_AGENTS = "listen.force.pull.agents";


	public static final String PULL_REFRESH_TIMEOUT_MILLIS = "pull.refresh.timeout.millis";
	public static final String PULL_REFRESH_WAIT_MIN_MILLIS = "pull.refresh.wait.min.millis";
	public static final String PULL_REFRESH_WAIT_MAX_MILLIS = "pull.refresh.wait.max.millis";


	public static final String POLL_REFRESH_TIMEOUT_MILLIS = "poll.refresh.timeout.millis";
	public static final String POLL_REFRESH_WAIT_MIN_MILLIS = "poll.refresh.wait.min.millis";
	public static final String POLL_REFRESH_WAIT_MAX_MILLIS = "poll.refresh.wait.max.millis";

}

/*
  * $Log: ConfigDefs.java,v $
  * Revision 1.9  2007/12/07 12:57:40  justb
  * added log4j and make it the default logging method
  *
  * Revision 1.8  2007/11/23 21:10:17  justb
  * add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
  *
  * Revision 1.7  2007/11/23 14:33:07  justb
  * core classes now configurable through factory
  *
  * Revision 1.6  2007/11/10 14:48:35  justb
  * make session key generation configurable (can use uuid)
  *
  * Revision 1.5  2005/02/28 09:14:55  justb
  * sessmgr/dispatcher factory/singleton support
  *
  * Revision 1.4  2005/02/21 16:59:00  justb
  * SessionManager and session lease introduced
  *
  * Revision 1.3  2005/02/21 11:50:46  justb
  * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
  *
  * Revision 1.2  2005/02/21 11:16:44  justb
  * add log level config prop
  *
  * Revision 1.1  2005/02/18 12:36:47  justb
  * changes for renaming and configurability
  *
  *
  *
  */

