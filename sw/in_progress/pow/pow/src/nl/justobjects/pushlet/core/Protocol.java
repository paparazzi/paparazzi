// Copyright (c) 2004 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;


/**
 * Constants for Pushlet protocols.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Protocol.java,v 1.15 2007/11/23 14:33:07 justb Exp $
 */
public interface Protocol {
	/**
	 * Default URI .
	 */
	public static final String DEFAULT_SERVLET_URI = "/pushlet/pushlet.srv";

	//
	// Common protocol header/parameter names
	//

	/**
	 * Event type (join, leave, data, subscribe etc) .
	 */
	public static final String P_EVENT = "p_event";

	/**
	 * Time in seconds since 1970
	 */
	public static final String P_TIME = "p_time";

	/**
	 * Event sequence number, numbers per-client.
	 */
	public static final String P_SEQ = "p_seq";

	/**
	 * Subject (topic) of data event.
	 */
	public static final String P_SUBJECT = "p_subject";

	/**
	 * Originator of Event.
	 */
	public static final String P_FROM = "p_from";

	/**
	 * Addressee of Event, subject or client p_id.
	 */
	public static final String P_TO = "p_to";

	/**
	 * Identifier for client instance within server.
	 */
	public static final String P_ID = "p_id";

	/**
	 * Subscription id, identifies single subscription.
	 */
	public static final String P_SUBSCRIPTION_ID = "p_sid";

	/**
	 * Format to receive events
	 */
	public static final String P_FORMAT = "p_format";

	/**
	 * Protocol mode.
	 */
	public static final String P_MODE = "p_mode";

	/**
	 * Reason for errors.
	 */
	public static final String P_REASON = "p_reason";

	/**
	 * URL attribute.
	 */
	public static final String P_URL = "p_url";

	/**
	 * Wait attribute.
	 */
	public static final String P_WAIT = "p_wait";

	/**
	 * Subscription label, may be used to return user-specific
	 * token with a data event, e.g. the name of a function for a callback.
	 */
	public static final String P_SUBSCRIPTION_LABEL = "p_label";

	//
	// Event values with direction for P_EVENT (C=client, S=server)
	//

	/**
	 * C-->S Request to join server.
	 */
	public static final String E_JOIN = "join";

	/**
	 * S-->C Acknowledgement of join.
	 */
	public static final String E_JOIN_ACK = "join-ack";

	/**
	 * C-->S Request to join server.
	 */
	public static final String E_JOIN_LISTEN = "join-listen";

	/**
	 * S-->C Acknowledgement of join.
	 */
	public static final String E_JOIN_LISTEN_ACK = "join-listen-ack";

	/**
	 * C-->S Client starts listening.
	 */
	public static final String E_LISTEN = "listen";

	/**
	 * S-->C Ack of listen.
	 */
	public static final String E_LISTEN_ACK = "listen-ack";

	/**
	 * C-->S Client leaves server.
	 */
	public static final String E_LEAVE = "leave";

	/**
	 * S-->C Ack of leave.
	 */
	public static final String E_LEAVE_ACK = "leave-ack";

	/**
	 * C-->S Publish to subject.
	 */
	public static final String E_PUBLISH = "publish";

	/**
	 * S-->C Publish to subject acknowledge.
	 */
	public static final String E_PUBLISH_ACK = "publish-ack";

	/**
	 * C-->S Subscribe to subject request.
	 */
	public static final String E_SUBSCRIBE = "subscribe";

	/**
	 * S-->C Subscribe to subject acknowledge.
	 */
	public static final String E_SUBSCRIBE_ACK = "subscribe-ack";

	/**
	 * C-->S Unsubscribe from subject request.
	 */
	public static final String E_UNSUBSCRIBE = "unsubscribe";

	/**
	 * S--C Unsubscribe from subject acknowledge.
	 */
	public static final String E_UNSUBSCRIBE_ACK = "unsubscribe-ack";

	/**
	 * S-->C Client error response, transitional error.
	 */
	public static final String E_NACK = "nack";

	/**
	 * S-->C Client should abort, permanent error.
	 */
	public static final String E_ABORT = "abort";

	/**
	 * S-->C Data.
	 */
	public static final String E_DATA = "data";

	/**
	 * S-->C or C-->S Heartbeat.
	 */
	public static final String E_HEARTBEAT = "hb";

	/**
	 * S-->C S-->C or C-->S Heartbeat confirmed.
	 */
	public static final String E_HEARTBEAT_ACK = "hb-ack";

	/**
	 * S-->C or C-->S client refresh of data channel.
	 */
	public static final String E_REFRESH = "refresh";

	/**
	 * S-->C client should refresh data channel.
	 */
	public static final String E_REFRESH_ACK = "refresh-ack";

	//
	// Values for P_FORMAT parameter
	//

	/**
	 * JavaScript callback.
	 */
	public static String FORMAT_JAVASCRIPT = "js";

	/**
	 * Java serialized object.
	 */
	public static String FORMAT_SERIALIZED_JAVA_OBJECT = "ser";

	/**
	 * Stream of XML documents.
	 */
	public static String FORMAT_XML = "xml";

	/**
	 * Single XML document containing zero or more events.
	 */
	public static String FORMAT_XML_STRICT = "xml-strict";

	//
	// Values for P_MODE parameter
	//
	public static final String MODE_STREAM = "stream";
	public static final String MODE_PULL = "pull";
	public static final String MODE_POLL = "poll";

	//
	// Values for special/reserved subjects
	// TODO: use these to publish events when clients do these actions
	// TODO: Dispatcher may intercept these subjects to send cached events
	//
	public static final String SUBJECT_META = "/meta";
	public static final String SUBJECT_META_SUBS = SUBJECT_META + "/subs";
	public static final String SUBJECT_META_JOINS = SUBJECT_META + "/joins";


}

/*
  * $Log: Protocol.java,v $
  * Revision 1.15  2007/11/23 14:33:07  justb
  * core classes now configurable through factory
  *
  * Revision 1.14  2006/10/19 12:33:40  justb
  * add atomic join-listen support (one request)
  *
  * Revision 1.13  2005/05/06 19:44:00  justb
  * added xml-strict format
  *
  * Revision 1.12  2005/02/28 13:05:59  justb
  * introduced join-listen protocol service
  *
  * Revision 1.11  2005/02/28 12:45:59  justb
  * introduced Command class
  *
  * Revision 1.10  2005/02/16 12:16:17  justb
  * added support for "poll" mode
  *
  * Revision 1.9  2005/01/24 22:46:02  justb
  * getting safari to work
  *
  * Revision 1.8  2005/01/24 13:42:00  justb
  * new protocol changes (p_listen)
  *
  * Revision 1.7  2005/01/18 16:47:10  justb
  * protocol changes for v2 and publishing from pushlet client
  *
  * Revision 1.6  2005/01/13 14:47:15  justb
  * control evt: send response on same (control) connection
  *
  * Revision 1.5  2004/10/24 13:52:52  justb
  * small fixes in client lib
  *
  * Revision 1.4  2004/10/24 12:58:18  justb
  * revised client and test classes for new protocol
  *
  * Revision 1.3  2004/09/20 22:01:38  justb
  * more changes for new protocol
  *
  * Revision 1.2  2004/09/03 22:35:37  justb
  * Almost complete rewrite, just checking in now
  *
  * Revision 1.1  2004/09/03 21:02:20  justb
  * make more formalized protocol
  *
  *
  */
