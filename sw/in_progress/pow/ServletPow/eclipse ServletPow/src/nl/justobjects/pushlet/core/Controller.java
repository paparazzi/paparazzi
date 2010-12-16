// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.PushletException;

import java.io.IOException;

/**
 * Handles servlet requests from client.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Controller.java,v 1.9 2007/11/23 14:33:07 justb Exp $
 */
public class Controller implements Protocol, ConfigDefs {

	private Session session;

	/**
	 * Protected constructor as we create through factory method.
	 */
	protected Controller() {
	}

	/**
	 * Create instance through factory method.
	 *
	 * @param aSession the parent Session
	 * @return a Controller object (or derived)
	 * @throws PushletException exception, usually misconfiguration
	 */
	public static Controller create(Session aSession) throws PushletException {
		Controller controller;
		try {
			controller = (Controller) Config.getClass(CONTROLLER_CLASS, "nl.justobjects.pushlet.core.Controller").newInstance();
		} catch (Throwable t) {
			throw new PushletException("Cannot instantiate Controller from config", t);
		}
		controller.session = aSession;
		return controller;
	}

	/**
	 * Handle command.
	 */
	public void doCommand(Command aCommand) {
		try {
			// Update lease time to live
			session.kick();

			// Set remote IP address of client
			session.setAddress(aCommand.httpReq.getRemoteAddr());

			debug("doCommand() event=" + aCommand.reqEvent);

			// Get event type
			String eventType = aCommand.reqEvent.getEventType();

			// Determine action based on event type
			if (eventType.equals(Protocol.E_REFRESH)) {
				// Pull/poll mode clients that refresh
				doRefresh(aCommand);
			} else if (eventType.equals(Protocol.E_SUBSCRIBE)) {
				// Subscribe
				doSubscribe(aCommand);
			} else if (eventType.equals(Protocol.E_UNSUBSCRIBE)) {
				// Unsubscribe
				doUnsubscribe(aCommand);
			} else if (eventType.equals(Protocol.E_JOIN)) {
				// Join
				doJoin(aCommand);
			} else if (eventType.equals(Protocol.E_JOIN_LISTEN)) {
				// Join and listen (for simple and e.g. REST apps)
				doJoinListen(aCommand);
			} else if (eventType.equals(Protocol.E_LEAVE)) {
				// Leave
				doLeave(aCommand);
			} else if (eventType.equals(Protocol.E_HEARTBEAT)) {
				// Heartbeat mainly to do away with browser "busy" cursor
				doHeartbeat(aCommand);
			} else if (eventType.equals(Protocol.E_PUBLISH)) {
				// Publish event
				doPublish(aCommand);
			} else if (eventType.equals(Protocol.E_LISTEN)) {
				// Listen to pushed events
				doListen(aCommand);
			}

			// Handle response back to client
			if (eventType.endsWith(Protocol.E_LISTEN) ||
					eventType.equals(Protocol.E_REFRESH)) {
				// Data channel events
				// Loops until refresh or connection closed
				getSubscriber().fetchEvents(aCommand);

			} else {
				// Send response for control commands
				sendControlResponse(aCommand);
			}

		} catch (Throwable t) {
			warn("Exception in doCommand(): " + t);
			t.printStackTrace();
		}
	}

	public String toString() {
		return session.toString();
	}

	/**
	 * Handle heartbeat event.
	 */
	protected void doHeartbeat(Command aCommand) {

		// Set heartbeat acknowledgement to client
		aCommand.setResponseEvent(new Event(E_HEARTBEAT_ACK));
	}

	/**
	 * Handle Join request.
	 */
	protected void doJoin(Command aCommand) throws PushletException {

		Event responseEvent = null;

		try {

			session.start();

			// Determine format for encoding Events to client.
			// Default assume a userAgent window on the other end.
			String format = aCommand.reqEvent.getField(P_FORMAT, FORMAT_JAVASCRIPT);

			session.setFormat(format);
			responseEvent = new Event(E_JOIN_ACK);

			// Set unique subscriber id and encoding format
			responseEvent.setField(P_ID, session.getId());
			responseEvent.setField(P_FORMAT, format);
			info("joined");
		} catch (Throwable t) {
			session.stop();
			responseEvent = new Event(E_NACK);
			responseEvent.setField(P_ID, session.getId());
			responseEvent.setField(P_REASON, "unexpected error: " + t);
			warn("doJoin() error: " + t);
			t.printStackTrace();
		} finally {
			// Always set response event in command
			aCommand.setResponseEvent(responseEvent);
		}

	}

	/**
	 * Handle JoinListen request.
	 */
	protected void doJoinListen(Command aCommand) throws PushletException {

		// Basically bundles a join and a listen
		// This request is handly for simple apps that
		// need to do a single request to get events immediately
		// For example in RESTful apps.

		// First do regular join
		doJoin(aCommand);
		if (!aCommand.getResponseEvent().getEventType().equals(E_NACK)) {
			// If successful do the listen
			doListen(aCommand);
			if (!aCommand.getResponseEvent().getEventType().equals(E_NACK)) {
				// If still ok do the listen ack
				aCommand.getResponseEvent().setField(P_EVENT, E_JOIN_LISTEN_ACK);
			}
		}
	}

	/**
	 * Handle Leave request.
	 */
	protected void doLeave(Command aCommand) throws IOException {

		Event responseEvent = null;

		try {
			// Also removes all subscriptions
			session.stop();

			// Prepare acknowledgement
			responseEvent = new Event(E_LEAVE_ACK);

			// Set unique subscriber id
			responseEvent.setField(P_ID, session.getId());
			info("left");
		} catch (Throwable t) {
			responseEvent = new Event(E_NACK);
			responseEvent.setField(P_ID, session.getId());
			responseEvent.setField(P_REASON, "unexpected error: " + t);
			warn("doLeave() error: " + t);
			t.printStackTrace();
		} finally {
			// Always set response event in command
			aCommand.setResponseEvent(responseEvent);
		}

	}

	/**
	 * Handle Listen request.
	 */
	protected void doListen(Command aCommand) throws PushletException {


		String mode = MODE_STREAM;
		// Should we always force "pull" mode ?
		if (Config.getBoolProperty(LISTEN_FORCE_PULL_ALL)) {
			mode = MODE_PULL;
		} else {
			// Determine optimal mode determined by parameter and/or user agent
			// Mode param determines how events are transfered to the client

			// In "stream" mode, a stream of events is sent, i.e. the document
			// is neverending. In "pull" or "poll" mode a complete document is returned
			// ending with a request to refresh.
			mode = aCommand.reqEvent.getField(P_MODE, MODE_STREAM);

			String userAgent = aCommand.httpReq.getHeader("User-Agent");
			if (userAgent != null) {
				userAgent = userAgent.toLowerCase();
				for (int i = 0; i < session.FORCED_PULL_AGENTS.length; i++) {
					if ((userAgent.indexOf(session.FORCED_PULL_AGENTS[i]) != -1)) {
						info("Forcing pull mode for agent=" + userAgent);
						mode = MODE_PULL;
						break;
					}
				}
			} else {
				userAgent = "unknown";
			}
		}

		getSubscriber().setMode(mode);

		// Prepare acknowledgement
		Event listenAckEvent = new Event(E_LISTEN_ACK);

		// Add subscription(s) if subject(s) specified
		String subject = aCommand.reqEvent.getField(P_SUBJECT);
		if (subject != null) {
			// Optional label for subscription
			String label = aCommand.reqEvent.getField(Protocol.P_SUBSCRIPTION_LABEL);

			// Add a subscription
			Subscription subscription = getSubscriber().addSubscription(subject, label);

			// Add subscription id and optional label to listen-ack event
			listenAckEvent.setField(P_SUBSCRIPTION_ID, subscription.getId());
			if (label != null) {
				listenAckEvent.setField(P_SUBSCRIPTION_LABEL, label);
			}
		}

		// Set unique subscriber id, push mode and encoding format
		listenAckEvent.setField(P_ID, session.getId());
		listenAckEvent.setField(P_MODE, mode);
		listenAckEvent.setField(P_FORMAT, session.getFormat());

		// Activate the subscriber
		getSubscriber().start();

		// Enqueue listen ack event on data channel
		aCommand.setResponseEvent(listenAckEvent);

		info("Listening mode=" + mode + " userAgent=" + session.getUserAgent());

	}

	/**
	 * Handle Publish request.
	 */
	protected void doPublish(Command aCommand) {
		Event responseEvent = null;

		try {
			String subject = aCommand.reqEvent.getField(Protocol.P_SUBJECT);
			if (subject == null) {
				// Return error response
				responseEvent = new Event(E_NACK);
				responseEvent.setField(P_ID, session.getId());
				responseEvent.setField(P_REASON, "no subject provided");
			} else {
				aCommand.reqEvent.setField(P_FROM, session.getId());
				aCommand.reqEvent.setField(P_EVENT, E_DATA);

				// Event may be targeted to specific user (p_to field)
				String to = aCommand.reqEvent.getField(P_TO);
				if (to != null) {
					Dispatcher.getInstance().unicast(aCommand.reqEvent, to);
				} else {
					// No to: multicast
					debug("doPublish() event=" + aCommand.reqEvent);
					Dispatcher.getInstance().multicast(aCommand.reqEvent);
				}

				// Acknowledge
				responseEvent = new Event(E_PUBLISH_ACK);
			}

		} catch (Throwable t) {
			responseEvent = new Event(E_NACK);
			responseEvent.setField(P_ID, session.getId());
			responseEvent.setField(P_REASON, "unexpected error: " + t);
			warn("doPublish() error: " + t);
			t.printStackTrace();
		} finally {
			// Always set response event in command
			aCommand.setResponseEvent(responseEvent);
		}
	}

	/**
	 * Handle refresh event.
	 */
	protected void doRefresh(Command aCommand) {
		// Set ack
		aCommand.setResponseEvent(new Event(E_REFRESH_ACK));
	}

	/**
	 * Handle Subscribe request.
	 */
	protected void doSubscribe(Command aCommand) throws IOException {

		Event responseEvent = null;
		try {
			String subject = aCommand.reqEvent.getField(Protocol.P_SUBJECT);
			Subscription subscription = null;
			if (subject == null) {
				// Return error response
				responseEvent = new Event(E_NACK);
				responseEvent.setField(P_ID, session.getId());
				responseEvent.setField(P_REASON, "no subject provided");
			} else {

				String label = aCommand.reqEvent.getField(Protocol.P_SUBSCRIPTION_LABEL);
				subscription = getSubscriber().addSubscription(subject, label);

				// Acknowledge
				responseEvent = new Event(E_SUBSCRIBE_ACK);
				responseEvent.setField(P_ID, session.getId());
				responseEvent.setField(P_SUBJECT, subject);
				responseEvent.setField(P_SUBSCRIPTION_ID, subscription.getId());
				if (label != null) {
					responseEvent.setField(P_SUBSCRIPTION_LABEL, label);
				}
				info("subscribed to " + subject + " sid=" + subscription.getId());
			}

		} catch (Throwable t) {
			responseEvent = new Event(E_NACK);
			responseEvent.setField(P_ID, session.getId());
			responseEvent.setField(P_REASON, "unexpected error: " + t);
			warn("doSubscribe() error: " + t);
			t.printStackTrace();
		} finally {
			// Always set response event in command
			aCommand.setResponseEvent(responseEvent);
		}
	}

	/**
	 * Handle Unsubscribe request.
	 */
	protected void doUnsubscribe(Command aCommand) throws IOException {


		Event responseEvent = null;
		try {
			String subscriptionId = aCommand.reqEvent.getField(Protocol.P_SUBSCRIPTION_ID);
			if (subscriptionId == null) {
				// Unsuscbribe all
				getSubscriber().removeSubscriptions();
				responseEvent = new Event(E_UNSUBSCRIBE_ACK);
				responseEvent.setField(P_ID, session.getId());
				info("unsubscribed all");
			} else {
				// Subscription id provided: remove Subscription
				Subscription subscription = getSubscriber().removeSubscription(subscriptionId);
				if (subscription == null) {
					// Unknown subscription id: return error response
					responseEvent = new Event(E_NACK);
					responseEvent.setField(P_ID, session.getId());
					responseEvent.setField(P_REASON, "no subscription for sid=" + subscriptionId);
					warn("unsubscribe: no subscription for sid=" + subscriptionId);
				} else {
					// OK return ack
					responseEvent = new Event(E_UNSUBSCRIBE_ACK);
					responseEvent.setField(P_ID, session.getId());
					responseEvent.setField(P_SUBSCRIPTION_ID, subscription.getId());
					responseEvent.setField(P_SUBJECT, subscription.getSubject());
					if (subscription.getLabel() != null) {
						responseEvent.setField(P_SUBSCRIPTION_LABEL, subscription.getLabel());
					}
					info("unsubscribed sid= " + subscriptionId);
				}
			}
		} catch (Throwable t) {
			responseEvent = new Event(E_NACK);
			responseEvent.setField(P_ID, session.getId());
			responseEvent.setField(P_REASON, "unexpected error: " + t);
			warn("doUnsubscribe() error: " + t);
			t.printStackTrace();
		} finally {
			// Always set response event in command
			aCommand.setResponseEvent(responseEvent);
		}
	}

	public Subscriber getSubscriber() {
		return session.getSubscriber();
	}

	/**
	 * Send response on the control channel.
	 */
	protected void sendControlResponse(Command aCommand) {
		try {

			// Try to prevent caching in any form.
			aCommand.sendResponseHeaders();

			// Let clientAdapter determine how to send event
			aCommand.getClientAdapter().start();

			// Push to client through client adapter
			aCommand.getClientAdapter().push(aCommand.getResponseEvent());

			// One shot response
			aCommand.getClientAdapter().stop();
		} catch (Throwable t) {
			session.stop();
		}
	}


	/**
	 * Info.
	 */
	protected void info(String s) {
		session.info("[Controller] " + s);
	}

	/**
	 * Exceptional print util.
	 */
	protected void warn(String s) {
		session.warn("[Controller] " + s);
	}

	/**
	 * Exceptional print util.
	 */
	protected void debug(String s) {
		session.debug("[Controller] " + s);
	}


}

/*
 * $Log: Controller.java,v $
 * Revision 1.9  2007/11/23 14:33:07  justb
 * core classes now configurable through factory
 *
 * Revision 1.8  2005/02/28 15:58:05  justb
 * added SimpleListener example
 *
 * Revision 1.7  2005/02/28 13:05:59  justb
 * introduced join-listen protocol service
 *
 * Revision 1.6  2005/02/28 12:45:59  justb
 * introduced Command class
 *
 * Revision 1.5  2005/02/28 09:14:55  justb
 * sessmgr/dispatcher factory/singleton support
 *
 * Revision 1.4  2005/02/25 15:13:00  justb
 * session id generation more robust
 *
 * Revision 1.3  2005/02/21 16:59:06  justb
 * SessionManager and session lease introduced
 *
 * Revision 1.2  2005/02/21 12:32:28  justb
 * fixed publish event in Controller
 *
 * Revision 1.1  2005/02/21 11:50:46  justb
 * ohase1 of refactoring Subscriber into Session/Controller/Subscriber
 *

 *
 */
