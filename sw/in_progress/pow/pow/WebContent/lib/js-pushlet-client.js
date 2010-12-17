/*
 * Pushlet JS client library.
 * NOTE: this should replace the js-pushlet-client.jsp file
 * (since we can figure out pushletWebRoot from within JS)
 *
 * $Id: js-pushlet-client.js,v 1.2 2007/11/10 14:17:18 justb Exp $
 */
var flag = false;
var pushletWebRoot = null;
var pushletURI;
var pushletNetURI;
var sessionId = null;
var controlQueue = new Queue(20);
var statusMsg = 'null';
var statusChanged = false;
var statusChar = '|';
var pushletNet;
var listenMode = null;
var listenSubject = null;

// Initialize various URLs
_initURIs();

/************ Public application functions ******************/

// Embed pushlet frame in page
function p_embed(thePushletWebRoot) {
	if (thePushletWebRoot) {
		// Use this when webapp is not in same server e.g. with virtual hosts
		pushletWebRoot = thePushletWebRoot;
		_initURIs();
	}
//alert(pushletNetURI+"");
	p_debug(flag, "p_embed", 'write ' + pushletLayer);
	var pushletLayer = '<iframe id="pushletFrame" name="pushletFrame" src="' + pushletNetURI + '" style="visibility: hidden; width: 0px; height: 0px; border: 0px;"></iframe>';

	self.document.write(pushletLayer);
	_setStatus('initializing...');
	_showStatus();
	_waitForPushletFrame();
}

// Join the pushlet server
function p_join() {
	// Ignore (for now) if already joined
	if (sessionId != null) {
		return;
	}

	_setStatus('connecting...');
	p_debug(flag, "p_join", 'joining..');


  // Check if pushlet frame is loaded
	if (pushletNet) {
		// Pushlet iframe is ready for calls
		pushletNet.setControlURI(pushletURI + '?p_event=join');
	} else {

		// Pushlet net iframe not loaded: continue waiting
		setTimeout("p_join()", 100);
	}
}

// Create data event channel with the server
function p_listen(subject, mode) {
	// Optional initial subject to subscribe to
	if (subject) {
		// Remember
		listenSubject = subject;
	}

  // Optional mode (stream, pull, poll) i.s.o. default
	if (mode) {
		// Remember
		listenMode = mode;
	}

  // Loop forever as long not joined
	if (sessionId) {
		// ok we have joined

		// Create event URI for listen
		var uri = pushletURI + '?p_id=' + sessionId + '&p_event=listen';

    // Optional subject to subscribe to
		if (listenSubject) {
			uri = uri + '&p_subject=' + listenSubject;
		}

    // Optional mode (stream, pull, poll) i.s.o. default
		if (listenMode) {
			uri = uri + '&p_mode=' + listenMode;
		}

		pushletNet.listen(uri);
		return;
	}

  // No join ack (sessionId) yet: wait until received
	setTimeout("p_listen()", 100);

}

// Shorthand: Join the pushlet server and start listening immediately
function p_join_listen(subject, mode) {
	p_join();
	p_listen(subject, mode);
}

// Leave the pushlet server
function p_leave() {
	// Ignore (for now) if already left
	if (sessionId == null) {
		return;
	}

	var uri = pushletURI + '?p_event=leave';
	p_debug(flag, 'p_leave', 'leave uri=' + uri);
	_sendControlURI(uri);
}


// Send heartbeat event; callback is onHeartbeatAck()
function p_heartbeat() {
	var uri = pushletURI + '?p_event=hb';
	p_debug(flag, 'p_heartbeat');
	_sendControlURI(uri);
}

// Publish to a subject
function p_publish(subject, nvPairs) {
	if (!subject) {
		return false;
	}

	var uri = pushletURI + '?p_event=publish&p_subject=' + subject;

	var args = p_publish.arguments;

  // Put the arguments' name/value pairs in the URI
	for (var i = 1; i < args.length; i++) {
		uri = uri + '&' + args[i] + '=' + args[++i];
	}

	p_debug(false, 'p_publish', 'publish uri=' + uri);
	_sendControlURI(uri);
}

// Subscribe to a subject with optional label
function p_subscribe(subject, label) {
	var uri = pushletURI + '?p_event=subscribe&p_subject=' + subject;
	if (label) {
		uri = uri + '&p_label=' + label;
	}

	p_debug(flag, 'p_subscribe', 'subscribe uri=' + uri);
	_sendControlURI(uri);
}

 // Unsubscribe from a subject
function p_unsubscribe(subscriptionId) {
	var uri = pushletURI + '?p_event=unsubscribe';

	if (subscriptionId) {
		uri = uri + '&p_sid=' + subscriptionId;
	}

	p_debug(flag, 'p_unsubscribe', 'unsubscribe uri=' + uri);
	_sendControlURI(uri);
}


// Get webroot for this webapp
function p_getWebRoot() {
	return pushletWebRoot;
}

// Get pushlet session id
function p_getSessionId() {
	return sessionId;
}

// Show debug window
function p_setDebug(aFlag) {
	flag = aFlag;
	p_setNetDebug(aFlag);
}

// Show network debug window
function p_setNetDebug(aFlag) {
	pushletNet.p_setDebug(aFlag);
}

/************ Private functions ******************/

/** CALLBACKS FROM pushletFrame ***/

// Generic callback from server; this function is called from within the
// Pushlet subscriber frame (see frames below).
function _push(args) {
	// Create a PushletEvent object from the arguments passed in
	// push.arguments is event data coming from the Server
	var event = new PushletEvent(args);

	p_debug(flag, '_push() from server: ', event.toString());

  // Do action based on event type
	var eventType = event.getEvent();

	if (eventType == 'data') {
		_setStatus('data');
		_doCallback(event, window.onData);
	} else if (eventType == 'join-ack') {
		sessionId = event.get('p_id');
		_setStatus('connected');
		_doCallback(event, window.onJoinAck);
	} else if (eventType == 'listen-ack') {
		_setStatus('listening');
		_doCallback(event, window.onListenAck);

	// Send empty heartbeat event. This
		// silences many busy browser windows.
		// At least in Moz and IE.
		p_heartbeat();
	} else if (eventType == 'hb') {
		_setStatus('heartbeat');
		_doCallback(event, window.onHeartbeat);
	} else if (eventType == 'hb-ack') {
		_doCallback(event, window.onHeartbeatAck);
	} else if (eventType == 'leave-ack') {
		sessionId = null;
		_setStatus('disconnected');
		_doCallback(event, window.onLeaveAck);
	} else if (eventType == 'refresh-ack') {
		_doCallback(event, window.onRefreshAck);
	} else if (eventType == 'subscribe-ack') {
		_setStatus('subscribed to ' + event.get('p_subject'));
		_doCallback(event, window.onSubscribeAck);
	} else if (eventType == 'unsubscribe-ack') {
		_setStatus('unsubscribed');
		_doCallback(event, window.onUnsubscribeAck);
	} else if (eventType == 'abort') {
		_setStatus('abort');
		_doCallback(event, window.onAbort);
	} else if (eventType.match(/nack$/)) {
		_setStatus('error response: ' + event.get('p_reason'));
		_doCallback(event, window.onNack);
	}
}

function getWebRoot() {
	/** Return directory of this relative to document URL. */
	if (pushletWebRoot != null) {
		return pushletWebRoot;
	}
	//derive the baseDir value by looking for the script tag that loaded this file
	var head = document.getElementsByTagName('head')[0];
	var nodes = head.childNodes;
	for (var i = 0; i < nodes.length; ++i) {
		var src = nodes.item(i).src;
		if (src) {
			var index = src.indexOf("lib/js-pushlet-client.js");
			if (index >= 0) {
				pushletWebRoot = src.substring(0, index);
				break;
			}
		}
	}
	return pushletWebRoot;
}

function _initURIs() {
	pushletURI = getWebRoot() + 'pushlet.srv';
	pushletNetURI = getWebRoot() + 'lib/js-pushlet-net.html';
	//alert(pushletURI);
//	alert(pushletNetURI);
}

function _showStatus() {
	// To show progress
	if (statusChanged == true) {
		if (statusChar == '|') statusChar = '/';
		else if (statusChar == '/') statusChar = '--';
		else if (statusChar == '--') statusChar = '\\';
		else statusChar = '|';
		statusChanged = false;
	}

	window.defaultStatus = statusMsg;
	window.status = statusMsg + '  ' + statusChar;
	timeout = window.setTimeout('_showStatus()', 400);
}

function _setStatus(status) {
	statusMsg = "pushlet - " + status;
	statusChanged = true;
}

function _onUnload() {
	p_debug(true, "pushlet-lib", "_onUnload() called");
}

function _onBeforeUnload() {
	p_debug(true, "pushlet-lib", "_onBeforeUnload() called");
}

function _onStop() {
	p_debug(true, "pushlet-lib", "_onStop() called");
}

function _doCallback(event, cbFunction) {
	// Do specific callback function if provided by client
	if (cbFunction) {
		// Do specific callback like onData(), onJoinAck() etc.
		cbFunction(event);
	} else if (window.onEvent) {
		// general callback onEvent() provided to catch all events
		onEvent(event);
	}
}

/** CALLS TO pushletFrame ***/


function _sendControlURI(uri) {
	if (controlQueue.isFull()) {
		// TODO divert to errpage
		alert('serious problem: control queue is full');
    // no sense going on
		return;
	}

	if (sessionId == null) {
		controlQueue.enqueue(uri);
		_processControlQueue();
		return;
	}

    // All clear to send immediately ?
	if (controlQueue.isEmpty()) {
		if (pushletNet.isControlReady()) {
			// Ok send direct
			uri = uri + '&p_id=' + sessionId;
			pushletNet.setControlURI(uri);
		} else {
			controlQueue.enqueue(uri);
		}
	} else {
		// Queue not empty
		controlQueue.enqueue(uri);
	}

	if (!controlQueue.isEmpty()) {
		_processControlQueue();
	}
}

function _processControlQueue() {
	if (controlQueue.isEmpty()) {
		// all done
		return;
	}

	if (sessionId != null) {
		// Dequeue next control URI if pushletFrame ready
		if (pushletNet.isControlReady()) {
			var uri = controlQueue.dequeue() + '&p_id=' + sessionId;
			pushletNet.setControlURI(uri);
		}
	}

  // Loop forever as long queue is not empty
	if (!controlQueue.isEmpty()) {
		setTimeout("_processControlQueue()", 50);
	}
}


function _waitForPushletFrame() {
	// Loop forever as long net uri not ready
	if (self.pushletFrame && self.pushletFrame.isLoaded && self.pushletFrame.isLoaded()) {
		_setStatus('loaded pushlet frame...');

		pushletNet = self.pushletFrame;
		return;
	}

	_setStatus('pushlet frame not ready');
	setTimeout("_waitForPushletFrame()", 20);
}

/************** Util classes *******************************/

/** NV pair object */
function NameValuePair(name, value) {
	this.name = name;
	this.value = value;
}

/** Simple Map object to store array of name/value pairs */
function Map() {
	// Data members
	this.index = 0;
	this.map = new Array();

   // Function members
	this.get = MapGet;
	this.put = MapPut;
	this.toString = MapToString;
	this.toTable = MapToTable;
}

/** get() */
function MapGet(name) {
	for (var i = 0; i < this.index; i++) {
		if (this.map[i].name == name) {
			return this.map[i].value;
		}
	}
	return '';
}

/** put() */
function MapPut(name, value) {
	this.map[this.index++] = new NameValuePair(name, value);
}

/** To HTML string */
function MapToString() {
	var res = '';

	for (var i = 0; i < this.index; i++) {
		res = res + this.map[i].name + '=' + this.map[i].value + '\n';
	}
	return res;
}

/** To HTML table */
function MapToTable() {
	var res = '<table border=1 cellpadding=3>';
	var styleDiv = '<div style="color:black; font-family:monospace; font-size:10pt; white-space:pre;">'

	for (var i = 0; i < this.index; i++) {
		res = res + '<tr><td bgColor=white>' + styleDiv + this.map[i].name + '</div></td><td bgColor=white>' + styleDiv + this.map[i].value + '</div></td></tr>';
	}
	res += '</table>'
	return res;
}

/** Simple FIFO Queue class */
function Queue(aCapacity) {
	// Data members
	this.capacity = aCapacity;
	this.size = 0;
	this.arr = new Array();
	this.front = 0;
	this.rear = 0;

   // Function members
	this.dequeue = QueueDequeue;
	this.enqueue = QueueEnqueue;
	this.next = QueueNext;
	this.isFull = QueueIsFull;
	this.isEmpty = QueueIsEmpty;
}

/** enqueue() */
function QueueEnqueue(item) {
	if (this.isFull()) {
		alert('queue full !!');
		return;
	}

	this.arr[this.rear] = item;
	this.rear = this.next(this.rear);
	this.size++;
}

/** dequeue() */
function QueueDequeue(name) {
	if (this.isEmpty()) {
		alert('queue empty !!');
		return;
	}
	var temp = this.arr[this.front];
	this.arr[this.front] = null;
	this.front = this.next(this.front);
	this.size--;
	return temp;
}

/** Circular counter. */
function QueueNext(index) {
	return (index + 1 < this.capacity ? index + 1 : 0);
}

function QueueIsFull() {
	return this.size == this.capacity;
}

function QueueIsEmpty() {
	return this.size == 0;
}

/* Class to represent nl.justobjects.pushlet.Event in JavaScript.
   Arguments are an array where args[i] is name and args[i+1] is value
*/
function PushletEvent(args) {
	// Member variable setup; the Map stores the N/V pairs
	this.map = new Map();

   // Member function setup
	this.getSubject = PushletEventGetSubject
	this.getEvent = PushletEventGetEvent
	this.put = PushletEventPut
	this.get = PushletEventGet
	this.toString = PushletEventToString
	this.toTable = PushletEventToTable

   // Put the arguments' name/value pairs in the Map
	for (var i = 0; i < args.length; i++) {
		this.put(args[i], args[++i]);
	}
}

// Get the subject attribute
function PushletEventGetSubject() {
	return this.map.get('p_subject')
}

// Get the subject attribute
function PushletEventGetEvent() {
	return this.map.get('p_event')
}

// Get event attribute
function PushletEventGet(name) {
	return this.map.get(name)
}

// Put event attribute
function PushletEventPut(name, value) {
	return this.map.put(name, value)
}

function PushletEventToString() {
	return this.map.toString();
}

// Convert content to HTML TABLE
function PushletEventToTable() {
	return this.map.toTable();
}

/*************** Debug utility *******************************/
var timestamp = 0
var debugWindow = null
var messages = new Array()
var messagesIndex = 0

/** Send debug messages to a (D)HTML window. */
function p_debug(flag, label, value) {

	// Only print if the flag is set
	if (!flag) {
		return;
	}

	var funcName = "none";

      // Fetch JS function name if any
	if (p_debug.caller) {
		funcName = p_debug.caller.toString()
		funcName = funcName.substring(9, funcName.indexOf(")") + 1)
	}

      // Create message
	var msg = "-" + funcName + ": " + label + "=" + value

      // Add optional timestamp
	var now = new Date()
	var elapsed = now - timestamp
	if (elapsed < 10000) {
		msg += " (" + elapsed + " msec)"
	}

	timestamp = now

    // Show.

	if ((debugWindow == null) || debugWindow.closed) {
		debugWindow = window.open("", "p_debugWin", "toolbar=no,scrollbars=yes,resizable=yes,width=600,height=400");
	}

  // Add message to current list
	messages[messagesIndex++] = msg

  // Write doc header
	debugWindow.document.writeln('<html><head><title>Pushlet Debug Window</title></head><body bgcolor=#DDDDDD>');

    // Write the messages
	for (var i = 0; i < messagesIndex; i++) {
		debugWindow.document.writeln('<pre>' + i + ': ' + messages[i] + '</pre>');
	}

  // Write doc footer and close
	debugWindow.document.writeln('</body></html>');
	debugWindow.document.close();
	debugWindow.focus();

}
