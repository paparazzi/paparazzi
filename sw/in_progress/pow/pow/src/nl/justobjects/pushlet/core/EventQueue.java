// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

/**
 * FIFO queue with guarded suspension.
 * <b>Purpose</b><br>
 * <p/>
 * <b>Implementation</b><br>
 * FIFO queue class implemented with circular array. The enQueue() and
 * deQueue() methods use guarded suspension according to a readers/writers
 * pattern, implemented with java.lang.Object.wait()/notify().
 * <p/>
 * <b>Examples</b><br>
 * <p/>
 * <br>
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: EventQueue.java,v 1.3 2007/11/23 14:33:07 justb Exp $
 */
public class EventQueue {
	/**
	 * Defines maximum queue size
	 */
	private int capacity = 8;
	private Event[] queue = null;
	private int front, rear;

	/**
	 * Construct queue with default (8) capacity.
	 */
	public EventQueue() {
		this(8);
	}

	/**
	 * Construct queue with specified capacity.
	 */
	public EventQueue(int capacity) {
		this.capacity = capacity;
		queue = new Event[capacity];
		front = rear = 0;
	}

	/**
	 * Put item in queue; waits() indefinitely if queue is full.
	 */
	public synchronized boolean enQueue(Event item) throws InterruptedException {
		return enQueue(item, -1);
	}

	/**
	 * Put item in queue; if full wait maxtime.
	 */
	public synchronized boolean enQueue(Event item, long maxWaitTime) throws InterruptedException {

		// Wait (optional maxtime) as long as the queue is full
		while (isFull()) {
			if (maxWaitTime > 0) {
				// Wait at most maximum time
				wait(maxWaitTime);

				// Timed out or woken; if still full we
				// had bad luck and return failure.
				if (isFull()) {
					return false;
				}
			} else {
				wait();
			}
		}

		// Put item in queue
		queue[rear] = item;
		rear = next(rear);

		// Wake up waiters; NOTE: first waiter will eat item
		notifyAll();
		return true;
	}

	/**
	 * Get head; if empty wait until something in queue.
	 */
	public synchronized Event deQueue() throws InterruptedException {
		return deQueue(-1);
	}

	/**
	 * Get head; if empty wait for specified time at max.
	 */
	public synchronized Event deQueue(long maxWaitTime) throws InterruptedException {
		while (isEmpty()) {
			if (maxWaitTime >= 0) {
				wait(maxWaitTime);

				// Timed out or woken; if still empty we
				// had bad luck and return failure.
				if (isEmpty()) {
					return null;
				}
			} else {
				// Wait indefinitely for something in queue.
				wait();
			}
		}

		// Dequeue item
		Event result = fetchNext();

		// Notify possible wait()-ing enQueue()-ers
		notifyAll();

		// Return dequeued item
		return result;
	}

	/**
	 * Get all queued Events.
	 */
	public synchronized Event[] deQueueAll(long maxWaitTime) throws InterruptedException {
		while (isEmpty()) {
			if (maxWaitTime >= 0) {
				wait(maxWaitTime);

				// Timed out or woken; if still empty we
				// had bad luck and return failure.
				if (isEmpty()) {
					return null;
				}
			} else {
				// Wait indefinitely for something in queue.
				wait();
			}
		}

		// Dequeue all items item
		Event[] events = new Event[getSize()];
		for (int i = 0; i < events.length; i++) {
			events[i] = fetchNext();
		}

		// Notify possible wait()-ing enQueue()-ers
		notifyAll();

		// Return dequeued item
		return events;
	}

	public synchronized int getSize() {
		return (rear >= front) ? (rear - front) : (capacity - front + rear);
	}

	/**
	 * Is the queue empty ?
	 */
	public synchronized boolean isEmpty() {
		return front == rear;
	}

	/**
	 * Is the queue full ?
	 */
	public synchronized boolean isFull() {
		return (next(rear) == front);
	}

	/**
	 * Circular counter.
	 */
	private int next(int index) {
		return (index + 1 < capacity ? index + 1 : 0);
	}

	/**
	 * Circular counter.
	 */
	private Event fetchNext() {
		Event temp = queue[front];
		queue[front] = null;
		front = next(front);
		return temp;
	}

	public static void p(String s) {
		System.out.println(s);
	}

	public static void main(String[] args) {
		EventQueue q = new EventQueue(8);
		Event event = new Event("t");
		try {
			q.enQueue(event);
			p("(1) size = " + q.getSize());
			q.enQueue(event);
			p("(2) size = " + q.getSize());
			q.deQueue();
			p("(1) size = " + q.getSize());
			q.deQueue();
			p("(0) size = " + q.getSize());

			q.enQueue(event);
			q.enQueue(event);
			q.enQueue(event);
			p("(3) size = " + q.getSize());
			q.deQueue();
			p("(2) size = " + q.getSize());
			q.enQueue(event);
			q.enQueue(event);
			q.enQueue(event);
			p("(5) size = " + q.getSize());
			q.enQueue(event);
			q.enQueue(event);
			p("(7) size = " + q.getSize());
			q.deQueue();
			q.deQueue();
			q.deQueue();
			p("(4) size = " + q.getSize());
			q.deQueue();
			q.deQueue();
			q.deQueue();
			;
			q.deQueue();
			p("(0) size = " + q.getSize());

			q.enQueue(event);
			q.enQueue(event);
			q.enQueue(event);
			q.enQueue(event);
			q.enQueue(event);
			p("(5) size = " + q.getSize());

			q.deQueue();
			q.deQueue();
			q.deQueue();
			;
			q.deQueue();
			p("(1) size = " + q.getSize());
		} catch (InterruptedException ie) {
		}
	}
}

/*
* $Log: EventQueue.java,v $
* Revision 1.3  2007/11/23 14:33:07  justb
* core classes now configurable through factory
*
* Revision 1.2  2005/02/21 11:50:46  justb
* ohase1 of refactoring Subscriber into Session/Controller/Subscriber
*
* Revision 1.1  2005/02/18 10:07:23  justb
* many renamings of classes (make names compact)
*
* Revision 1.6  2005/02/16 12:16:16  justb
* added support for "poll" mode
*
* Revision 1.5  2005/01/13 14:47:15  justb
* control evt: send response on same (control) connection
*
* Revision 1.4  2004/09/03 22:35:37  justb
* Almost complete rewrite, just checking in now
*
* Revision 1.3  2003/08/15 08:37:40  justb
* fix/add Copyright+LGPL file headers and footers
*
*
*/
