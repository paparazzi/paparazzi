// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.test;

import nl.justobjects.pushlet.core.Dispatcher;
import nl.justobjects.pushlet.core.Event;
import nl.justobjects.pushlet.core.EventSource;
import nl.justobjects.pushlet.util.Rand;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.util.StringTokenizer;
import java.util.Vector;

/**
 * Event sources that push events (for testing).
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: TestEventPushSources.java,v 1.10 2007/11/09 13:16:57 justb Exp $
 */
public class TestEventPushSources {

	static public class AEXStocksEventPushSourceABN {
		String pageURL = "http://ri2.rois.com/E36msPtnZC0e15CVb4KT97JAGfGSfCcrvv6*FcyZIoNyh/CTIB/RI2APISNAP?RIC=0%23.AEX&FORMAT=XML";
		// This could be further expanded: getting the Reuters AEX stocks
		// as XML from ABN with this URL, but we may get into legal problems...
	}

	/**
	 * Produces events from REAL stocks from the AEX.
	 */
	static public class AEXStocksEventPushSource implements EventSource, Runnable {
		/**
		 * Here we get our stocks from.
		 */
		String pageURL = "http://www.debeurs.nl/koersen/aex.asp";
		Thread thread = null;
		volatile boolean active = false;

		// Since Baan has been thrown out...
		public final static int NR_OF_STOCKS = 24;

		public final static String EMPTY = "wait...";
		private int restarts = 1;

		class Stock {
			public String name = EMPTY;
			public String rate = EMPTY;
			volatile public boolean modified = false;
		}

		Vector stocksCache = new Vector(NR_OF_STOCKS);

		public AEXStocksEventPushSource() {
			for (int i = 0; i < NR_OF_STOCKS; i++) {
				stocksCache.addElement(new Stock());
			}
			// updateCache();
		}

		/**
		 * Activate the event source.
		 */
		synchronized public void activate() {
			e("activating...");
			// Stop a possibly running thread
			stopThread();

			// Start new thread and
			thread = new Thread(this, "AEXStocksPublisher-" + (restarts++));
			active = true;
			thread.start();
			e("activated");
		}

		/**
		 * Deactivate the event source.
		 */
		synchronized public void passivate() {
			e("passivating...");
			active = false;
			stopThread();

			// Mark the cache modified so we'll send the contents
			// on the next activation.
			for (int i = 0; i < NR_OF_STOCKS; i++) {
				((Stock) stocksCache.elementAt(i)).modified = true;
			}

			e("passivated");
		}


		/**
		 * Deactivate the event source.
		 */
		synchronized public void stop() {
		}

		public void run() {
			// Publish cache content (if any) first.
			publishStocks();

			int count = 5; // enforce update first
			while (active) {

				// Only do work if active
				// Update cache every 10 secs.
				if (count == 5) {
					updateCache();
					count = 0;
				}
				count++;

				// Do updates for changed stock rates
				sendUpdates();

				// If we were interrupted just return.
				if (thread == null || thread.isInterrupted()) {
					break;
				}

				// Sleep 2 secs before sending next updates
				try {
					thread.sleep(2000);
				} catch (InterruptedException ie) {
					break;
				}
			}

			// Loop terminated: reset vars
			thread = null;
			active = false;
		}

		private String getStocksLine() {
			BufferedReader br = null;
			InputStream is = null;
			String nextLine = "";

			// Read line from server
			try {
				is = new URL(pageURL).openStream();
				br = new BufferedReader(new InputStreamReader(is));
				boolean foundLine = false;
				while (!foundLine) {
					nextLine = br.readLine();
					if (nextLine == null) {
						return "";
					}
					foundLine = (nextLine.indexOf("details.asp?iid=14053&parent=aex") != -1);
				}
			} catch (Exception e) {
				e("could not open or read URL pageURL=" + pageURL + " ex=" + e);
				return "";
			} finally {
				try {
					if (is != null) is.close();
				} catch (IOException ignore) {
				}
			}
			return nextLine;
		}

		private void publishStocks() {
			// Publish only modified stocks from the cache
			for (int i = 0; i < NR_OF_STOCKS; i++) {
				Stock nextStock = (Stock) stocksCache.elementAt(i);

				// Publish modified stocks
				if (nextStock.modified) {
					publishStock(i, nextStock.name, nextStock.rate);
					nextStock.modified = false;
					try {
						Thread.sleep(400);
					} catch (InterruptedException ie) {
						return;
					}
				}
			}
		}

		private void publishStock(int index, String name, String rate) {
			Event event = Event.createDataEvent("/stocks/aex");
			event.setField("number", index + "");
			event.setField("name", name);
			event.setField("rate", rate);
			p("publish: nr=" + index + " name=" + name + " rate=" + rate);
			Dispatcher.getInstance().multicast(event);
		}

		private void sendUpdates() {
			p("sending updates");
			// In any case send a random stock value by
			// making it modified, just to see something moving...
			int randomIndex = Rand.randomInt(0, NR_OF_STOCKS - 1);
			Stock randomStock = (Stock) stocksCache.elementAt(randomIndex);
			randomStock.modified = true;

			publishStocks();
		}

		private void stopThread() {
			if (thread != null) {
				thread.interrupt();
				thread = null;
			}
		}

		private void updateCache() {
			p("updating Cache");

			// Get the line with all stocks from HTML page
			String stocksLine = getStocksLine();
			if ("".equals(stocksLine)) {
				e("updateCache: stocksLine == null");
				return;
			}

			// Parse the stocksline and put in cache.
			// Beware: this is the messy part!!
			// We assume that stock/names and rates are located at
			// regular positions in the line.
			String delim = "<>";
			StringTokenizer st = new StringTokenizer(stocksLine, delim);
			String nextToken = "";
			int count = 0;
			String nextStock = "";
			String nextQuote = "";
			String currentQuote = null;
			int index = -1;
			while (st.hasMoreTokens()) {
				nextToken = st.nextToken();
				count++;
				// The <TD> with the stock name
				if ((count - 5) % 57 == 0) {
					p("c=" + count + " s=" + nextToken);
					nextStock = nextToken;
				}

				// The <TD> with the stock rate
				if ((count - 10) % 57 == 0) {
					nextQuote = nextToken;
					index++;
					p("c=" + count + " val=" + nextQuote);
					Stock currentStock = (Stock) stocksCache.elementAt(index);

					// Only update new or modified stocks
					if (EMPTY.equals(currentStock.rate) || !currentStock.rate.equals(nextQuote)) {
						p("modified: " + nextStock);
						currentStock.name = nextStock;
						currentStock.rate = nextQuote;
						currentStock.modified = true;
					}
				}
			}
		}
	}


	/**
	 * Util: stderr print method.
	 */
	public static void e(String s) {
		System.out.println("AEXStocksEventPushSource: " + s);
	}

	/**
	 * Util: stdout print method.
	 */
	public static void p(String s) {
		// System.out.println(s);
	}


	public static void main(String[] args) {
		// new TestEventPushSources$AEXStocksEventPushSource();
	}
}

/*
 * $Log: TestEventPushSources.java,v $
 * Revision 1.10  2007/11/09 13:16:57  justb
 * use Rand from util package (and and Rand.java to pushlet client jar
 *
 * Revision 1.9  2005/02/28 09:14:56  justb
 * sessmgr/dispatcher factory/singleton support
 *
 * Revision 1.8  2005/02/21 16:59:17  justb
 * SessionManager and session lease introduced
 *
 * Revision 1.7  2005/02/18 10:07:23  justb
 * many renamings of classes (make names compact)
 *
 * Revision 1.6  2005/02/18 09:54:15  justb
 * refactor: rename Publisher Dispatcher and single Subscriber class
 *
 * Revision 1.5  2004/09/03 22:35:37  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.4  2004/03/10 15:45:55  justb
 * many cosmetic changes
 *
 * Revision 1.3  2003/08/15 08:37:41  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.2  2003/05/18 16:15:08  justb
 * support for XML encoded Events
 *
 * Revision 1.1.1.1  2002/09/24 21:02:33  justb
 * import to sourceforge
 *
 * Revision 1.1.1.1  2002/09/20 22:48:20  justb
 * import to SF
 *
 * Revision 1.1.1.1  2002/09/20 14:19:02  justb
 * first import into SF
 *
 * Revision 1.6  2001/02/18 23:45:13  just
 * fixes for AEX
 *
 * Revision 1.5  2000/10/30 14:16:09  just
 * no message
 *
 * Revision 1.4  2000/09/24 21:02:43  just
 * chnages due to changed webpage in debeurs.nl
 *
 * Revision 1.3  2000/08/31 12:49:50  just
 * added CVS comment tags for log and copyright
 *
 *
 */
