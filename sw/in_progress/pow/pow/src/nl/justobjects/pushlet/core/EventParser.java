// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import java.io.*;
import java.util.HashMap;

/**
 * Parses XML into Event objects.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: EventParser.java,v 1.3 2007/11/23 14:33:07 justb Exp $
 */
public class EventParser {


	private EventParser() {
	}

	/**
	 * Parse Event from a File.
	 */
	public static Event parse(File aFile) throws IOException {
		BufferedReader br = new BufferedReader(new FileReader(aFile));
		return parse(br);
	}

	/**
	 * Parse Event from input Reader.
	 */
	public static Event parse(Reader aReader) throws IOException {
		StringBuffer preparsedString = new StringBuffer(24);

		// First find the opening tag ('<')
		char nextChar;
		while ((nextChar = (char) aReader.read()) != '<') ;

		// Append '<'
		preparsedString.append(nextChar);

		// Then find end-tag ('>'), appending all chars to preparsed string.
		do {
			nextChar = (char) aReader.read();
			preparsedString.append(nextChar);
		} while (nextChar != '>');

		return parse(preparsedString.toString());
	}

	/**
	 * Parse Event from a String.
	 */
	public static Event parse(String aString) throws IOException {
		aString = aString.trim();

		if (!aString.startsWith("<") || !aString.endsWith("/>")) {
			throw new IOException("No start or end tag found while parsing event [" + aString + "]");
		}

		// Create the attributes object.
		HashMap properties = new HashMap(3);

		// Remove the start and end (< ... />) from the string
		aString = aString.substring(1, aString.length() - 2).trim();

		int index = 0;

		// Parse the tag
		while (!Character.isWhitespace(aString.charAt(index))
				&& (index < aString.length())) {
			index++;
		}

		// We don't use the tag: remove from string
		aString = aString.substring(index).trim();
		index = 0;

		String attrName;
		String attrValue;

		while (index < aString.length()) {

			// Parse attribute name
			while ((aString.charAt(index) != '=')
					&& (index < aString.length())) {
				index++;
			}

			// Create attr name string
			attrName = aString.substring(0, index).trim();

			// remove the attributeName and the '=' from the string
			aString = aString.substring(index + 1).trim();
			index = 1;	// read past the first wrapping "\""

			// Parse attribute value
			while ((aString.charAt(index) != '\"')
					&& (index < aString.length())) {

				// bypass the special characters '\' and '"' inside the
				// attributevalue itself which are deliniated with a preceding
				// '\'
				if (aString.charAt(index) == '\\') {
					aString = aString.substring(0, index)
							+ aString.substring(index + 1);	// remove the '\'
				}

				index++;
			}

			// create the attribute value; exclude the wrapping quote-characters
			attrValue = aString.substring(1, index);

			// Set the attribute N/V
			properties.put(attrName, attrValue);

			aString = aString.substring(index + 1).trim();
			index = 0;
		}

		return new Event(properties);
	}

	/**
	 * Test method: use files to test.
	 */
	public static void main(String[] args) {
		try {
			Event event = parse(new File(args[0]));
			System.out.println("OK parsed Event file " + args[0]);
			System.out.println(event.toXML());

			event = parse(event.toXML());
			System.out.println("OK parsed Event string");
			System.out.println(event.toXML());
		} catch (Throwable t) {
			System.out.println("Error parsing event file: " + args[0]);
			t.printStackTrace();
		}
	}
}

/*
  * $Log: EventParser.java,v $
  * Revision 1.3  2007/11/23 14:33:07  justb
  * core classes now configurable through factory
  *
  * Revision 1.2  2006/05/06 00:10:11  justb
  * various chgs but not too serious...
  *
  * Revision 1.1  2005/02/18 10:07:23  justb
  * many renamings of classes (make names compact)
  *
  * Revision 1.3  2004/09/03 22:35:37  justb
  * Almost complete rewrite, just checking in now
  *
  * Revision 1.2  2003/08/15 08:37:40  justb
  * fix/add Copyright+LGPL file headers and footers
  *
  * Revision 1.1  2003/05/18 16:12:27  justb
  * adding support for XML encoded Events
  *
  */
