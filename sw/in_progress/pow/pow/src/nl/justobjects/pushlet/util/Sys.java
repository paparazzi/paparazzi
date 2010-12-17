// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.util;


import java.io.FileInputStream;
import java.io.IOException;
import java.text.CharacterIterator;
import java.text.StringCharacterIterator;
import java.util.Properties;

/**
 * Utilities that interact with the underlying OS/JVM.
 *
 * @author Just van den Broecke
 * @version $Id: Sys.java,v 1.4 2007/11/10 14:17:18 justb Exp $
 */
public class Sys {

	/**
	 * Replace characters having special meaning <em>inside</em> HTML tags
	 * with their escaped equivalents, using character entities such as <tt>'&amp;'</tt>.
	 * <p/>
	 * <P>The escaped characters are :
	 * <ul>
	 * <li> <
	 * <li> >
	 * <li> "
	 * <li> '
	 * <li> \
	 * <li> &
	 * </ul>
	 * <p/>
	 * <P>This method ensures that arbitrary text appearing inside a tag does not "confuse"
	 * the tag. For example, <tt>HREF='Blah.do?Page=1&Sort=ASC'</tt>
	 * does not comply with strict HTML because of the ampersand, and should be changed to
	 * <tt>HREF='Blah.do?Page=1&amp;Sort=ASC'</tt>. This is commonly seen in building
	 * query strings. (In JSTL, the c:url tag performs this task automatically.)
	 */
	static public String forHTMLTag(String aTagFragment) {
		final StringBuffer result = new StringBuffer();

		final StringCharacterIterator iterator = new StringCharacterIterator(aTagFragment);
		char character = iterator.current();
		while (character != CharacterIterator.DONE) {
			if (character == '<') {
				result.append("&lt;");
			} else if (character == '>') {
				result.append("&gt;");
			} else if (character == '\"') {
				result.append("&quot;");
			} else if (character == '\'') {
				result.append("&#039;");
			} else if (character == '\\') {
				result.append("&#092;");
			} else if (character == '&') {
				result.append("&amp;");
			} else {
				//the char is not a special one
				//add it to the result as is
				result.append(character);
			}
			character = iterator.next();
		}
		return result.toString();
	}

	/**
	 * Load properties file from classpath.
	 */
	static public Properties loadPropertiesResource(String aResourcePath) throws IOException {
		try {
			// Use the class loader that loaded our class.
			// This is required where for reasons like security
			// multiple class loaders exist, e.g. BEA WebLogic.
			// Thanks to Lutz Lennemann 29-aug-2000.
			ClassLoader classLoader = Sys.class.getClassLoader();

			Properties properties = new Properties();

			// Try loading it.
			properties.load(classLoader.getResourceAsStream(aResourcePath));
			return properties;
		} catch (Throwable t) {
			throw new IOException("failed loading Properties resource from " + aResourcePath);
		}
	}

	/**
	 * Load properties file from file path.
	 */
	static public Properties loadPropertiesFile(String aFilePath) throws IOException {
		try {

			Properties properties = new Properties();

			// Try loading it.
			properties.load(new FileInputStream(aFilePath));
			return properties;
		} catch (Throwable t) {
			throw new IOException("failed loading Properties file from " + aFilePath);
		}
	}

	/**
	 * Shorthand for current time.
	 */
	static public long now() {
		return System.currentTimeMillis();
	}

}
