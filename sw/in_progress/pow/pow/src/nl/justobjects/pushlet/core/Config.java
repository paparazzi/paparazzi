// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet.core;

import nl.justobjects.pushlet.util.Log;
import nl.justobjects.pushlet.util.PushletException;
import nl.justobjects.pushlet.util.Sys;

import java.io.File;
import java.util.Properties;

/**
 * Loads and maintains overall configuration.
 *
 * @author Just van den Broecke - Just Objects &copy;
 * @version $Id: Config.java,v 1.5 2007/11/23 21:10:17 justb Exp $
 */
public class Config implements ConfigDefs {
	private static final String PROPERTIES_FILE = "pushlet.properties";
	private static Properties properties;

	/**
	 * Factory method: create object from property denoting class name.
	 *
	 * @param aClassNameProp property name e.g. "session.class"
	 * @return an instance of class denoted by property
	 * @throws PushletException when class cannot be instantiated
	 */
	public static Object createObject(String aClassNameProp, String aDefault) throws PushletException {
		Class clazz = getClass(aClassNameProp, aDefault);
		try {
			return clazz.newInstance();
		} catch (Throwable t) {
			// Usually a misconfiguration
			throw new PushletException("Cannot instantiate class for " + aClassNameProp + "=" + clazz, t);
		}
	}

	/**
	 * Factory method: create object from property denoting class name.
	 *
	 * @param aClassNameProp property name e.g. "session.class"
	 * @return a Class object denoted by property
	 * @throws PushletException when class cannot be instantiated
	 */
	public static Class getClass(String aClassNameProp, String aDefault) throws PushletException {
		// Singleton + factory pattern:  create object instance
		// from configured class name
		String clazz = (aDefault == null ? getProperty(aClassNameProp) : getProperty(aClassNameProp, aDefault));

		try {
			return Class.forName(clazz);
		} catch (ClassNotFoundException t) {
			// Usually a misconfiguration
			throw new PushletException("Cannot find class for " + aClassNameProp + "=" + clazz, t);
		}
	}

	/**
	 * Initialize event sources from properties file.
	 */
	public static void load(String aDirPath) {
		// Load Event sources using properties file.
		try {
			// Try loading through classpath first (e.g. in WEB-INF/classes or from .jar)
			Log.info("Config: loading " + PROPERTIES_FILE + " from classpath");
			properties = Sys.loadPropertiesResource(PROPERTIES_FILE);
		} catch (Throwable t) {
			// Try from provided dir (e.g. WEB_INF/pushlet.properties)
			String filePath = aDirPath + File.separator + PROPERTIES_FILE;
			Log.info("Config: cannot load " + PROPERTIES_FILE + " from classpath, will try from " + filePath);

			try {
				properties = Sys.loadPropertiesFile(filePath);
			} catch (Throwable t2) {
				Log.fatal("Config: cannot load properties file from " + filePath, t);

				// Give up
				return;
			}
		}

		Log.info("Config: loaded values=" + properties);
	}

	public static String getProperty(String aName, String aDefault) {
		return properties.getProperty(aName, aDefault);
	}

	public static String getProperty(String aName) {
		String value = properties.getProperty(aName);
		if (value == null) {
			throw new IllegalArgumentException("Unknown property: " + aName);
		}
		return value;
	}

	public static boolean getBoolProperty(String aName) {
		String value = getProperty(aName);
		try {
			return value.equals("true");
		} catch (Throwable t) {
			throw new IllegalArgumentException("Illegal property value: " + aName + " val=" + value);
		}
	}

	public static int getIntProperty(String aName) {
		String value = getProperty(aName);
		try {
			return Integer.parseInt(value);
		} catch (Throwable t) {
			throw new IllegalArgumentException("Illegal property value: " + aName + " val=" + value);
		}
	}

	public static long getLongProperty(String aName) {
		String value = getProperty(aName);
		try {
			return Long.parseLong(value);
		} catch (Throwable t) {
			throw new IllegalArgumentException("Illegal property value: " + aName + " val=" + value);
		}
	}

	public static boolean hasProperty(String aName) {
		return properties.containsKey(aName);
	}


}

/*
 * $Log: Config.java,v $
 * Revision 1.5  2007/11/23 21:10:17  justb
 * add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
 *
 * Revision 1.4  2007/11/23 14:33:07  justb
 * core classes now configurable through factory
 *
 * Revision 1.3  2007/11/10 13:44:02  justb
 * pushlet.properties and sources.properties can now also be put under WEB-INF
 *
 * Revision 1.2  2006/05/06 00:10:11  justb
 * various chgs but not too serious...
 *
 * Revision 1.1  2005/02/18 12:36:47  justb
 * changes for renaming and configurability
 *

 *
 */
