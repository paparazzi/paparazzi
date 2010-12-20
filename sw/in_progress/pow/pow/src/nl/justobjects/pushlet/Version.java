// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.

package nl.justobjects.pushlet;

/**
 * Version info class.
 *
 * <h3>Purpose</h3>
 * Extract version info from jar manifest file.
 *
 * @author Just van den Broecke
 * @version $Id: Version.java,v 1.4 2006/05/06 00:10:11 justb Exp $
 */

public class Version {
	/** Version info extracted from the .jar manifest file (see build.xml and build.properties). */
	public static final String SOFTWARE_VERSION = Version.class.getPackage().getSpecificationVersion();
	public static final String BUILD_DATE = Version.class.getPackage().getImplementationVersion();
}

/*
 * $Log: Version.java,v $
 * Revision 1.4  2006/05/06 00:10:11  justb
 * various chgs but not too serious...
 *
 * Revision 1.3  2004/02/08 16:07:55  justb
 * *** empty log message ***
 *
 * Revision 1.2  2003/08/15 08:37:40  justb
 * fix/add Copyright+LGPL file headers and footers
 *
 * Revision 1.1  2003/08/11 21:29:48  justb
 * first checkin
 *
 *
 */
