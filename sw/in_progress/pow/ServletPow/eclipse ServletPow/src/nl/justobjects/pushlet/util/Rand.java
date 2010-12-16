// Copyright (c) 2000 Just Objects B.V. <just@justobjects.nl>
// Distributable under LGPL license. See terms of license at gnu.org.
package nl.justobjects.pushlet.util;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.Random;

/**
 * Randomizing routines.
 *
 * @author Just van den Broecke
 * @version $Id: Rand.java,v 1.4 2007/12/07 12:57:40 justb Exp $
 */
public class Rand {
	private static char CHARS[] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'r', 's', 't', 'u', 'v', 'w', 'y', 'z'};
	private static char NON_VOWELS[] = {'b', 'c', 'd', 'f', 'g', 'h', 'j', 'k', 'l', 'm', 'n', 'p', 'q', 'r', 's', 't', 'v', 'w', 'z'};
	private static char VOWELS[] = {'a', 'e', 'i', 'o', 'u', 'y'};
	private static Random random = new Random();

	public static char randomChar() {
		return CHARS[randomInt(0, CHARS.length - 1)];
	}

	public static char randomVowel() {
		return VOWELS[randomInt(0, VOWELS.length - 1)];
	}

	public static char randomNonVowel() {
		return NON_VOWELS[randomInt(0, NON_VOWELS.length - 1)];
	}

	public static File randomTempDir() throws Exception {
		File file = new File(System.getProperty("java.io.tmpdir") + File.separator + "oasetest" + File.separator + randomString(12));
		file.mkdirs();
		file.deleteOnExit();
		return file;
	}

	public static File randomTempFile() throws Exception {
		File file = new File(System.getProperty("java.io.tmpdir") + File.separator + "oase-" + randomString(6));
		file.createNewFile();
		file.deleteOnExit();
		return file;
	}

	public static File randomBinaryFile(int aSize) throws Exception {
		File file = randomTempFile();
		FileOutputStream fos = new FileOutputStream(file);
		fos.write(randomBytes(aSize));
		fos.close();
		return file;
	}

	public static File randomTextFile(int aSize) throws Exception {
		File file = randomTempFile();
		PrintWriter pw = new PrintWriter(new FileOutputStream(file));
		pw.write(randomString(aSize));
		pw.close();
		return file;
	}

	public static byte[] randomBytes(int aSize) {
		return randomBlob(aSize);
	}

	public static byte[] randomBlob(int aSize) {
		byte[] retval = new byte[aSize];
		for (int i = 0; i < retval.length; i++) {
			retval[i] = randomByte();
		}
		return retval;
	}

	public static byte randomByte() {
		return (byte) random.nextInt();
	}

	public static double randomDouble() {
		return random.nextLong();
	}


	public static int randomInt() {
		return random.nextInt();
	}

	public static int randomInt(int min, int max) {
		return (int) ((Math.random() * (double) (max + 1 - min)) + min);
	}

	public static long randomLong() {
		return random.nextLong();
	}

	public static long randomLong(long min, long max) {
		return (long) ((Math.random() * (double) (max + 1L - min)) + min);
	}

	public static String randomName(int aLength) {
		StringBuffer sb = new StringBuffer(aLength);
		for (int i = 0; i < aLength; i++) {
			sb.append(i % 2 == 0 ? randomNonVowel() : randomVowel());
		}
		return sb.toString();
	}

	public static void randomSleep(long min, long max) {
		try {
			Thread.sleep(randomLong(min, max));
		} catch (InterruptedException ie) {

		}
	}

	public static String randomString(int aLength) {
		StringBuffer sb = new StringBuffer(aLength);
		for (int i = 0; i < aLength; i++) {
			sb.append(randomChar());
		}
		return sb.toString();
	}

	public static String randomString() {
		return "" + randomLong();
	}


}

/*
 * $Log: Rand.java,v $
 * Revision 1.4  2007/12/07 12:57:40  justb
 * added log4j and make it the default logging method
 *
 * Revision 1.3  2007/11/23 21:10:17  justb
 * add hooks for custom logging (you can override DefaultLogger in pushlet.properties)
 *
 * Revision 1.2  2004/09/03 22:35:38  justb
 * Almost complete rewrite, just checking in now
 *
 * Revision 1.1  2004/03/10 12:21:27  justb
 * *** empty log message ***
 *
 *
 */