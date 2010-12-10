package pow;
import java.io.UnsupportedEncodingException;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
/**
 * tool class to implemente MD5 encoding
 * ( found on the web )
 *
 */
public class Md5
{
    public static String encode(String password)
    {
        byte[] uniqueKey = password.getBytes();
        byte[] hash      = null;

        try
        {
            hash = MessageDigest.getInstance("MD5").digest(uniqueKey);
        }
        catch (NoSuchAlgorithmException e)
        {
            throw new Error("No MD5 support in this VM.");
        }

        StringBuilder hashString = new StringBuilder();
        for (int i = 0; i < hash.length; i++)
        {
            String hex = Integer.toHexString(hash[i]);
            if (hex.length() == 1)
            {
                hashString.append('0');
                hashString.append(hex.charAt(hex.length() - 1));
            }
            else
                hashString.append(hex.substring(hex.length() - 2));
        }
        return hashString.toString();
    }
/*
    public static String hash(String plaintext) {
        MessageDigest md = null;
 
        try {
            md = MessageDigest.getInstance("SHA-1"); // SHA-1 generator instance
        } catch(NoSuchAlgorithmException e) {
            return "";
        }
 
        try {
            //8859_1 ou UTF-8
            md.update(plaintext.getBytes("UTF-8")); // Message summary generation
        } catch(UnsupportedEncodingException e) {
            return "";
        }
 
        byte raw[] = md.digest(); // Message summary reception
        
        try{
            String hash = new String(org.apache.commons.codec.binary.Base64.encodeBase64(raw),"UTF-8");
            //String hash = new String(raw);
            return hash;
        }
        catch (UnsupportedEncodingException use){
            return "";
        }
    }
    
  */  
    public static void main(String[] args)
    {
        if (args.length != 1)
        {
            System.out.println("Usage: java Md5 <string_to_encode>");
            return;
        }

        String toEncode = args[0];

        System.out.println("Original string ... " + toEncode);
        System.out.println("String MD5 ........ " + encode(toEncode));
    }
}