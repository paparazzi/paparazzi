package pow;

import java.security.*;
import java.security.spec.InvalidKeySpecException;
import java.security.spec.InvalidParameterSpecException;
import java.security.spec.KeySpec;
//import java.security.spec.KeySpec;

import javax.crypto.*;
import javax.crypto.spec.*;

//import org.apache.commons.codec.binary.Hex;

import java.io.*;

/**
* implements AES encryption and decryption algorithms
* see @link http://java.sun.com/developer/technicalArticles/Security/AES/AES_v1.html
* see @link http://stackoverflow.com/questions/992019/java-256bit-aes-encryption
*/

public class AES {

  private byte[] key;
  private SecretKeySpec skeySpec;
  private Cipher cipher_encrypt;
  private Cipher cipher_decrypt;
  //
  /**
   * construct a cipher with a generated 128 bits key
 * @throws InvalidParameterSpecException 
 * @throws InvalidAlgorithmParameterException 
 * @throws NoSuchAlgorithmException
 * @throws NoSuchPaddingException
   */
  public AES() throws NoSuchAlgorithmException, NoSuchPaddingException, InvalidKeyException, InvalidParameterSpecException, InvalidAlgorithmParameterException {
	    // Get the KeyGenerator
	    KeyGenerator kgen = KeyGenerator.getInstance("AES");
	    kgen.init(128); // 192 and 256 bits may not be available
	    // Generate the secret key specs.
	    SecretKey skey = kgen.generateKey();
	    key = skey.getEncoded();
	    skeySpec = new SecretKeySpec(key, "AES");
	    cipher_encrypt = Cipher.getInstance("AES/CBC/PKCS5Padding");
	    cipher_encrypt.init(Cipher.ENCRYPT_MODE, skeySpec);
	    byte[] iv = cipher_encrypt.getParameters().getParameterSpec(IvParameterSpec.class).getIV();
	    cipher_decrypt = Cipher.getInstance("AES/CBC/PKCS5Padding");
	    cipher_decrypt.init(Cipher.DECRYPT_MODE, skeySpec,new IvParameterSpec(iv));
  }
  /**
   * construct a cipher from an user specified key
   * @param password a string representing the user key
   * @throws NoSuchAlgorithmException
   * @throws NoSuchPaddingException
   * @throws InvalidKeyException
   * @throws InvalidParameterSpecException
   * @throws InvalidAlgorithmParameterException
   * @throws InvalidKeySpecException
   */
  public AES(String password) throws NoSuchAlgorithmException, NoSuchPaddingException, InvalidKeyException, InvalidParameterSpecException, InvalidAlgorithmParameterException, InvalidKeySpecException {
	   // autre facon de generer la cle
	  	SecureRandom random = new SecureRandom();
	  	byte salt[] = new byte[8];
	  	random.nextBytes(salt);

	  	SecretKeyFactory factory = SecretKeyFactory.getInstance("PBKDF2WithHmacSHA1");
	  	KeySpec spec = new PBEKeySpec(password.toCharArray(), salt, 1024, 256);
	  	SecretKey tmp = factory.generateSecret(spec);
	  	SecretKey skey = new SecretKeySpec(tmp.getEncoded(), "AES");
	  	//
	    key = skey.getEncoded();
	    skeySpec = new SecretKeySpec(key, "AES");
	    cipher_encrypt = Cipher.getInstance("AES/CBC/PKCS5Padding");
	    cipher_encrypt.init(Cipher.ENCRYPT_MODE, skeySpec);
	    byte[] iv = cipher_encrypt.getParameters().getParameterSpec(IvParameterSpec.class).getIV();
	    cipher_decrypt = Cipher.getInstance("AES/CBC/PKCS5Padding");
	    cipher_decrypt.init(Cipher.DECRYPT_MODE, skeySpec,new IvParameterSpec(iv));
  }
  /**
   * construct a cipher from a key and a parameter array 
   * usefull to create a remote decryption cypher
   * @param key the key in array byte format
   * @param iv parameter of the cipher
   * @throws InvalidKeyException
   * @throws InvalidAlgorithmParameterException
   * @throws NoSuchAlgorithmException
   * @throws NoSuchPaddingException
   */
  public AES(byte[] key,byte[] iv) throws InvalidKeyException, InvalidAlgorithmParameterException, NoSuchAlgorithmException, NoSuchPaddingException{
	  	skeySpec = new SecretKeySpec(key, "AES");
	  	key = skeySpec.getEncoded();
	    cipher_encrypt = Cipher.getInstance("AES/CBC/PKCS5Padding");
	    cipher_encrypt.init(Cipher.ENCRYPT_MODE, skeySpec,new IvParameterSpec(iv));
	    cipher_decrypt = Cipher.getInstance("AES/CBC/PKCS5Padding");
	    cipher_decrypt.init(Cipher.DECRYPT_MODE, skeySpec, new IvParameterSpec(iv));
  }
  /**
   * get the init parameter of the cipher object in order to initialyse correctly a 
   * remote cipher
   */
  public byte[] getParamsEncrypt() throws InvalidParameterSpecException{
	  AlgorithmParameters params = cipher_encrypt.getParameters();
	  byte[] iv = params.getParameterSpec(IvParameterSpec.class).getIV();
	  return iv;
  }
  /**
   * get the cipher key 
   * @return the cipher key in byte array
   */
  public byte[] getKey(){return key;}
  
/**
 * encrypt a byte array with AES algorithm
 * @see org.apache.commons.codec.binary.Hex to convert byte Array into hexString
 * @param msg
 * @return a byte array containing the ciphered data
 * @throws IOException
 * @throws BadPaddingException 
 * @throws IllegalBlockSizeException 
 */
  public byte[] encrypt(byte[] msg) throws IOException, IllegalBlockSizeException, BadPaddingException{
	  ByteArrayInputStream in= new ByteArrayInputStream(msg);
	  ByteArrayOutputStream out = new ByteArrayOutputStream();
	  crypt(in, out, cipher_encrypt);
      return out.toByteArray();
  }
  /**
   * decrypt a byte array with AES algorithm
   * see @link org.apache.commons.codec.binary.Hex to convert byte Array into hexString
   * @param msg the msg to decrypt
   * @return the message decrypted 
   * @throws IOException
 * @throws BadPaddingException 
 * @throws IllegalBlockSizeException 
   */
  public byte[] decrypt(byte[] msg) throws IOException, IllegalBlockSizeException, BadPaddingException{
	  ByteArrayInputStream in= new ByteArrayInputStream(msg);
	  ByteArrayOutputStream out = new ByteArrayOutputStream();
	  crypt(in, out, cipher_decrypt);
      return out.toByteArray();
  }
  /**
   * Crypts or decrypts the specified input stream to the specified output
   * stream with a given cipher. The crypting or decrypting operation is 
   * determined by the cipher's state.
   * @param cipher The cipher used to crypt the specified input stream to the specified output
   * stream.
   * @param in the input srteal stream to be encypted or decrypted.
   * @param out the output stream to be encypted or decrypted.
   * @throws java.io.IOException if an I/O error occurs during crypting the input stream to the output stream.
 * @throws BadPaddingException 
 * @throws IllegalBlockSizeException 
   */
  private void crypt(InputStream in, OutputStream out, Cipher cipher)
      throws IOException, IllegalBlockSizeException, BadPaddingException
  {
      int blockSize = cipher.getBlockSize();
      int outputSize = cipher.getOutputSize(blockSize);
      byte[] inBytes = new byte[blockSize];
      byte[] outBytes = new byte[outputSize];
      
      int inLength = 0;
      boolean done = false;
      while(!done)
      {
          inLength = in.read(inBytes);
          if(inLength == blockSize)
          {
              try
              {
                  int outLength = cipher.update(inBytes, 0, blockSize, outBytes);
                  out.write(outBytes, 0, outLength);
              }
              catch(ShortBufferException e)
              {
                  e.printStackTrace();
              }
          }
          else
              done = true;
      }
      
          if(inLength > 0)
              outBytes = cipher.doFinal(inBytes, 0, inLength);
          else
              outBytes = cipher.doFinal();
          out.write(outBytes);
     
  }
}
