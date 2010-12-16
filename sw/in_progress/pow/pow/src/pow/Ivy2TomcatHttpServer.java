package pow;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;

import javax.servlet.ServletConfig;
import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import org.apache.commons.codec.binary.Hex;
import org.apache.commons.fileupload.*;
import org.apache.commons.fileupload.servlet.ServletFileUpload;
//import org.apache.commons.fileupload.util.Streams;
import org.apache.commons.fileupload.disk.DiskFileItemFactory;

import java.util.HashMap;
import java.util.List;
import java.util.Iterator;
import java.io.File;
import java.net.InetAddress;
/**
  * Handles the connection request from a new ivy bus
 * provide a unique id for the ivy bus
 * provide a unique id for each new drone on a ivy bus
 * handles the uploading of the configuration files for each new drone on a ivy bus
 */
public class Ivy2TomcatHttpServer extends HttpServlet {
	private static final long serialVersionUID = 1L;
	private static int webIdCount = 1;
	private static int droneIdCount = 1;
	private String default_folder="blabla";
	private HashMap<InetAddress,SessionIvy> tableIvySession ;// correspondance webid,session ivy
    /**
     * 
     */
    public Ivy2TomcatHttpServer() {
        super();
        
    }

    public void init(ServletConfig config) throws ServletException {
    	//
    	// faire le menage du repertoire upload
    	//config.getServletContext().getRealPath("");
    	//String default_folder=getServletContext().getRealPath("/"); // repertoire de l'appli
    	default_folder = config.getServletContext().getRealPath("");
    	//System.out.println("default appli folder : "+ default_folder);
    	
    	String filepath = default_folder + "/upload";
    	File repository = new File(filepath);
    	deleteDirectory(repository);
		repository = new File(filepath);
		if ((!repository.exists()) && (!repository.isDirectory())) {
			repository.mkdir(); // exists isDirectory() 
		}
		tableIvySession = new HashMap<InetAddress,SessionIvy>();
		// mise a disposition des sessions pour les autres servlet
		config.getServletContext().setAttribute("ivySessionTable", tableIvySession);
		// creation d'un fichier d'user pour Ivy par defaut si n'existe pas
    	File def_user_file = new File(default_folder + "/conf/"+"userIvyTable.tbl");
    	if (!def_user_file.exists()){
    		User usr0 = new User("admin_ivy","pwdadmin_ivy",Rights.IVY);    		
            UserTab logTab = new UserTab();
            try{
            	logTab.insert(usr0);
            }
            catch(AlreadyRegisteredUserException e) {}    
            logTab.serialize(default_folder + "/conf/"+"userIvyTable.tbl");
    	}
    }
    
    static private boolean deleteDirectory(File path) { 
        boolean resultat = true; 
        
        if( path.exists() ) { 
                File[] files = path.listFiles(); 
                for(int i=0; i<files.length; i++) { 
                        if(files[i].isDirectory()) { 
                                resultat &= deleteDirectory(files[i]); 
                        } 
                        else { 
                        resultat &= files[i].delete(); 
                        } 
                } 
        } 
        resultat &= path.delete(); 
        return( resultat ); 
}
	/**
	 * 
	 */
	protected void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
		//PrintWriter out = response.getWriter();
    	//out.println("Hello, world!");
		//out.close();
	}

	/**
	 * handles request from ivy http(s) client
	 * check login and password
	 * give unique webId to identify the client
	 * generate AES cipher to encrypt and decrypt udp link 
	 */
	protected void doPost(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
		String order = request.getParameter("order");
		// Check that we have a file upload request
		if (order!=null){
			//System.out.println("ivyId : " + ivyId);
			if (order.equals("requestWebId")) 
			{ // client request to have a webId
				System.out.println("Web Id requested !!!");
				String login = request.getParameter("login");
				String password = request.getParameter("pwd");
				UserTab ivyLogTab = UserTab.unserialize(default_folder + "/conf/"+"userIvyTable.tbl");
				if (ivyLogTab.checkUser(login, password)){
					int newWebId = webIdCount;webIdCount++;
					//tableSession.put(newWebId,new SessionPow(newWebId));
			    	// generating aes key
			    	try{
				    	AES cipher_aes = new AES();
				    	// necessaire de convertir en hexa pour eviter des caracteres non supportes par XML parser
				    	String aesKey = Hex.encodeHexString(cipher_aes.getKey());
				    	String aesIv  = Hex.encodeHexString(cipher_aes.getParamsEncrypt());
				    	PrintWriter out = response.getWriter();
						out.println("<ivyconnection>");
						out.println("<ConnectionAck>1</ConnectionAck>");
				    	out.println("<webid>"+newWebId+"</webid>");
				    	out.println("<aeskey>"+aesKey+"</aeskey>");
				    	out.println("<aesiv>"+aesIv+"</aesiv>");
				    	out.println("</ivyconnection>");
				    	InetAddress remoteClient = InetAddress.getByName(request.getRemoteAddr());
				    	SessionIvy session = new SessionIvy(login,newWebId,cipher_aes,remoteClient);
				    	tableIvySession.put(remoteClient,session);
				    	out.close();
				    	System.out.println("**************************************");
						System.out.println("Web Id sended         = " + newWebId);
						System.out.println("remote client         = " + request.getRemoteAddr() + " host=" + request.getRemoteHost() + " port=" +  request.getRemotePort());
						System.out.println("key sended            = " + aesKey );
						System.out.println("iv parameter          = " + aesIv );
						System.out.println("**************************************");
			    	}
			    	catch (Exception ex){
			    		PrintWriter out = response.getWriter();
						out.println("<ivyconnection>");
				    	out.println("<ConnectionAck>2</ConnectionAck>");
				    	out.println("</ivyconnection>");
						out.close();
						System.out.println("AES encryption not supported by server!!!\n");
						ex.printStackTrace();
			    	}	    	
				}
				else {
					PrintWriter out = response.getWriter();
					out.println("<ivyconnection>");
			    	out.println("<ConnectionAck>0</ConnectionAck>");
			    	out.println("</ivyconnection>");
					out.close();
					System.out.println("Not authorized access attempt from an ivy client!!!");
				}
			}
			else if (order.equals("reconnect"))
			{
				
			}
			else if (order.equals("requestNewDroneWebId"))
			{// client has a webId (not in multipart post) yet and wants to do somtheing else than uploading
				System.out.println("Drone Web Id for new drone requested !!!");
				String ivyId = request.getParameter("ivyWebId");
				//String ivyDroneId = request.getParameter("ivyDroneId");
				System.out.println("ivyId  " + ivyId + " request new Drone Web Id for unknown ivy drone ");// + ivyDroneId);
				int webidsession = Integer.parseInt(ivyId);
				//int droneivyid = Integer.parseInt(ivyDroneId);
				int newdroneWebId = droneIdCount;droneIdCount++;
				PrintWriter out = response.getWriter();				
				out.println("<DroneWebId:"+ newdroneWebId +">");					
				out.close();
				System.out.println("new drone Web Id sended (dronewebid = "+newdroneWebId+") !!!");
			}
		}
		else  //  client has a webId ( in multipart post) yet and wants to upload conf files
		{   
			System.out.println("uploading file requested...");
			String res = uploadFile(request);		
			PrintWriter out = response.getWriter();				
			out.println("<droneconnection>");
	    	out.println(res);
	    	out.println("</droneconnection>");					
			out.close();			
		}// end if check IvyId
	
	}

	/* for parameter
	see http://stackoverflow.com/questions/992019/java-256bit-aes-encryption
	
	 */
	
	
	
	
	/*
	 * handles the upload of a file from a ivy bus to the server via http
	 */
	private String uploadFile(HttpServletRequest request){
		boolean isMultipart = ServletFileUpload.isMultipartContent(request);
		String resu = "<UploadAck>-1</UploadAck>";
		List  items=null;
		String webid="zzz";
		String droneid = "yyy";
        String filepath="";
       // String default_folder=getServletContext().getRealPath("/");
       
        System.out.println("!!!def upload rep !!! " + default_folder);
        File uploadedFile;
        if (isMultipart)
		{   		 
        	//	System.out.println("is multipart ....");
			// Create a factory for disk-based file items
			FileItemFactory factory = new DiskFileItemFactory();
			// Set factory constraints
			//	factory.setSizeThreshold(yourMaxMemorySize);
			//  factory.setRepository(yourTempDirectory);
			// Create a new file upload handler
			ServletFileUpload upload = new ServletFileUpload(factory);
			// Set overall request size constraint
			// upload.setSizeMax(yourMaxRequestSize);		
			// Parse the request
			try {
				items = upload.parseRequest(request); 
				// Process the uploaded items
				// on cherche le web id
				// System.out.println("on cherche le web id ....");
				boolean doIt1 = true;boolean doIt2 = true;
				Iterator iter = items.iterator();				
				while ((doIt1||doIt2) && iter.hasNext()) {
					FileItem item = (FileItem) iter.next();
					if (item.isFormField()) {
						String fieldName = item.getFieldName();
						if (fieldName.equals("ivyWebId")){
							doIt1 = false ;
						    webid = item.getString();
						}
						else if (fieldName.equals("droneWebId")){
							doIt2 = false;
							droneid = item.getString();
						}
					}
				}
				filepath = default_folder +"/upload/"+droneid;
				File repository = new File(filepath);
				if ((!repository.exists()) && (!repository.isDirectory())) {
					boolean res=repository.mkdir(); // exists isDirectory() 
					System.out.println("repertoire "+filepath+" cree ? " + res);
				}
				if (!webid.equals("zzz")){				
					// on cherche les fichiers
					iter = items.iterator();				
					while (iter.hasNext()) {	
						FileItem item = (FileItem) iter.next();
						if (!item.isFormField()) { // on verifie la nature de l'item						
							String fileName = item.getName();
							//String contentType = item.getContentType();
							//boolean isInMemory = item.isInMemory();
							//long sizeInBytes = item.getSize();
							uploadedFile = new File(filepath+"/"+fileName);
					    	try {
								item.write(uploadedFile);
								resu = "<UploadAck iddrone=\""+droneid+"\">1</UploadAck>";
								System.out.println("file: "+fileName+" for web drone "+droneid+" was uploaded successfully");
							} catch (Exception e) {
								resu = "<UploadAck iddrone=\""+droneid+"\">0</UploadAck>";
								System.out.println("unable to write file on server : "+droneid+"/"+fileName);
								e.printStackTrace();
							}
						} //end if
					} // end while				
				}
			} catch (FileUploadException e) {
				System.out.println("unable to parse client request ");
				e.printStackTrace();
			}
		} // end if multipart
        return resu;
	}
}
