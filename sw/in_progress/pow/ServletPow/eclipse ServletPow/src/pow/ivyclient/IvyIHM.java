package pow.ivyclient;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
/**
 * represent the interface for a ivy user who wants to connect a bus ivy to
 * the web server
 * @author genin
 *
 */
public class IvyIHM implements Runnable{
	private JFrame glob_fenetre;
	private JLabel login_lbl;
	private JTextField login_txtfield;
	private JLabel pwd_lbl;
	private JPasswordField pwd_txtfield;
	private JLabel url_lbl;
	private JTextField url_txtfield;
	
	private Container contenuMore;
	private boolean moreIsVisible ;
	private JLabel protocol_lbl;
	private JComboBox protocol_field;
	private JLabel port_lbl;
	private JComboBox port_field;
	private JLabel site_lbl;
	private JTextField site_txtfield;
	private JLabel servlet_lbl;
	private JTextField servlet_txtfield;
	private String[] protocolsStrings = { "http", "https" };
	private String[] portsStrings = { "none", "80","8080","443" };
	private JButton log_btn;
	private JButton quit_btn;
	private JButton more_btn;
	private JButton stop_btn;
	private boolean logged ;
	private boolean relogged ;
	private static int taille_text = 20;
	
	private Ivy2Udp module;
	/**
	 * construct the interface
	 */
	public IvyIHM(){
		module = null;
		logged = false;relogged=false;
		glob_fenetre = new JFrame("Connection to POW Server");
		glob_fenetre.setLocation(100,200);
		
		Container contenuTextField ;
		//ensemble des textfiled
		JPanel listTextField= new JPanel();
		listTextField.setLayout(new BoxLayout(listTextField, BoxLayout.PAGE_AXIS));
			contenuTextField = new JPanel(new FlowLayout());
			login_lbl = new JLabel("login");
			login_txtfield= new JTextField("your login",taille_text);
			contenuTextField.add(login_lbl);
			contenuTextField.add(login_txtfield);
			listTextField.add(contenuTextField);
			//
			contenuTextField = new JPanel(new FlowLayout());
			pwd_lbl = new JLabel("password");
			pwd_txtfield= new JPasswordField("",taille_text);
			contenuTextField.add(pwd_lbl);
			contenuTextField.add(pwd_txtfield);
			listTextField.add(contenuTextField);
			//
			contenuTextField = new JPanel(new FlowLayout());
			url_lbl = new JLabel("hostname");
			url_txtfield= new JTextField("blanc",taille_text);
			contenuTextField.add(url_lbl);
			contenuTextField.add(url_txtfield);
			listTextField.add(contenuTextField);
			//
		listTextField.setBorder(BorderFactory.createEmptyBorder(10,10,10,10));
		// les trois boutons
		more_btn = new JButton("More...");
		log_btn = new JButton("Log !");
		stop_btn = new JButton("Stop !");
		quit_btn = new JButton("Quit !");	
		Container contenuButton = new JPanel(new FlowLayout());
		contenuButton.add(quit_btn);
		contenuButton.add(log_btn);
		contenuButton.add(stop_btn);
		contenuButton.add(more_btn);
		more_btn.addActionListener(new MoreAction(glob_fenetre));
		log_btn.addActionListener(new LogAction());
		stop_btn.addActionListener(new StopAction());
		quit_btn.addActionListener(new QuitAction());
		//le panneau more....
		contenuMore = new JPanel();
		contenuMore.setLayout(new BoxLayout(contenuMore, BoxLayout.PAGE_AXIS));
		
		Container contenuMoreMore = new JPanel(new FlowLayout());
		
		protocol_field = new JComboBox(protocolsStrings);
		protocol_lbl = new JLabel("protocol");
		contenuMoreMore.add(protocol_lbl);
		contenuMoreMore.add(protocol_field);
		//contenuMore.add(contenuMoreMore);
		
		//contenuMoreMore = new JPanel(new FlowLayout());
		port_lbl = new JLabel("port");
		port_field = new JComboBox(portsStrings);;
		contenuMoreMore.add(port_lbl);
		contenuMoreMore.add(port_field);
		contenuMore.add(contenuMoreMore);
		
		contenuMoreMore = new JPanel(new FlowLayout());
		site_lbl= new JLabel("site name");
		site_txtfield= new JTextField("ServletPow",taille_text);
		contenuMoreMore.add(site_lbl);
		contenuMoreMore.add(site_txtfield);
		contenuMore.add(contenuMoreMore);
		
		contenuMoreMore = new JPanel(new FlowLayout());
		servlet_lbl= new JLabel("servlet name");
		servlet_txtfield= new JTextField("Ivy2TomcatHttpServer.srv",taille_text);
		contenuMoreMore.add(servlet_lbl);
		contenuMoreMore.add(servlet_txtfield);
		contenuMore.add(contenuMoreMore);
		moreIsVisible = false;
		contenuMore.setVisible(moreIsVisible);
		//
		glob_fenetre.setLayout(new BorderLayout());
		Container contenu = glob_fenetre.getContentPane();
		contenu.add(listTextField,BorderLayout.NORTH);
		contenu.add(contenuMore,BorderLayout.CENTER);
		contenu.add(contenuButton,BorderLayout.SOUTH);
		glob_fenetre.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		glob_fenetre.pack();
		glob_fenetre.setVisible(true);
	}
	/**
	 * specify whether the module should reconnected or not
	 * @param l true or false
	 */
	public void setReLogged(boolean l){
		relogged = l;
	}
	/**
	 * loop to perform a periodic check for automatic reconnection
	 */
	public void run(){
		while(true){
			try {
				Thread.sleep(15000);
				if(module !=null){
					
					if(relogged){
						System.out.println("module auto reconnecting to server");
						module = null;
						relogged=!connectModule(false);
	
					}
				}
				//check
			} catch (InterruptedException e) {}
		}
	}
	/**
	 * launch the interface to connect the ivy bus to POW server
	 * @param args
	 */
	 public static void main(String[] args)
	   {
		 SwingUtilities.invokeLater(new Runnable() {
			    public void run() {
			    	IvyIHM ivyihm = new IvyIHM();
			    	new Thread(ivyihm).start();
			    }
			});
	   }
	 /**
	  * implements the action performed when the quit button is pressed
	  * @author genin
	  */
	 class QuitAction implements ActionListener {
		 public void actionPerformed(ActionEvent e){
			 Runnable code = new Runnable() {
				    public void run() {
				    	System.exit(0);
				    }
			 };
			 if (SwingUtilities.isEventDispatchThread()) {
				    code.run();
			 } else {
				    SwingUtilities.invokeLater(code);
			 }
			 
		 }
	 }
	 /**
	  * implements the action performed when the stop button is pressed
	  * @author genin
	  */
	 class StopAction implements ActionListener {
		 public void actionPerformed(ActionEvent e){
			
			Runnable code = new Runnable() {
				    public void run() {			    	
				    	module.stop_thread();
				    	logged = false;relogged = false;
				    }
			};
			
			 if (SwingUtilities.isEventDispatchThread()) {
				    code.run();
				  } else {
				    SwingUtilities.invokeLater(code);
				  }
		}
	}
			
	 /**
	  * implements the action performed when the log button is pressed
	  * @author genin
	  */
	 class LogAction implements ActionListener {
		 public void actionPerformed(ActionEvent e){
			if(!logged){
			Runnable code = new Runnable() {
				    public void run() {
				    	connectModule(true);
				    }
				  };
				  if (SwingUtilities.isEventDispatchThread()) {
				    code.run();
				  } else {
				    SwingUtilities.invokeLater(code);
				  }
			 }
			 else{
				 JOptionPane.showMessageDialog(glob_fenetre,
						    "you are already logged and send information to server",
						    "Info",JOptionPane.WARNING_MESSAGE);
			 }
		 }
		
	 }
	 /**
	  * performs all the step to run the ivy module
	  * @param displayWarning set if information about wrong login are displayed
	  * @return true if the https connection to the server is successful
	  */
	 private boolean connectModule(boolean displayWarning){
		 boolean res;
		 String login = login_txtfield.getText();
		 	String host  = url_txtfield.getText();
		 	char[] pwd_array = pwd_txtfield.getPassword();
		 	String pwd = new String(pwd_array);
	    	if (moreIsVisible){
	    		String siteName     = site_txtfield.getText();
			 	String servletName  = servlet_txtfield.getText();
			 	String strports ;
			 	strports  = portsStrings[port_field.getSelectedIndex()];
			 	int port=-1;
			 	if (!strports.equals("none")){
			 		port = Integer.parseInt(strports);
			 	}
			 	String strProto = protocolsStrings[ protocol_field.getSelectedIndex()];
				System.out.println("port : "+port+"\tproto :"+ strProto);
			 	PowUrl info   = new  PowUrl(strProto,host,port,siteName,servletName);
				JOptionPane.showMessageDialog(glob_fenetre,
					    "you will be connected to "+ info.getWebUrl(),
					    "logging info",
					    JOptionPane.INFORMATION_MESSAGE);
				module = new Ivy2Udp(this,info,login,pwd);
	    	}
	    	else{
	    		PowUrl info = new PowUrl("https",host,-1,site_txtfield.getText(), servlet_txtfield.getText());
	    		System.out.println("logging to : " + info.getWebUrl());
	    		module = new Ivy2Udp(this,info,login,pwd);
	    	}
		 	try {
				module.getWebId();
				new Thread(module).start();
				logged = true;
				res = true;
			} catch (IvyConnectionExeption e) {
				module.stop_thread();
				// echec du log
				//custom title, error icon
				System.out.println("erreur de connection : "+e.toString());
				if (displayWarning){
				JOptionPane.showMessageDialog(glob_fenetre,
				    "Your password or your login are not correct, retry please...\n"+
				    "Be sure a valid SSL certificat exist in your $JAVA_HOME/jre/lib/security folder,\n"+
				    "otherwise use InstallCert [hostname] java programm"+
				    "\nit's possible also that server is not currenlty running",
				    "logging error",
				    JOptionPane.ERROR_MESSAGE);
				}
				logged = false;
				res = false;
			}
		 	// clearing
		 	for(int i = 0; i< pwd_array.length;i++){
		 		pwd_array[i]='0';
		 	}
		 	return res;
	 }
	 
	 /**
	  * implements the action performed when the more button is pressed
	  * @author genin
	  */
	 class MoreAction implements ActionListener {
		 JFrame globalFrame;
		 
		 MoreAction(JFrame g){
			 globalFrame = g;
		 }
		 public void actionPerformed(ActionEvent e){
			 Runnable code = new Runnable() {
				    public void run() {
				    	if(moreIsVisible){
				    		contenuMore.setVisible(false);moreIsVisible=false;more_btn.setText("More...");}
				    	else {
				    		contenuMore.setVisible(true);moreIsVisible=true;more_btn.setText("Less...");
				    	}
				    	globalFrame.pack();
				    	
				    }
			 };
			 if (SwingUtilities.isEventDispatchThread()) {
				    code.run();
			 } else {
				    SwingUtilities.invokeLater(code);
			 }
			 
		 }
	 }
	 
}

