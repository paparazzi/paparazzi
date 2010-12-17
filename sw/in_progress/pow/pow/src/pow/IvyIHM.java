package pow;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

public class IvyIHM {
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
	private boolean logged ;
	private static int taille_text = 20;
	public IvyIHM(){
		logged = false;
		glob_fenetre = new JFrame("Connection to POW Server");
		glob_fenetre.setLocation(100,200);
		
		Container contenuTextField ;
		//ensemble des textfiled
		JPanel listTextField= new JPanel();
		listTextField.setLayout(new BoxLayout(listTextField, BoxLayout.PAGE_AXIS));
			contenuTextField = new JPanel(new FlowLayout());
			login_lbl = new JLabel("login");
			login_txtfield= new JTextField("",taille_text);
			contenuTextField.add(login_lbl);
			contenuTextField.add(login_txtfield);
			listTextField.add(contenuTextField);
			//
			contenuTextField = new JPanel(new FlowLayout());
			pwd_lbl = new JLabel("password");
			pwd_txtfield= new JPasswordField(taille_text);
			contenuTextField.add(pwd_lbl);
			contenuTextField.add(pwd_txtfield);
			listTextField.add(contenuTextField);
			//
			contenuTextField = new JPanel(new FlowLayout());
			url_lbl = new JLabel("hostname");
			url_txtfield= new JTextField("",taille_text);
			contenuTextField.add(url_lbl);
			contenuTextField.add(url_txtfield);
			listTextField.add(contenuTextField);
			//
		listTextField.setBorder(BorderFactory.createEmptyBorder(10,10,10,10));
		// les deux boutons
		
		log_btn = new JButton("Log !");
		quit_btn = new JButton("Quit !");;		
		Container contenuButton = new JPanel(new FlowLayout());
		contenuButton.add(quit_btn);
		contenuButton.add(log_btn);
		
		log_btn.addActionListener(new LogAction());
		quit_btn.addActionListener(new QuitAction());
		//
		glob_fenetre.setLayout(new BorderLayout());
		Container contenu = glob_fenetre.getContentPane();
		contenu.add(listTextField,BorderLayout.CENTER);
		contenu.add(contenuButton,BorderLayout.SOUTH);
		glob_fenetre.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		glob_fenetre.pack();
		glob_fenetre.setVisible(true);
	}
	
	public IvyIHM(boolean t){
		logged = false;
		glob_fenetre = new JFrame("Connection to POW Server");
		glob_fenetre.setLocation(100,200);
		
		Container contenuTextField ;
		//ensemble des textfiled
		JPanel listTextField= new JPanel();
		listTextField.setLayout(new BoxLayout(listTextField, BoxLayout.PAGE_AXIS));
			contenuTextField = new JPanel(new FlowLayout());
			login_lbl = new JLabel("login");
			login_txtfield= new JTextField("admin_ivy",taille_text);
			contenuTextField.add(login_lbl);
			contenuTextField.add(login_txtfield);
			listTextField.add(contenuTextField);
			//
			contenuTextField = new JPanel(new FlowLayout());
			pwd_lbl = new JLabel("password");
			pwd_txtfield= new JPasswordField("pwdadmin_ivy",taille_text);
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
		quit_btn = new JButton("Quit !");;		
		Container contenuButton = new JPanel(new FlowLayout());
		contenuButton.add(quit_btn);
		contenuButton.add(log_btn);
		contenuButton.add(more_btn);
		more_btn.addActionListener(new MoreAction(glob_fenetre));
		log_btn.addActionListener(new LogAction());
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
		site_txtfield= new JTextField("TestServletPow",taille_text);
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
	
	 public static void main(String[] args)
	   {
		 SwingUtilities.invokeLater(new Runnable() {
			    public void run() {
			    	new IvyIHM(true);
			    }
			});
	   }
	 
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
	 
	 
	 class LogAction implements ActionListener {
		 public void actionPerformed(ActionEvent e){
			if(!logged){
			Runnable code = new Runnable() {
				    public void run() {
				    	Ivy2UdpReading module;
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
							module = new Ivy2UdpReading(info,login,pwd);
				    	}
				    	else{
					    	module = new Ivy2UdpReading(host,login,pwd);
				    	}
					 	
					 	try {
							module.getWebId();
							new Thread(module).start();
						} catch (IvyConnectionExeption e) {
							// echec du log
							//custom title, error icon
							System.out.println("erreur de connection : "+e.toString());
							JOptionPane.showMessageDialog(glob_fenetre,
							    "Your password or your login are not correct, retry please...\n"+
							    "Be sure a valid SSL certificat exist in your $JAVA_HOME/jre/lib/security folder,\n"+
							    "otherwise use InstallCert [hostname] java programm",
							    "logging error",
							    JOptionPane.ERROR_MESSAGE);
							

						}
					 	
					 	
					 	// clearing
					 	for(int i = 0; i< pwd_array.length;i++){
					 		pwd_array[i]='0';
					 	}
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

