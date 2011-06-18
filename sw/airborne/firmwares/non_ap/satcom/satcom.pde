// Program for ArduIno sending message received via the I2C to the Satcom module
// Originaly designed and tested for an ArduIno Mini Pro board from Sparkfun connected with a 9062 Iridium modem
//
// Autor : Loic Maudou (loic.maudou_corsica@polytechnique.org)
// Date : Mai 2011
//
//****************************
//  Connections between the arduIno
//  and the Satcom module
//****************************/
// TX0 -> TX satcom, serial port
// RX1 <- RX satcom, serial port
// 2 
// 3 <- NWA (unsused)
// 4
// 5
// 6 -> ON/OFF
// 7 <- CTS
// 8 -> RTS
// 9 -> DTR
// 10<- DSR
// 11
// 12
// 13
// A0
// A1
// A2
// A3
// A4 I2C autopilot
// A5 I2C autopilot


/********************************
/* Includes 
/********************************/
#include <Wire.h> // Librairy used for the I2C


/********************************
/* Define 
/********************************/
#define ADDRESS_I2C 19 // Define the adress of the arduino in the I2C connection. (Arduino is seen as a slave)
#define MINIMAL_SATELLITE_COVER 2 // Minimal satellite reception quality out of 5 (5 for a very good cover) before trying to run an SBDIX session
#define MAX_SIZE_MESSAGE 60 // Taille maximale en byte des messages

/********************************
/* Global declarations
/********************************/

// Buffer containning the last message to be send
// By doing this way, we avoid sending old messages
// if the sending os not fast enough, but we do not allowed
// file of messages
byte bufferMessage[MAX_SIZE_MESSAGE]; // The size of the message is maximized here
int bufferSize; // Size used of the buffer

// flag telling if a message is waiting to be send
boolean flagWaitingMessage=false;

/********************************
/* Definition of the name of the pins
/********************************/

int Nwa=3;
int OnOff=6;
int Cts=7;
int Rts=8;
int Dtr=9;
int Dsr=10;
int Led=13;

/********************************
/* Flags for the parseur
/********************************/
boolean flagOk=false; // flag telling that an "OK" collation has been read
boolean flagReady=false; // flag telling that a "READY" collation has been read
int flagReception=0; // flag telling the quality of the reception
boolean flagSBDIX=false; // flag telling that a SBDIX collation was read


/********************************************************************************
/* Procedure setting up the module and the arduIno board
/********************************/
void setup()
{ 
  // Definition of the input/output
  pinMode(Nwa, INPUT);
  pinMode(OnOff,OUTPUT);
  pinMode(Cts,INPUT);
  pinMode(Rts,OUTPUT);
  pinMode(Dtr,OUTPUT);
  pinMode(Dsr,INPUT);
  pinMode(Led,OUTPUT); // Pin for debuging, connected to a LED included in the ArduIno Mini Pro board


  // Launch of the I2C protocol

  // We define the adress for the slave connection
  Wire.begin(ADDRESS_I2C);
  //
  Wire.onReceive(receiveEvent);

  // Launch of the serial port
  Serial.begin(19200);

  // Sending of a message "READY"

  // Definition of the message
  bufferSize=5;

  bufferMessage[0]=byte('R');
  bufferMessage[1]=byte('E');
  bufferMessage[2]=byte('A');
  bufferMessage[3]=byte('D');
  bufferMessage[4]=byte('Y');

  sendMessageViaSatcom(0);
}

// ******************************************************************************
// Loop of the program
// Check if there is a message waiting to be sent
void loop()
{ // If a message is waiting
  if (flagWaitingMessage){
    // Send the message
    digitalWrite(Led,HIGH);
    sendMessageViaSatcom(0);
    digitalWrite(Led,LOW);
  }
  else {
    // Wait 1 sec
    delay(1000);
  }
}

// ******************************************************************************
// Function launched each time a message is received via the I2C
// working as an exception raised by the reception via the I2C 
void receiveEvent(int howMany){
  if (howMany>=1){
    if (Wire.receive()!=0){
      digitalWrite(Led,HIGH);
      bufferSize=howMany-1;

      // Collect all the byte in the buffer
      for (int i =0; i<bufferSize;i++){
        bufferMessage[i]=Wire.receive();
      }

      // We notify the program that a message is waiting to be sent
      flagWaitingMessage=true;
  delay(100);
      digitalWrite(Led,LOW);
 
    }
  }
}

// ******************************************************************************
// Function sending the message via the Satcom
// The argument n reprensent the consecutive number of time the program tried to send a message unsuccessfully
void sendMessageViaSatcom(int n){
  // If the Satcom module is Off
  if (digitalRead(OnOff)==LOW){
    // Power on the Satom module and make it ready for AT commands
    initialisation();
    // Configure the modem to send a command each time the quality of the satellite reception changes
    configSatelliteAlert();
  }

  // Wait to have a sufficient satellite reception
  waitForSatelliteCover(MINIMAL_SATELLITE_COVER);

  // Begining of the sending procedure

  // Put the flag of a waiting message down
  flagWaitingMessage=false;
  
  // Send the message to be sent to the Satcom module
  writeBuffer();
  // Launch an SBD session for the modem to send the message in the BO (Buffer Out)
  SBDsession(n);

  // if there is no more message to send (meaning that no new messge arrived via the I2C during the sending)
  if (!flagWaitingMessage){
    // stops the modem with all the precautions available
    smoothShutdown();
  }
  //digitalWrite(Led,LOW);
}

// ******************************************************************************
// Power on the Satcom module and wait for it to be ready to receive AT commands
// If the program waits more than 10 seconds without sucess, reboot the module
void initialisation(){
  //digitalWrite(Led,HIGH);
  // Flush the serial port buffer
  Serial.flush();
  // initialisation of the flags
  flagReception =0;
  flagOk=false;

  // Power on the Satcom module
  digitalWrite(OnOff,HIGH);

  // Tell the satcom module that the Data Terminal is Ready
  digitalWrite(Dtr,LOW); // Active low output

  // Loop counter
  int i=0;

  // Wait for the Data Set to be ready
  // (maximal waiting time of 10 seconds)
  // test every 500 ms
  while(digitalRead(Dsr)==HIGH && i<20){
    delay(500);
    i++;
  }

  // if the Data Terminal is not ready
  if (digitalRead(Dsr)==HIGH){ // low active
    forcedShutdown(); // strong reboot because the terminal is not active
    initialisation(); // retry to boot the modem
  }
  else { // if the Data Terminal is telling that it is ready

    // loop counter
    int compt=0;

    // collation "OK"
    boolean collation=false;
  
    // Try to send some AT commands until it gets a good collation
    // because I noticed that the first AT commands often doesn't not work and send errors
    do {
      Serial.println("AT");
      // wait 1 sec for the answer, normally, it is very fast
      delay(1000);
      compt++;
      collation=collationOk(500,2000);
    }
    while (collation!=0 && compt<10);
    if (collation!=0){
      forcedShutdown();
      initialisation();
    }
  }
  //digitalWrite(Led,LOW);
}

// ******************************************************************************
// Configure l'envoi d'un message sur le port serie par le Satcom dès que la reception satellite change
// Normalement déjà inclus dans la configuration sauvegardée
// Délai maximum inclus sinon hard reboot
void configSatelliteAlert(){
  flagReception=0;
  Serial.println("AT+CIER=1,1,0");
  long collation = collationOk(200,10000);
  if (collation!=0){
    if (collation>0){
      smoothShutdown(); // Reboot normal car on a seulement une erreur
    }
    else if (collation==-1){
      forcedShutdown(); // Reboot violent car la commande ne répond pas
    }
    initialisation(); 
    configSatelliteAlert();
  }
}

// ******************************************************************************
void waitForSatelliteCover(long barres){
  // N'attend pas de check donc pas de code d'erreur non plus
  parseur();
  long attenteMaxi = 300000L; // temps d'attente maximal de 5 min
  long i=0L;
  long delai=2000L;
  // attendre reception satellite suffisante
  while (flagReception<barres && i*delai<attenteMaxi){
    delay(delai);
    parseur(); 
    i++;
  }
  if (flagReception<barres){
    smoothShutdown(); // Reboot propre car à priori répond aux commandes. Peut etre juste probleme de connection
    initialisation();
    configSatelliteAlert(); // Inutile car dans la config sauvegardee
    waitForSatelliteCover(barres);
  }
}


// ******************************************************************************
void writeBuffer(){
  Serial.print("AT+SBDWB=");
  Serial.println(bufferSize);
  long collation = collationReady(200,10000);
  if (collation!=0){
    if (collation>0){
      smoothShutdown(); // Reboot normal car on a seulement une erreur
    }
    else {
      forcedShutdown(); // Reboot violent car la commande ne répond pas
    }
    initialisation(); 
    configSatelliteAlert();
    writeBuffer();
  } else {
    serialSendMessageWithCTRL();
  }
}

// ******************************************************************************
// Lance une session SBD (envoie les messages en attente)
void SBDsession(long n){
  waitForSatelliteCover(MINIMAL_SATELLITE_COVER);
  Serial.println("AT+SBDIX");
  long collation = collationSBDIX();
  if (collation==-1){
    forcedShutdown();
    sendMessageViaSatcom(n+1); // Dans ce cas, on peut relancer du début...
  }
  // Si code erreur entre 0 et 4, c'est que c'est bon
  else if (n>40){ // La ça marche pas du tout donc on peut éteindre le module définitivement
    forcedShutdown();
    while(true){
      delay(10000000);
    }
  }
  else if (collation<=4){
    //digitalWrite(Led,HIGH); // Put the LED ON to tell that the module is working and tested
  }
  else if (collation>4 && collation <16){ // Erreurs envoyees par la Gateway
    // On refait 5 essais et on zappe le message en éteignant le modem
    if (n<5){
      SBDsession(n+1);
    }
    else {
      smoothShutdown();
    }
  }
  else {
    if (collation==16){
      smoothShutdown();
      delay(300000); // On attend 5 min, parce que là, ça pue, donc ça sert à rien de surconsommer
      sendMessageViaSatcom(n+1);
    }
    else if (collation <  34){
      // On fait 4 essais avant de zapper le message
      if (n<4){
        SBDsession(n+1);
      }
    }
    else if (collation == 34){
      Serial.println("AT*R1");
      long resolution = collationOk(200,5000);
      // Si la resolution a fonctionné, on refait 2 essais et on zappe le message
      if (resolution == 0 && n<5){
        SBDsession(n+1);
      }
      else if (n<2){ // Si la resolution n'a pas fonctionné, on reboote le modem
        smoothShutdown();
        sendMessageViaSatcom(n+1);
      }
    }
    else if (collation == 35){
      if (n<4){
        delay(5000);
        SBDsession(n+1);
      }
    }
    else if (collation == 1000){ // Si le AT+SBDIX a été collationné par une ERREUR
      if (n<5){
        SBDsession(n+1);
      }
    }
    else {
      if (n<1){
        SBDsession(n+1);
      }
      else if (n<2){
        smoothShutdown();
        sendMessageViaSatcom(n+1);
      }
    }
  }
}
// ******************************************************************************
// Fonction qui relance le Satcom en cas de plantage
void forcedShutdown(){
  Serial.println("AT*F");
  delay(5000);
  digitalWrite(OnOff,LOW);
  delay(2500); // On attend un peu plus de 2 secondes pour relancer (cf datasheet)
}

// Fonction qui relance le Satcom en cas de plantage
void smoothShutdown(){
  Serial.println("AT*F");
  long delai=200; // Delai entre 2 essais
  long timeMax=10000; //Delai d'attente max
  long i=0; // Compteur de cycle
  while(parseur() && !flagOk && i*delai<timeMax){
    delay(200);
    i++;
  }
  digitalWrite(OnOff,LOW);
  delay(2500); // On attend un peu plus de 2 secondes pour relancer (cf datasheet)
}

// ******************************************************************************
// Fonction qui attend la collation  OK.
// Renvoie -1 si timeout
// Renvoie 0 si collation par un ok
// Renvoie un code d'erreur (positif) sinon
long collationOk(long delai, long timeMax){
  flagOk=false;
  long i=0;
  long erreur=0;
  do {
    erreur = parseur();
    delay(delai);
    i++;
  } 
  while (!flagOk && erreur==0 && i*delai<timeMax);
  if (erreur!=0){
    return erreur;
  }
  else if ( i*delai>=timeMax){
    return -1;
  }
  else return 0;
}

// ******************************************************************************
// Fonction qui attend la collation  READY.
// Renvoie -1 si timeout
// Renvoie 0 si collation par un ok
// Renvoie un code d'erreur (positif) sinon
long collationReady(long delai, long timeMax){
  flagReady=false;
  long i=0;
  long erreur=0;
  do {
    erreur = parseur();
    delay(delai);
    i++;
  } 
  while (!flagReady && erreur==0 && i*delai<timeMax);
  if (erreur!=0){
    return erreur;
  }
  else if ( i*delai>=timeMax){
    return -1;
  }
  else return 0;
}

// Fonction qui attend la collation  +SBDIX.
// Renvoie -1 si timeout
// Renvoie le MO status sinon
long collationSBDIX(){
  flagSBDIX=false;
  long i=0;
  long erreur=0;
  long timeMax=20000;
  long delai=500;
  do {
    erreur = parseur();
    delay(delai);
    i++;
  } 
  while (!flagSBDIX && erreur==0 && i*delai<timeMax);
  if (!flagSBDIX){
    return 1000; // Renvoie un gros code d'erreur !
  }
  else if (i*delai>=timeMax){
    return -1;
  }
  else return erreur;
}


// Parseur de commande AT reçue
// Renvoie 0 si pas d'erreur, sinon le code d'erreur correspondant
// Lit les collations "OK"
// Lit changement de reception "+CIEV:0,n" avec n=0..5
int parseur(){
  String mot="";
  while (Serial.available()>0){
    char c = Serial.read();
    // Enlever les espaces et les sauts de ligne et le retour chariot
    while(Serial.available()>0 && (c==' ' || c=='\n' || c=='\r')){
      c=Serial.read();
    }
    //Décole le mot, jusqu'à '\n', '\r' ou : ou la fin du buffer
    String mot="";
    while (Serial.available()>0 && (c != '\n') && (c!=' ') && (c!=':') && (c!='\r')){
      mot+=c;
      c=Serial.read();
    }
    // Si le caractère courant est un \n, un \r, un espace ou une fin de ligne, on regarde si ERROR, READY ou OK
    if (Serial.available()==0 || c=='\n' || c==' ' || c=='\r'){
      if (mot.equalsIgnoreCase("ERROR")){
        return 1;
      }
      else if (mot.equalsIgnoreCase("OK")){
        flagOk = true;
      }
      else if (mot.equalsIgnoreCase("READY")){
        flagReady = true;
      }
      else if (mot.equalsIgnoreCase("0") || mot.equalsIgnoreCase("1")||mot.equalsIgnoreCase("2")||mot.equalsIgnoreCase("3")){
        return mot.charAt(0)-48;
      }
    }
    else if (c==':'){
      if (mot.equalsIgnoreCase("+CIEV")){
        c=Serial.read();
        c=Serial.read();
        flagReception=int(Serial.read())-48;
      }
      else if (mot.equalsIgnoreCase("+SBDIX")){
        int codeErreur= Serial.read();
        if (codeErreur==' '){
          codeErreur=0;
        }
        else {
          codeErreur-=48; // Pour convertir le code caractère en nombre
        }
        c=Serial.read();
        // On lit le deuxième chiffe du code d'erreur
        codeErreur=codeErreur*10+c-48;
        // On termine la ligne d'instruction
        while (Serial.available() && c !='\n' && c!='\r'){
          Serial.read();
        }
        flagSBDIX=true;
        return codeErreur;
      }
    }
  }
  return 0;
}

void serialSendMessageWithCTRL(){
  word check = checksum();
  byte checkBytes[2];
  checkBytes[0]=highByte(check);
  checkBytes[1]=lowByte(check);
  boolean res1=serialSendWithCTRL(bufferMessage,bufferSize);
  boolean res2=serialSendWithCTRL(checkBytes,2);
  if (!res1 && !res2){
    forcedShutdown();
    initialisation();
    configSatelliteAlert();
    waitForSatelliteCover(MINIMAL_SATELLITE_COVER);
    sendMessageViaSatcom(0);
  }
  flagOk=false;
  int res=parseur();
  while (flagOk!=0 && res==0){
    parseur();
    delay(500);
  }
  if (res!=0){ // Code d'erreur qui signifie que le message n'est pas passé
    forcedShutdown();
    initialisation();
    configSatelliteAlert();
    waitForSatelliteCover(MINIMAL_SATELLITE_COVER);
    sendMessageViaSatcom(0);
  }
}

// Send true if the sending works
boolean serialSendWithCTRL(byte message[], int messageLength){
  int i=0;
  long nbLoop=0;
  while (i<messageLength && nbLoop<1000){
    if (digitalRead(Cts)==LOW){
      Serial.print(message[i],BYTE);
      i++;
    } else {
      delay(100);
    }
    nbLoop++;
  }
  return nbLoop!=1000; 
}

word checksum(){
  word result=0;
  for (int i=0;i<bufferSize;i++){
    result+=bufferMessage[i];
  }
  return result;
}

