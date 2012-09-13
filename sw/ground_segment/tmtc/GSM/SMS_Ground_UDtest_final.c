/**********	SMS_Ground Uplink & Downlink	***********/
/********** */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <glib.h>
#include <gtk/gtk.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <stropts.h>


#define CTRLZ 		0x1A

#define _ATTENTE			0
#define	_AT					1
#define	_CMGF				2
#define	_CNMI				3
#define	_CPMS				4
#define _FINCONF			5
#define _RECEPTION_SMS		6
#define	_SUPPR_SMS			7
#define _SEND_MSG			8

int fct_en_cours = 0, wait_reponse=0, wait_prompt = 0;

int 	index_msg_precedent, longueur_derniere_commande;
char 	num_avion[15];
struct 	termios option;
FILE	*log_SMS;

int ligne_occupee = 0, GSM_reponse = 0;
char *chaine_recue, GSM_Line[160], data_to_send[160];
char info_SMS_recu[20];
int envoi_sms_en_cours = 0, reception_sms_en_cours = 0;

// Données relatives au dernier SMS recu
int index_msg;
char flag[11];
char expediteur[12];
char dateheure[20];
char data[160];

typedef struct pile
{
	char text[100];
	struct pile *prec;
} pile ;



int 	config_port_serie(void);

gboolean 	init(gpointer handle);
gboolean 	lecture_port(gpointer handle);
void 	Traitement_reponse(int handle);

void 	Send(int srl_handle, char chaine_envoyee[]);

void 	Send_AT(int handle);
void 	Send_cmgf(int handle);
void 	Send_cnmi(int handle);
void 	Send_cpms(int handle);

void 	Send_Msg_part1(int handle);
void 	Send_Msg_part2(int handle);

void 	Reception_SMS(int handle);
void 	Reception_SMS_Continue(int handle);
void 	Recuperation_SMS(int srl_handle);
int 	Verification( char* num_expediteur);
void 	Remplissage_SMS(char reponse_module[]);
int 	Check_index( int index_test );
void 	decoupage(char message_complet[]);
void 	Save_SMS_In_Log();
void 	Suppr_SMS(int srl_handle, int index_test);

int 	recherche_caractere(char* chaine, char caractere, int nb_occ);
int 	locate(char chaine_a_trouver[], char chaine_source[]);
void 	Extraction(char source[], char char_dbt, int nb_occurrence1, int decalage1, char char_fin, int nb_occurrence2, int decalage2, char destination[]);
int str_istr (const char *cs, const char *ct);

void 	Push(pile **, char message[]);
char 	*Pop(pile **);
void 	Clear(pile **);
int 	Length(pile *p);
void 	View(pile *);
//static void 	Jump_To_Block(IvyClientPtr app, void *user_data, int argc, char *argv[]);
void 	*Envoi_SMS_Uplink();

pile *MaPile = NULL;

typedef enum {INIT, AT, CMGF, SMSMODE, CNMI, CPMS, ERREUR} Etat_Liste;

// Informations extraites du SMS recu de l'avion
char extr_gps_utm_east[15], extr_gps_utm_north[15], extr_gps_course[15], extr_gps_alt[15], extr_gps_gspeed[15], extr_gps_climb[15], extr_vsupply[15], extr_autopilot_flight_time[15], extr_qualite_signal_GSM[10];

char reponse_attendue[20];
int prompt_recu = 0;

// Paramétrage du port série
int config_port_serie()
{
	int handle;

	char uart[20]={"/dev/ttyUSB0"};

	//Ouverture du port série
	handle=open ( uart, O_RDWR | O_NOCTTY | O_NDELAY );

	if ( handle<0 )
	{
		perror ( "serial port fail to open\n" );
		exit(0);
	}
	else
	{
		if(fcntl ( handle, F_SETFL, FNDELAY)<0)
			perror("fcntl");
	}


	//settings for the uart
	tcgetattr ( handle, &option );
	cfmakeraw ( &option );
	cfsetspeed ( &option, B9600 );
	option.c_cflag |= ( CLOCAL | CREAD | CS8 );
	option.c_cflag &= ( ~PARENB & ~CSTOPB & ~CRTSCTS & ~CSIZE);

	tcsetattr ( handle, TCSANOW, &option );

	return handle;
}


// Fonction d'envoi d'une chaine de caractères sur l'UART
void Send(int srl_handle, char chaine_envoyee[])
{
	char* commande_envoyee = (char*)malloc(sizeof(char)*160);

	printf("Envoi : %s\n", chaine_envoyee);

	while (ligne_occupee == 1);

	ligne_occupee = 1;
	sprintf(commande_envoyee, "%s\r\n", chaine_envoyee);
	longueur_derniere_commande = strlen(chaine_envoyee);

  int len = strlen(commande_envoyee)*sizeof(char);
	if (write(srl_handle, commande_envoyee, len) != len) {
    printf("Erreur de transmission sur UART\n");
  }
	free(commande_envoyee);
}


// Première partie de la fonction d'envoi d'un SMS
void Send_Msg_part1(int handle)
{
	char* commande_envoyee = (char*)malloc(sizeof(char)*22);

	while ((reception_sms_en_cours == 1) || (envoi_sms_en_cours == 1));

	sleep(5);

	envoi_sms_en_cours = 1;
	sprintf(commande_envoyee, "AT+CMGS=\"%s\"", num_avion);
	wait_prompt = 1;
	fct_en_cours = _SEND_MSG;
	Send(handle, commande_envoyee);

	free(commande_envoyee);
}

// Deuxième partie de la fonction d'envoi d'un SMS
void Send_Msg_part2(int handle)
{
	char* info_envoyee = (char*)malloc(sizeof(char)*165);
	sprintf(info_envoyee, "%s %c", data_to_send, CTRLZ);
	wait_reponse = 1;

	Send(handle, info_envoyee);

	envoi_sms_en_cours = 0;
	free(info_envoyee);
}


gboolean init(gpointer handle)
{
	Send_AT((int)handle);
	return FALSE;
}

void Send_AT(int handle)
{
	strcpy(reponse_attendue, "OK");
	fct_en_cours = _AT;
	wait_reponse = 1;
	Send(handle, "AT");
}

void Send_cmgf(int handle)
{
	strcpy(reponse_attendue, "OK");
	fct_en_cours = _CMGF;
	wait_reponse = 1;
	Send(handle, "AT+CMGF=1");
}

void Send_cnmi(int handle)
{
	strcpy(reponse_attendue, "OK");
	fct_en_cours = _CNMI;
	wait_reponse = 1;
	Send(handle, "AT+CNMI=1,1,0,0,0");
}

void Send_cpms(int handle)
{
	strcpy(reponse_attendue, "+CPMS:");
	fct_en_cours = _CPMS;
	wait_reponse = 1;
	Send(handle, "AT+CPMS=\"SM\"");
}

void Send_fin_config(int handle)
{
	// Envoi d'un SMS de vérification au démarrage
	fct_en_cours = _FINCONF;

	sprintf(data_to_send, "Configuration correcte du module GSM");
	Send_Msg_part1(handle);
}

// Callback effectuant la lecture des données sur l'UART
gboolean lecture_port(gpointer handle)
{
	char *caracteres_recus = (char*)malloc(sizeof(char)*450);
	int nb_octets_dispos, flush;

	chaine_recue = (char*)malloc(sizeof(char)*450);

	ioctl((int) handle, FIONREAD, &nb_octets_dispos);

  if (nb_octets_dispos !=0)
  {
    if (read((int) handle, caracteres_recus, nb_octets_dispos) > 0) {

      strcpy(chaine_recue, caracteres_recus);
      flush = tcflush((int) handle, TCIOFLUSH);

      Traitement_reponse((int) handle);
    }
	}

	caracteres_recus[0] = '\0';
	chaine_recue[0] = '\0';
	free(chaine_recue);
	free(caracteres_recus);
	return TRUE;
}


// fonction de traitement des données recues sur l'UART
void Traitement_reponse(int handle)
{
	char buffer[450];
	strcpy(buffer, chaine_recue);
	chaine_recue[0] = '\0';
	printf("Recu : %s\n", buffer);


//	si on a recu un nouveau SMS
	if(locate("+CMTI:", buffer) == 1)
	{
		while((envoi_sms_en_cours == 1) || (reception_sms_en_cours));

		reception_sms_en_cours = 1;

		printf("Nouveau SMS recu\n");
		Reception_SMS(handle);
	}
	else if ((wait_prompt == 1) && (locate(">", buffer) == 1))
	{
		printf("Suite de l'envoi du SMS\n");
		wait_prompt = 0;
		ligne_occupee = 0;
		Send_Msg_part2(handle);
	}
	else if (wait_reponse == 1)
	{
		ligne_occupee = 0;

		if(locate(reponse_attendue, buffer) == 1)
		{
			GSM_reponse = 1;
			wait_reponse = 0;
			reponse_attendue[0] = '\0';
		}
		else
			GSM_reponse = 0;

		// Suite des opérations...
		switch(fct_en_cours)
		{
			case _AT : 	if(GSM_reponse == 1)
							Send_cmgf(handle);
						else
							Send_AT(handle);
			break;

			case _CMGF : if(GSM_reponse == 1)
						{
							Send_cnmi(handle);
						}
						else
							Send_cmgf(handle);
			break;

			case _CNMI : if(GSM_reponse == 1)
							Send_cpms(handle);
						else
							Send_cnmi(handle);
			break;

			case _CPMS : if(GSM_reponse == 1)
						{
							printf("Configuration correcte du module GSM\n");
							//Send_fin_config(handle);
						}
						else
							Send_cpms(handle);
			break;

			case _SUPPR_SMS : if(GSM_reponse == 1)
								reception_sms_en_cours = 0;
			break;

			case _FINCONF :
			break;

			case _RECEPTION_SMS : 	if(GSM_reponse == 1)
									{
										strcpy(GSM_Line, buffer);
										Reception_SMS_Continue(handle);
									}
			break;

			default : break;
		}
	}

	buffer[0] = '\0';
  fflush(stdout);
}



// Première partie de la fonction de réception de SMS
void Reception_SMS(int handle)
{
	Recuperation_SMS((int) handle);
}

// Deuxième partie de la fonction d'envoi d'un SMS
void Reception_SMS_Continue(int handle)
{
	char buffer_SMS_recu[250];

	strcpy(buffer_SMS_recu, GSM_Line);

	Remplissage_SMS(buffer_SMS_recu);

	if (Verification(expediteur) == 1)// Le SMS provient bien de l'avion
	{
		printf("Nouveau message recu de l'avion\n");

		/* Stockage du message dans un fichier de log */
		Save_SMS_In_Log();

		/* Vérification de l'index (pour ne pas en laisser passer un)*/
		if (Check_index(index_msg) == 0) /*on a laissé passer qqch*/
		{
			printf("Attention perte d'un ou plusieurs messages...\n");
		}

		printf("Contenu du message :%s\n", data);



		// Découpage prévu du SMS recu
		decoupage(data);

		/* Envoi sur le bus Ivy */
		IvySendMsg("16 GPS 3 %s %s %s %s %s %s 0 335297960 31 0", extr_gps_utm_east, extr_gps_utm_north, extr_gps_course, extr_gps_alt, extr_gps_gspeed, extr_gps_climb);
    IvySendMsg("16 FBW_STATUS 0 1 %s 0",extr_vsupply);


		/* Suppression du message de la carte SIM */
		Suppr_SMS(handle, index_msg);
	}

  fflush(stdout);
}


// Fct renvoyant l'indice de dbt de la chaine ct ds cs
int str_istr (const char *cs, const char *ct)
{
   int index = -1;

   if (cs != NULL && ct != NULL)
   {
      char *ptr_pos = NULL;

      ptr_pos = strstr (cs, ct);
      if (ptr_pos != NULL)
      {
         index = ptr_pos - cs;
      }
   }
   return index;
}

// Fonction indiquant la présence de la chaine_a_trouver dans la chaine_source
int locate(char chaine_a_trouver[], char chaine_source[])
{
	int i=0, indice_dbt = -1;
	char buffer[20];

	if ( strstr(chaine_source, chaine_a_trouver) != NULL)
	{
		if (strcmp(chaine_a_trouver, "+CMTI:") == 0)
		{
			if((indice_dbt = str_istr(chaine_source, "+CMTI:")) == -1)
				return 0;
			else
			{
				for (i = 0; i < 15; i++)
				{
					buffer[i] = chaine_source[i+indice_dbt];
				}
				strcpy(info_SMS_recu, buffer);
			}
		}
		return 1;
	}
	else
		return 0;
}



/* Fonction decoupant le message complet en plusieurs chaines de caracteres relatives à chaque info*/
void decoupage( char message_complet[])
{
	char data_to_cut[160];
	printf("dbt fct decoupage\n");
	strcpy(data_to_cut, data);

	Extraction(data_to_cut, '\n', 1, 1, ' ', 1, 0, extr_gps_utm_east);
	Extraction(data_to_cut, ' ', 1, 1, ' ', 1, 0, extr_gps_utm_north);
	Extraction(data_to_cut, ' ', 1, 1, ' ', 1, 0, extr_gps_course);
	Extraction(data_to_cut, ' ', 1, 1, ' ', 1, 0, extr_gps_alt);
	Extraction(data_to_cut, ' ', 1, 1, ' ', 1, 0, extr_gps_gspeed);
	Extraction(data_to_cut, ' ', 1, 1, ' ', 1, 0, extr_gps_climb);
	Extraction(data_to_cut, ' ', 1, 1, ' ', 1, 0, extr_vsupply);
	Extraction(data_to_cut, ' ', 1, 1, ' ', 1, 0, extr_autopilot_flight_time);
	Extraction(data_to_cut, ' ', 1, 1, '\r', 1, 0, extr_qualite_signal_GSM);

	printf("Message :\n utm_east %s\n utm_north %s\n course %s\n alt %s\n speed %s\n climb %s\n bat %s\n flight_time %s\n signal %s\n", extr_gps_utm_east, extr_gps_utm_north, extr_gps_course, extr_gps_alt, extr_gps_gspeed, extr_gps_climb, extr_vsupply, extr_autopilot_flight_time, extr_qualite_signal_GSM);
  fflush(stdout);
}


/*Vérification du numéro de l'expéditeur du message */
int Verification( char* num_expediteur)
{
	if (strncmp(num_expediteur, num_avion, 12) == 0)  // Le message provient bien de notre drone
		return 1;
	else
		return 0;
}


// Vérification de l'index du message recu
int Check_index( int index_test )
{
	/* si l'index du nouveau msg est égal à l'index du precedent msg recu c'est bon
	   (on aura supprimé le précédent msg de la mémoire de la carte SIM)*/

	if (index_test == index_msg_precedent)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


// fonction supprimant le SMS (repéré par son index) de la mémoire
void Suppr_SMS(int srl_handle, int index_)
{
	char demande_suppr[10];

	sprintf(demande_suppr, "AT+CMGD=%d", index_); // AT+CMGD=1,4  --> erase all messages
	sprintf(reponse_attendue, "OK");
	wait_reponse = 1;
	fct_en_cours = _SUPPR_SMS;

	Send(srl_handle, demande_suppr);
}


// FOnction de sauvegarde des SMS recus dans un fichier de log
void Save_SMS_In_Log()
{
	char ecriture_log[500];
	time_t 	moment_reception = time(NULL);
	int longueur_ecrite;

	sprintf(ecriture_log, "%sIndex : %d\nFlag : %s\nContenu du message : %s\n\n", ctime(&moment_reception), index_msg, flag, data);

	longueur_ecrite = strlen(ecriture_log);

	if(( log_SMS = fopen("/home/cocoleon/dev/paparazzi3/log_SMS", "a+")) == NULL )
	{
		perror("Erreur, impossible d'ouvrir le fichier de log :");
	}
	else
	{
		fwrite(ecriture_log, sizeof(char), longueur_ecrite, log_SMS);

		fclose(log_SMS);
	}
}


// Fonction effectuant la demande de lecture d'un SMS
void Recuperation_SMS(int handle)
{
	char demande_lecture[10], chaine_extraite[4];

	Extraction(info_SMS_recu, ',', 1, 1, '\0', 1, 0, chaine_extraite);
	index_msg = atoi(chaine_extraite);
	printf("index du message : %d\n", index_msg);

	sprintf(demande_lecture, "AT+CMGR=%d", index_msg);
	strcpy(reponse_attendue, "+CMGR:");
	wait_reponse = 1;
	fct_en_cours = _RECEPTION_SMS;
	Send(handle, demande_lecture);
}


// Fonction d'extraction a partir de la chaine source de la ssous-chaine comprise entre char_dbt et char_fin, stockée dans destination
void Extraction(char source[], char char_dbt, int nb_occurrence1, int decalage1, char char_fin, int nb_occurrence2, int decalage2, char destination[])
{
	int indice_debut, indice_fin, i, rch1, rch2;

  	rch1 = recherche_caractere(source, char_dbt, nb_occurrence1);

	if(rch1 == -1)
		strncpy(destination, "0", 1);
	else
	{

	  indice_debut = rch1 + decalage1;
	  for(i=0; i<strlen(source); i++)
		source[i] = source[i+indice_debut];

		rch2 = recherche_caractere(source, char_fin, nb_occurrence2);

	  if(rch2 == -1)
	  		strncpy(destination, "0", 1);
	  else
	  {
	  	indice_fin = rch2 + decalage2;
	  	strncpy(destination, source, indice_fin);
	  	destination[indice_fin] = '\0';
	  }
	}
}

// Fonction remplissant les différents champs relatifs au dernier SMS recu
void Remplissage_SMS(char reponse_module[])
{
	char buffer[250];

	strcpy(buffer, reponse_module);

	printf("Index du message : %d\n", index_msg);

	Extraction(buffer, '"', 1, 1, '"', 1, 0, flag);
	printf("Flag : %s\n", flag);

	Extraction(buffer, '"', 2, 1, '"', 1, 0, expediteur);
	printf("Expéditeur : %s\n", expediteur);

 	Extraction(buffer, '"', 2, 1, '"', 1, 0, dateheure);
 	printf("Dateheure : %s\n", dateheure);

	Extraction(buffer, '"', 1, 3, '\0', 1, -3, data);
	printf("Contenu du message : %s\n", data);

  fflush(stdout);
}

int recherche_caractere(char* chaine, char caractere, int nb_occ)
{
	int nb_trouve = 0, i=0, longueur_chaine = strlen(chaine);

	while(i<longueur_chaine && nb_trouve < nb_occ)
	{
		if(chaine[i] == caractere)
		{
			nb_trouve ++;
		}
		i++;
	}

	if (i<=longueur_chaine)
  	return i-1;
  else
  	return -1;
}




/******************************************************************************/
/*********FONCTIONS D'ENVOI D'UN SMS DEPUIS LA STAION SOL****************/
/******************************************************************************/

void Push(pile **p, char message[])
{
	pile *element = malloc(sizeof(pile));
	if(!element) exit(1);     /* Si l'allocation a �chou�e. */
	strcpy(element->text, message);
	element->prec = *p;
							*p = element;       /* Le pointeur pointe sur le dernier �l�ment. */
}

/*************************************************************************/

char *Pop(pile **p)
{
	static char message[100];
	pile *tmp;
	//     if(!*p) return -1;     /* Retourne -1 si la pile est vide. */
	tmp = (*p)->prec;
	strcpy(message, (*p)->text );
	free(*p);
						*p = tmp;       /* Le pointeur pointe sur le dernier �l�ment. */
						return message;     /* Retourne le message soutir� de la pile. */

}

/*************************************************************************/

void Clear(pile **p)
{
	pile *tmp;
	while(*p)
	{
		tmp = (*p)->prec;
		free(*p);
		*p = tmp;
	}
}

/*************************************************************************/

int Length(pile *p)
{
	int n=0;
	while(p)
	{
		n++;
		p = p->prec;
	}
	return n;
}

/*************************************************************************/

void View(pile *p)
{
	while(p)
	{
		printf("%s\n",p->text);
		p = p->prec;
	}
}

/*************************************************************************/
//static void Jump_To_Block(IvyClientPtr app, void *user_data, int argc, char *argv[])
//{
//	char message_complet[100];
//
//	sprintf(message_complet,"%s JUMP_TO_BLOCK %s %s\n", argv[0], argv[1], argv[2]);
//	printf(message_complet,"%s JUMP_TO_BLOCK %s %s\n", argv[0], argv[1], argv[2]);
//	Push(&MaPile,message_complet);
//  fflush(stdout);
//}

/*************************************************************************/
void * Envoi_SMS_Uplink(void* hdl)
{
	while(1)
	{
		sleep(1);

		if(!MaPile) {} // La pile est vide .
		else
		{
			strcpy(data_to_send, Pop(&MaPile));
			Send_Msg_part1((int) hdl);
		}
	}
	pthread_exit(NULL);
}



int main( int argc, char** argv)
{
	int srl_handle = config_port_serie();
	//pthread_t idthread;

	strcpy(num_avion, "+33640286564");

	//if(pthread_create(&idthread,NULL,Envoi_SMS_Uplink, (void*)srl_handle)!=0)// creation thread envoi_SMS_Uplink
	//{
	//	printf("Erreur creation du thread envoi_SMS_Uplink");
	//	exit(1);
	//}

	GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

	//gtk_init(&argc, &argv);

	IvyInit ("SMS_GROUND", "SMS_GROUND READY", NULL, NULL, NULL, NULL);
	IvyStart("127.255.255.255");

	//IvyBindMsg(Jump_To_Block, NULL, "^(\\S*) JUMP_TO_BLOCK (\\S*) (\\S*)");

	g_timeout_add(300, lecture_port, (gpointer)srl_handle);

	g_timeout_add(400, init, (gpointer)srl_handle);


	g_main_loop_run(ml);


	//Clear(&MaPile);    /* Vider la pile avant de quitter. */

	//pthread_join(idthread,NULL);

	return 0;
}
