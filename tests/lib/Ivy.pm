#
#	Ivy, Perl interface
#
#	Copyright 1997-2009
#	Centre d'Études de la Navigation Aérienne
#
#	Authors: Alexandre Bustico <alexandre.bustico@cena.fr>
#		Stéphane Chatty <chatty@intuilab.com>
#		Hervé Damiano <herve.damiano@aviation-civile.gouv.fr>
#		Christophe Mertz <mertz@intuilab.com>
#
#	All functions
#
#	$Id: Ivy.pm 3491 2011-06-20 09:35:58Z bustico $
#
#	This program is free software; you can redistribute it and/or
#	modify it under the terms of the GNU LGPL Libray General Public License
#	as published by the Free Software Foundation; either version 2
#	of the License, or (at your option) any later version.
#
#	This program is distributed in the hope that it will be useful,
#	but WITHOUT ANY WARRANTY; without even the implied warranty of
#	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#	GNU Library General Public License for more details.
#
#	You should have received a copy of the GNU Library General Public License
#	along with this program; if not, write to the Free Software
#	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA,
#	or refer to http://www.gnu.org/copyleft/lgpl.html
#
##################################################################

package Ivy ;

use Sys::Hostname;
use IO::Socket;
use strict;
use Time::HiRes qw(gettimeofday);
use Carp;
use IO::Socket::Multicast;
use File::Temp;

use vars qw($VERSION);
use Fcntl qw(F_GETFL F_SETFL O_NONBLOCK);
use Socket qw(TCP_NODELAY);

# to compute the VERSION from the CVS tag (or if no tag, as the cvs file revision)
my $TAG= q$Name:  $;
my $REVISION = q$Revision: 3491 $ ;
$VERSION = '1.49' ; # for Makefile.PL
($VERSION) = $TAG =~ /^\D*([\d_]+)/ ;
if (defined $VERSION and $VERSION ne "_") {
    $VERSION =~ s/_/\./g;
}
else {
    $VERSION = $REVISION;
}

#############################################################################
####                    PROTOTYPES                                      #####
#############################################################################
sub init;		# methode de classe, permet de renseigner
			# tous les parametres globaux. Ces parametres
			# seront utilises par new si ils ne sont pas
			# donnes lors de l'appel de new.


sub new ($%);           # verifie la validite de tous les parametres,
			# cree et retourne un objet Ivy. Les parametres
			# appName, networks, messWhenReady, peuvent
			# etre donnes, meme si ils ont deja ete
			# donnes dans init, dans ce cas, ce sont ceux
			# de new qui prevalent

sub start;		# debut de l'integration au bus :
		 	# - cree la socket d'application, recupere le no
			#   de port
		 	# - cree la socket supervision
		 	# - envoie le "no de port"
		 	# - bind le file descriptor de la socket de
			#   supervision a la fonction getBonjour pour
			#   traiter les bonjours
		 	# - bind le fd de connection sur la fonction
			#   getConnections
			#   pour etablir les connections "application"

sub DESTROY ($);		# - envoie un BYE et clôt les connections

sub bindRegexp ($$$;$$) ;   # permet d'associer une regexp avec un callBack
			# ou d'annuler une precedente association
sub bindRegexpOneShot ($$$); #  permet d'associer une regexp avec un callBack avec
			     # desabonnement automatique après reception du premier
			     # message qui matche

sub changeRegexp ($$$); # permet de changer une regexp d'un abonnement
			# precedemment fait avec bindRegexp
			
sub bindDirect;    # permet d'associer un identifiant de msg direct
			# avec une fonction de callBack, ou de l'annuler

sub sendMsgs;	# envoie une liste de messages
sub sendAppNameMsgs;	# envoie une liste de messages precedes
				# du nom de l'application
sub sendDirectMsgs; # envoie une liste de messages directs a une appli
sub sendDieTo;	 # envoie un <<kill>>  a une appli
sub ping ($$\&);		  # teste qu'une appli soit encore vivante
sub dumpTable ($$\&);		  # demande à une appli de dumper sa table de regexps dans
				  # un fichier à des fins de debug
sub mainLoop (;$);        # la mainloop locale (sans tk)
sub stop ();		# methode de classe : on delete les bus, mais
			# on reste dans la mainloop
sub exit ();		# methode de classe : on delete tous les
			# bus (donc on ferme proprement toutes les
			# connexions).
			# Si on est en mainloop locale  on sort de la
			# mainloop, le code qui suit l'appel mainLoop
			# sera execute.
			# par contre si on est en mainloop Tk,
			# il faut en plus detruire la mainwindow
			# pour sortir de la mainloop;
sub after ($$;$);		# temps en millisecondes, callback
sub repeat ($$;$);	# temps en millisecondes, callback
sub afterCancel ($;$); # l'id d'un cancel ou d'un repeat
sub afterResetTimer ($;$); # pour ré-armer un timer non-encore déclenché à sa valeur initiale
sub fileEvent ($$;$); # associe un fd a un callback pour la mainloop locale
sub getUuid ($); # rend un identifiant unique d'application sur le bus, utile
		 # pour faire des genres de rpc avec un abonnement desabonnement dynamique

################ PRIVEE ####################################################
sub _getBonjour ($); # lit le (ou les) bonjour(s) sur le canal de supervision
		# et se connecte, verifie qu'il ne se reponds pas lui
		# meme, ni qu'il ne repond pas a un service deja connecte

sub _getConnections ($); # est appele lors d'une demande de connection :
		    # accepte la connection et mets a jour  @sendRegList
		    # rajoute le fd du canal aux fd a scruter dans la
		    # boucle d'evenements

sub _getMessages ($$);   # est appele lorqu'un message arrive

sub _sendWantedRegexp ($$); # envoie les regexp  a l'appli distante

sub _sendLastRegexpToAllreadyConnected ($$) ; # envoie la derniere regexp
					    # pushee dans @recCbList
					    # a toutes les applis deja
					    # connectees
sub _removeFileDescriptor ($$$);    # on vire un fd et les structures associees
sub _sendErrorTo ($$$); #(fd, error) envoie un message d'erreur a un fd
sub _sendDieTo ($$); #(fd) envoie un message de demande de suicide a un fd
sub _sendMsgTo ($$\$);  # (fd, message)
sub _pong ($$$); # (fd)
sub _dumpTable ($$$); # (fd)
sub _tkFileEvent ($$); # associe un fd a un callback pour la mainloop tk
sub _scanAfter () ; # parse si il faut appeler un callback associe a un after
sub _myCanRead (); # interface au select
sub _scanConnStatus ($$$@); # verifie les connections effectuees et
		     # appelle la fonction $statusFunc
sub _inetAdrByName ($$); # transforme une adresse inet native en chaine
			 # $host:$port
sub  _getHostByAddr ($);
sub _toBePruned ($$$);
sub _parseIvyBusParam ($); # prends une adresse de bus de la forme
			   # 143.196.53,DGAC-CENATLS:2010 et
			   # renvoie une liste de deux elements :
		           # un numero de port et une ref sur une
			   # liste d'adresses addr_inet

sub _substituteEscapedChar ($$); #permet de transformer une regexp etendue
				# 'perl' en regexp de base

sub _callCongestionCb ($$$); # appelle la callback de notification de congestion,
			    # si elle a été définie par l'utilisateur

sub _getNameByFileDes ($$); # retourne le nom de l'appi en fonction du filedes
			   # de la socket
sub _univSend ($$$);       # effectue les send de manière bloquante ou non bloquante
			    # et accumule les messages si la socket est bloquée

sub _regexpGen ($$$);		# routines for generating regexps wich matches
sub _strictPosRegexpGen ($$$$); # numerical interval using the special syntax
sub _genAtRank ($$$);		# (?I-20#-10) or (?I-20#-10i)
sub _genPreRank ($$$);
sub _genRank ($$$);
sub _genPostRank ($);
sub _nextMax ($$);
sub _max ($$);
sub _min ($$);

#############################################################################
####                    CONSTANTES                                      #####
#############################################################################
use constant MSG_FMT => "%d %d\002%s\n";

# par defaut, on diffuse le bonjour en local
use constant BROADCAST_ADDRS => "127.255.255.255" ;
use constant BROADCAST_PORT => "2010";

use constant BYE	 => 0;
use constant REGEXP	 => 1;
use constant MSG	 => 2;
use constant ERROR	 => 3;
use constant DELREGEXP	 => 4;
use constant ENDREGEXP	 => 5;
use constant APP_NAME    => 6;
use constant DIRECT_MSG	 => 7;
use constant DIE	 => 8;
use constant PING	 => 9;
use constant PONG	 => 10;
use constant DUMP_TABLES => 11;
use constant DUMP_TABLES_FILE => 12;
use constant IVY_PROTOCOLE_VERSION => 3;

use constant AFTER	=> 0;
use constant REPEAT	=> 1;
use constant TK_MAINLOOP => 0;
use constant LOCAL_MAINLOOP => 1;
use constant CALL_BY_VALUE => 0;
use constant CALL_BY_REF => 1;
use constant BIND_ONCE => 2;
use constant MAX_TIMOUT => 1000;


#  TCP_NODELAY is for a specific purpose; to disable the Nagle buffering
#    algorithm. It should only be set for applications that send frequent
#    small bursts of information without getting an immediate response,
#    where timely delivery of data is required (the canonical example is
#    mouse movements).
#    Since Ivy is most of the time used to send events, we will priviligiate
#    lag over throughtput, so  _TCP_NO_DELAY_ACTIVATED is set to 1
use constant TCP_NO_DELAY_ACTIVATED => 1;


# pour pouvoir employer les regexps perl. Attention lors de l'utilisation
# ne pas mettre un \n dans une chaine entre "" car l'\n sera interprete.
use constant REG_PERLISSISME => ('w' => '[a-zA-Z0-9_]',
				 'W' => '[^a-zA-Z0-9_]',
				 's' => "[\t ]",
				 'S' => "[^\t ]",
				 'd' => '[0-9]',
				 'D' => '[^0-9]',
				 'n' => '', # Il ne faut pas mettre d'\n :
				            # c'est un delimiteur pour le bus
				 'e' => '[]') ;

#############################################################################
####                    VARIABLES de CLASSE                             #####
#############################################################################

# l'objet Ivy utilise par defaut quand le programmeur
# utilise le mode de compatibilite de la version 3, et ne
# manipule pas d'objets
my $globalIvy ;

# optimisation : si l'on connait les sujets des messages
# qu'on envoie, cette variable contient une liste de
# sujets qui doivent matcher les regexps d'abonnement
# pour que celle ci se soient pas eliminees
my @topicRegexps;

# les adresses de reseau sur lesquelles ont broadcaste
# suivies du No de port :
# exemples :	"143.196.1.255,143.196.2.255:2010"
#		"DGAC-CENATLS-PII:DGAC-CENATLS:2010"
#		":2010" <= dans ce cas c'est la valeur
# de reseau de broadcast par defaut qui est prise : 127.255.255.255
# c.a.d local a la machine
my $ivyBus ;

# le nom de l'appli pour le bus
my $appName ;

# message a envoyer a un canal lorsqu'on
# a recu le message endregexp.
my $messWhenReady ;

# fonction de cb appelee lorsque l'appli a recu l'ordre
# de quitter, on peut dans ce callback fermer
# proprement les ressources avant de sortir.
# ps : ne pas fasire d'exit dans le callback,
# c'est le bus qui s'en charge
my $onDieFunc;


# permet de donner des valeurs successives aux constantes permettant
# d'acceder aux differents champs de l'objet
my $constantIndexer =0;

# pointeur sur la fonction permettant d'associer
# des callbacks a un file desc, (ainsi que de les enlever)
my $fileEventFunc;

# dans le cas ou l'on soit dans une mainLoop
# locale, cette var pointe une un objet
# de type IO::Select, qui est l'ensemble des file descriptor
# des sockets que l'on scrute en lecture (attente des messages)
my $localLoopSelRead;

# dans le cas ou l'on soit dans une mainLoop
# locale, cette var pointe une un objet
# de type IO::Select qui est l'ensemble des file descriptor
# des sockets que l'on scrute en ecriture : les fd congestionnés
# des "slow agent" que l'on surveille pour ecrire dedans dès que 
# possible (mode non bloquant)
my $localLoopSelWrite;

# table d'ass. handle -> callback
my %localBindByHandle;

# table d'ass. fhd -> nom appli
my %nameByHandle;

# tableau d'ass [AFTER ou REPEAT,
#   timeTotal, deadLine, [callback, arg, arg, ...]]
my %afterList=();

my $afterId = 0;

# timeout le plus petit pour le select
my $selectTimout = MAX_TIMOUT;
my $loopMode;

# liste des bus actifs
my %allBuses = ();

# cache des nom retournés par gethostbyaddr pour _getHostByAddr
my %hostNameByAddr = ();

my $pingId = 1; # identifiant d'un ping (renvoyé par le pong)

#my $trace;

#############################################################################
####                    CLEFS DES VARIABLES D'INSTANCE                  #####
####									#####
#### l'objet Ivy sera 'blessed' sur une reference sur un array et non	#####
#### sur une table de hash comme pratique courament de facon a		#####
#### 1/ optimiser au niveau vitesse					#####
#### 2/ avoir des clefs sous forme de symboles (use constant...)	#####
####	et nom des clefs sous forme de chaines de caracteres		#####
####	de facon a eviter des erreurs					#####
####									#####
####									#####
#############################################################################
use constant servPort		=> $constantIndexer++;
use constant neededApp 		=> $constantIndexer++;
use constant statusFunc 	=> $constantIndexer++;
use constant slowAgentFunc 	=> $constantIndexer++;
use constant blockOnSlowAgent 	=> $constantIndexer++;
use constant supSock  		=> $constantIndexer++;
use constant connSock 		=> $constantIndexer++;
use constant sockList  		=> $constantIndexer++;
use constant threadList  	=> $constantIndexer++;
use constant appliList  	=> $constantIndexer++;
use constant sendRegList  	=> $constantIndexer++;
use constant sendRegListSrc  	=> $constantIndexer++;
use constant topicRegexps  	=> $constantIndexer++;
use constant recCbList  	=> $constantIndexer++;
use constant directCbList 	=> $constantIndexer++;
use constant cnnxion   		=> $constantIndexer++;
use constant connectedUuid	=> $constantIndexer++;
use constant bufRecByCnnx   	=> $constantIndexer++;
use constant bufEmiByCnnx   	=> $constantIndexer++;
use constant broadcastPort  	=> $constantIndexer++;
use constant broadcastBuses	=> $constantIndexer++;
use constant useMulticast	=> $constantIndexer++;
use constant appName		=> $constantIndexer++;
use constant messWhenReady 	=> $constantIndexer++;
use constant uuid	 	=> $constantIndexer++;
use constant pongQueue	 	=> $constantIndexer++;
use constant readyToSend 	=> $constantIndexer++;

#############################################################################
####                    METHODES      PUBLIQUES                     #####
#############################################################################
sub init
{
  if (defined $fileEventFunc) {
    print STDERR "Ivy warning: init has already been called\n";
    return;
  }


  srand(); # initialisation du generateur aléatoire qui sert à produire un UUID
  my $class = shift if (@_ and $_[0] eq __PACKAGE__);
  my (%options) = @_;

  # valeurs par defaut pour le parametre : variable d'environnement
  # ou valeur cablee, a defaut
  my $default_ivyBus = defined $ENV{"IVYBUS"} ?
			$ENV{"IVYBUS"} :
			BROADCAST_ADDRS.':'.BROADCAST_PORT;

  my %optionsAndDefaults = ( #PARAMETRES OBLIGATOIRES
			-loopMode         => undef,
			# TK ou LOCAL

			-appName	  => undef,
			# nom de l'appli

			# PARAMETRES FACULTATIFS (avec valeurs par defaut)

			# les adresses de reseau sur lesquelles ont broadcaste
			# suivies du No de port :
			# exemples :	"143.196.1.255,143.196.2.255:2010"
			#		"DGAC-CENATLS-PII:DGAC-CENATLS:2010"
			#		":2010" <= dans ce cas c'est la valeur
			# de reseau de broadcast par defaut qui est prise :
			# 127.255.255.255  c.a.d local a la machine
			-ivyBus		=> $default_ivyBus,

			-messWhenReady    => "_APP NAME READY",
			# message de synchro a envoyer quand pret

			-onDieFunc	=> [sub {}],
			# fonction de cb appelee lorsque l'appli a recu l'ordre
			# de quitter, on peut dans ce callback fermer
			# proprement les ressources avant de sortir.
			# ps : ne pas faire d'exit dans le callback,
			# c'est le bus qui s'en charge


			-filterRegexp => [],# nouvelle api cohérente avec ivy-c, c++, java
			-pruneRegexp => [], # obsolete
			# optimisation : si l'on connait les sujets des messages
		        # qu'on envoie, on fournit la liste des sujets
		        # et les regexps qui ne matchent pas
		        # ces sujets sont eliminees.
		       ) ;

  # on examine toutes les options possibles
  foreach my $opt  (keys %optionsAndDefaults) {
    # si un parametre a ete fourni, ignorer les valeurs par defaut
    next if defined $options{$opt} ;
    # sinon, prendre la valeur par defaut si elle existe
    if (defined $optionsAndDefaults{$opt}) {
      $options{$opt} = $optionsAndDefaults{$opt} ;
    # sinon, on jette l'eponge
    } else {
      croak "Error in Ivy::init: option $opt is mandatory\n";
    }
  }

  # on examine toutes les options fournies, pour detecter les inutiles
  foreach my $opt (keys %options) {
    unless (exists ($optionsAndDefaults{$opt})) {
      carp "Warning in Ivy::init:  option $opt is unknown";
    }
  }

  $ivyBus = $options{-ivyBus};
  $appName = $options{-appName} ;
  $onDieFunc = $options{-onDieFunc} ;


  # trace pour le debug
  my $verFile = 0;
#  while (!open ($trace, , ">>", "/tmp/Ivy_$appName:$verFile.log")) {$verFile++};
#  syswrite ($trace, "DEBUT\n");


  if (scalar (@{$options{-pruneRegexp}})) {
    carp "-pruneRegexp is *OBSOLETE*. -filterRegexp should be used instead\n";
    $options{-filterRegexp} = $options{-pruneRegexp} unless defined $options{-filterRegexp};
  }

  @topicRegexps = @{$options{-filterRegexp}};
  $messWhenReady =  $options{-messWhenReady} eq "_APP NAME READY" ?
			"$appName READY" :
			$options{-messWhenReady};


  if ($options{-loopMode} =~ /local/i) {
    # mode boucle d'evenement locale
    use IO::Select;
    $fileEventFunc = \&fileEvent ;
    $localLoopSelRead = IO::Select->new ();
    $localLoopSelWrite = IO::Select->new ();
    $loopMode = LOCAL_MAINLOOP;
  } elsif ($options{-loopMode} =~ /tk/i) {
    # mode boucle d'evenement de TK
    $fileEventFunc = \&_tkFileEvent ;
    $loopMode = TK_MAINLOOP;
  } else {
    croak "Error in Ivy::init, argument loopMode must be either TK or LOCAL\n";
  }

 $SIG{'PIPE'} = 'IGNORE' ;
} # end init

############# METHODE DE CLASSE NEW
sub new ($%)
{
  my ($class, %options) = @_;
  my $self = [];
  $#{$self} = $constantIndexer; # on predimensionne le tableau
  bless ($self, $class);

  # on verifie que la methode de classe init ait ete appelee
  unless ((defined $appName) && ($appName ne '')) {
    croak  "Error in Ivy::new,  you should have called Ivy->init () first.";
  }

  # No de port tcp du serveur
  $self->[servPort] = '';

  # liste des applis necessaires a l'appli locale
  $self->[neededApp] = [];

  # callback prenant en param 3 refs sur des listes :
  # [applis presentes, appli absentes, hash_applis_present]
  # cette fonction est appelee :
  # - tout les pollingTime tant que toutes les applis
  #   ne sont pas presentes
  # - des qu'une appli se connecte
  # - lorsqu'une appli se deconnecte
  $self->[statusFunc] = '';
  $self->[slowAgentFunc] = '';

  # callback prenant en param 1 refs sur une liste :
  #  [ref sur fonction, parametres]

  # socket de supervision en lecture/ecriture
  $self->[supSock] = '';

  # socket de connexion tcp
  $self->[connSock] = '';

  # tab ass : nom du fd => fd
  $self->[sockList] = {};

  # tab ass : nom de l'appli => fd
  $self->[appliList] = {};

  # tableau ass de liste  du type
  # sockId => [fonction, fonction, ...]
  # pour savoir quoi envoyer a qui
  # les fonctions anonymes sont compilees
  # dynamiquement a la reception des messages REGEXP
  # et filtrent les mess a envoyer et les envoient
  # au besoin
  $self->[sendRegList] = {};

  # tableau ass de liste  du type
  # sockId => ["regexp"...]
  # pour connaitre la valeur des regexp meme apres compilation
  $self->[sendRegListSrc] = {};

  # liste des topics qu'on envoie si on
  # filtre les regexps
  $self->[topicRegexps] = [];

  # liste de ref sur des couples
  # (regexp,callBack) les callbacks
  # sont appeles lors de
  # la reception de messages en fonction
  # du numero de regexp.
  $self->[recCbList] = [];

  # liste de callBack pour les messages directs
  $self->[directCbList] = [];

  # tableau ass : clef = nom:numero_de port
  # permet de verifier qu'on ne se connecte pas
  # sur nous meme et qu'on ne se reconnecte
  # pas sur un service en cas de bonjours repetes
  # valeur : nom de l'application
  $self->[cnnxion] = {};
  $self->[connectedUuid] = {};


  # tableau associatif, clef => file desc,
  # valeur :buffer au cas ou la lecture ne se termine
  # pas par \n, de maniere a resegmenter les messages
  $self->[bufRecByCnnx] = {};

  # tableau associatif, clef => file desc,
  # valeur :buffer au cas ou l'écriture bloque sur un fd
  # pour eviter de bloquer la mainloop sur un send on bufferise
  # les données dans le process
  $self->[bufEmiByCnnx] = {};

  # identifiant unique
  $self->[uuid] = sprintf ("%d%d", time(), rand()*(2**31));

  # queue de gestion des pings et des dumpTable:
  # clef : socket fd, valeur  :liste [timestamp, machine:port, callBack]
  $self->[pongQueue] = {};




  my %optionsAndDefaults = (
			-appName	  =>  $appName,
			# nom de l'appli

			# PARAMETRES FACULTATIFS (avec valeurs par defaut)
			-messWhenReady    => $messWhenReady,
			# message de synchro a envoyer quand pret


			# PARAMETRES FACULTATIFS (avec valeurs par defaut)

			# les adresses de reseau sur lesquelles ont broadcaste
			# suivies du No de port :
			# exemples :	"143.196.1.255,143.196.2.255:2010"
			#		"DGAC-CENATLS-PII:DGAC-CENATLS:2010"
			#		":2010" <= dans ce cas c'est la valeur
			# de reseau de broadcast par defaut qui est prise :
			# 127.255.255.255  c.a.d local a la machine
			-ivyBus		=> $ivyBus,

			-neededApp	  => [],
			# liste des appplis necessaires

			-statusFunc	 => sub {},
			# fonction de callBack qui sera appelee tant que
			# toutes les applis necessaires ne sont pas presentes,
			# et des que toutes les applis necessaires sont
			# presentes, et si une appli necessaire se deconnecte
			# les  parametres passes à la callback sont :
			# °[liste des applis presentes],
			# °[liste des applis absentes],
			# °[table de hash, clefs = applis presentes, valeurs = nombre d'applis .
			#    normalement ce nombre devrait etre 1, sinon
			#    ca veut dire que plus d'une appli de meme nom
			#    tourne sur le meme bus : danger !!
			# ° nom de l'appli qui genere l'evenement
			# ° evenement : 'subscribing'|'filtered'|'unsubscribing'|'died'|'new'
			# ° adresse
			# ° regexp si c'est un abonnement, un desabonnement ou un filtered

			-blockOnSlowAgent =>  1,
			# comportement lorque un ou plusieurs des agents connectés
			# ne consomment pas suffisement rapidement les messages. Ou bien on laisse
			# le send bloquer, ou bien on accumule les messages localement
			# en attendant que l'agent congestionné soit capable de les traiter.
			# cette méthode a l'avantage de ne pas bloquer la mainloop locale,
			# par contre la consomation mémoire peut devenir problématique.

			-slowAgentFunc =>  sub {},
			# fonction de callBack qui sera appelee si l'envoi de messages
			# à un agent n'est plus possible parce que l'agent en question ne
			# consomme pas ses messages assez vite, ou si un agent qui etait
			# dans cette etat retrouve sa capacité à lire les messages.
			# les paramètres passés à la callback sont
			# nom de l'appli, 
			# adresse
			# etat (congestion = 1, decongestion = 0)

			-filterRegexp => [@topicRegexps],
			-pruneRegexp => [], # obsolete
 		        # optimisation : si l'on connait les sujets des messages
		        # qu'on envoie, on fournit la liste des sujets
		        # et les regexps qui ne matchent pas
		        # ces sujets sont eliminees.
		       ) ;


  # on examine toutes les options possibles
  foreach my $opt  (keys %optionsAndDefaults) {
    # si un parametre a ete fourni, ignorer les valeurs par defaut
    next if defined $options{$opt} ;
    # sinon, prendre la valeur par defaut si elle existe
    if (defined $optionsAndDefaults{$opt}) {
      $options{$opt} = $optionsAndDefaults{$opt} ;
    # sinon, on jette l'eponge
    } else {
      croak "Error in Ivy::new: option $opt is mandatory\n";
    }
  }

  if (scalar (@{$options{-pruneRegexp}})) {
    carp "-pruneRegexp is *OBSOLETE*. -filterRegexp should be used instead\n";
    $options{-filterRegexp} = $options{-pruneRegexp} unless defined $options{-filterRegexp};
  }

  # on examine toutes les options fournies, pour detecter les inutiles
  foreach my $opt (keys %options) {
    unless (exists ($optionsAndDefaults{$opt})) {
      carp "Warning in Ivy::new, option $opt is unknown";
    }
  }

  $self->[appName] =    $options{-appName} ;
  $self->[messWhenReady] =    $options{-messWhenReady} ;
  @{$self->[neededApp]} =     @{$options{-neededApp}} ;
  $self->[statusFunc] =       $options{-statusFunc} ;
  $self->[slowAgentFunc] =    $options{-slowAgentFunc} ;
  $self->[blockOnSlowAgent] = $options{-blockOnSlowAgent} ;

  $self->[topicRegexps] = $options{-filterRegexp} ;
  $allBuses{$self}  = $self;

  ($self->[useMulticast], $self->[broadcastPort], $self->[broadcastBuses]) =
		_parseIvyBusParam ($options{-ivyBus});
  $self->[readyToSend] = {};


  return ($self);
} # end new

############### METHODE	IVY DESTROY
sub DESTROY ($)
{
  my $self = shift;
  return unless exists $allBuses{$self};
  # print ("DBG> DESTROY appele sur l'objet $self\n");

  # pour toutes les connections
  foreach my $fd (values %{$self->[sockList]}) {
    #      send ($fd, sprintf (MSG_FMT, BYE, 0, ""), 0)
    #	or $self->_removeFileDescriptor ($fd);
    # the 2 previous lines seems to works with other ivy-perl applis
    # but DO NOT work with ivy-c api.
    # the 2 next lines works. This has to been validated! CM 21/12/2000
    if (defined $fd) {
      _univSend ($self, $fd, sprintf (MSG_FMT, BYE, 0, ""));
      $self->_removeFileDescriptor ($fd, 'DESTROY');
    }
  }

  #  on clot la socket de signalisation (UDP)
  # print "DBG> fermeture de supSock ", $self->[supSock] ,"\n";
  # the following test has been expanded to avoid some nasty bug
  # which appeared when upgrading from perl-tk 800.023 to 800.024
  $self->[supSock]->close() if ($self->[supSock] and $self->[supSock]->connected());
  delete $allBuses{$self};

  # on clot la socket de connection
  # print "DBG> fermeture de connSock ", $self->[connSock], "\n";
  # the following test has been expanded to avoid some nasty bug
  # which appeared when upgrading from perl-tk 800.023 to 800.024
  $self->[connSock]->close() if ($self->[connSock] and $self->[connSock]->connected());
  undef (@$self);
#  close ($trace);
} # end DESTROY

############### METHODE DE CLASSE STOP
sub stop ()
{
  foreach my $bus (values %allBuses) {
    $bus->DESTROY();
  }  # pour toutes les connections
} # end stop


############## METHODE DE CLASSE EXIT
sub exit ()
{
  Ivy::stop ();
  if (defined $localLoopSelRead) {
    undef  $localLoopSelRead ;
    undef  $localLoopSelWrite ;
  } else {
    Tk::exit ();
  }
} # end exit

############### PROCEDURE	BUS START
sub start
{
    my $self;

    # compatibility for version 3 interface, ie. no objects manipulated by programmer
    if (not @_ or ref ($_[0]) ne __PACKAGE__) {
	init (@_);
	$self = $globalIvy = new Ivy;
    } else {
	$globalIvy = $self = shift;
    }

    if ($self->[connSock]) {
	print "*** the Ivy bus is already started\n";
	return;
    }
    # cree la socket de connexion, recupere le no de port
    my $connSock = $self->[connSock] = IO::Socket::INET->new(Listen => 128,
							     Proto    => 'tcp',
							     ReuseAddr   => 1) ;
    # on memorise tout ca, ce qui evitera par la suite de se
    # repondre a soi-meme. On le fait sous nos deux noms :
    # le nom de machine et 'localhost'
    my ($n, $al, $t, $l, @hostAddrs) = gethostbyname (hostname());
    foreach my $a (@hostAddrs) {
#     syswrite ($trace, ("DBG> I am " . unpack ('CCCC', $a) . $connSock->sockport . "\n"));
      $self->[cnnxion]->{"$a:". $connSock->sockport} = "\004";
    }

    my $localhostAddr = (gethostbyname ('localhost'))[4] ;
    $self->[cnnxion]->{"$localhostAddr:". $connSock->sockport} = "\004";

    # le message de bonjour à envoyer: "no de version no de port"
    my $bonjourMsg = sprintf ("%d %d %s %s\n", IVY_PROTOCOLE_VERSION, $connSock->sockport(),
			      $self->[uuid], $self->[appName]);

    if (!$self->[useMulticast]) {
	# cree la socket de broadcast
	$self->[supSock] = IO::Socket::INET->new
	    (LocalPort  => $self->[broadcastPort],
	     Proto      => 'udp',
	     Type       => SOCK_DGRAM,
	     ReuseAddr      => 1);

	$self->[supSock]->sockopt (SO_BROADCAST, 1);
	foreach my $netBroadcastAddr (@{$self->[broadcastBuses]}) {
#	    print "BroadcastBus: --", $netBroadcastAddr, "--\n";
	    send ($self->[supSock], $bonjourMsg, 0, $netBroadcastAddr) or
		carp "Warning in Ivy::start, broadcast of Hello message failed: $!";
	}
    }
    else {
	# creating the multicast socket
	$self->[supSock] = IO::Socket::Multicast->new
	    (LocalPort  => $self->[broadcastPort],
	     ReuseAddr  => 1);

	# Multicast datagrams with initial TTL 0 are restricted to the same host.
	# Multicast datagrams with initial TTL 1 are restricted to the same subnet.
	# Multicast datagrams with initial TTL 32 are restricted to the same site.
	# Multicast datagrams with initial TTL 64 are restricted to the same region.
	# Multicast datagrams with initial TTL 128 are restricted to the same continent.
	# Multicast datagrams with initial TTL 255 are unrestricted in scope.
	$self->[supSock]->mcast_ttl(64);
	# $self->[supSock]->mcast_loopback(1); must be 1, which is the default

 	foreach my $netMulticastAddr (@{$self->[broadcastBuses]}) {
	    my ($port,$multicastGroupI) = sockaddr_in ($netMulticastAddr);
	    my $multicastGroup = inet_ntoa($multicastGroupI);
	    # print "DBG> MulticastBus: --", $multicastGroup,":$port", "--\n";
	    $self->[supSock]->mcast_add($multicastGroup);
	    $self->[supSock]->mcast_send($bonjourMsg, $multicastGroup.":".$port) or
		carp "Warning in Ivy::start, multicast of Hello message failed: $!";
	}
    }
    # callback pour traiter la reception des bonjours
    &$fileEventFunc ($self->[supSock], [\&_getBonjour, $self]) ;

    # callback pour traiter les demandes de cxion
    &$fileEventFunc ($self->[connSock], [\&_getConnections, $self]) ;

   return $self;
} # end start


############### PROCEDURE	BIND REGEXP
sub bindRegexp ($$$;$$)
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my ($regexp, $cb, $callByRef, $bindOnce) = @_;
  my $id;

  $callByRef = 0 unless defined $callByRef ;
  $bindOnce = 0 unless defined $bindOnce ;

  my $extraParam;
  if ($bindOnce) {
    $extraParam = BIND_ONCE;
  } else {
     $extraParam = $callByRef ? CALL_BY_REF : CALL_BY_VALUE;
  }

#  print ("DBG> bindRegexp:: self=$self, regexp=$regexp, extraParam=$extraParam\n");
#  my $original_regexp = $regexp;
#   # on substitue les meta caracteres des regexps perl : \d, \w, \s, \e
#   # par les classes de caracteres corespondantes de maniere a ce
#   # qu'une appli distante non perl comprenne ces regexp.
#   $regexp =~ s|
#     (
#      (?<!\\) \[	# le premier crochet ouvrant non precede d'un \
#      .*? 		# ce qu'il y a dans le crochet, en mode frugal
#      (?<!\\) \]	# le premier crochet fermant non precede d'un \
#     )
#       |
# 	_substituteEscapedChar ('inside', $1)
# 	  |xge;

#   $regexp = _substituteEscapedChar ('outside', $regexp);

  # substitution des intervalles numériques de la forme
  # (?I-10#20) ou (?I-10#20f) ou (?I-10#20i)

  $regexp =~ s|
     \(\?I	# l'extension (?I
     ([\d-]+)   # la borne inférieure
     \#         # l'operateur d'intervalle
     ([\d-]+)   # la borne supérieure
     ([if]?)    # le caractère de codage f pour flottant, i pour integer, flottant par defaut
     \)         # la parenthèse fermante
      |
	_regexpGen ($1, $2, $3);
	  |xge;

  # print ("DBG> regexp = $regexp\n");
  if ($^W) {
    eval {my $test = "a" =~ /$regexp/ } ;  # testing the regexp for avoiding
    if ($@) {
      carp "Warning in Ivy::bindRegexp, ill-formed regexp: '$regexp'" ; 
      return;
    };
  }


  if ($cb) {
    # on rajoute le couple $regexp, $cb dans la liste des messages
    # qu'on prend

    # on teste la validité de l'argument
    if (ref ($cb) ne 'ARRAY') {
      carp ("Warning binRegexp on $regexp :\nargument 3 (callback) is not correct and will be ignored\n");
      return ();
    }
    if ((ref ($cb->[0]) ne 'CODE') && (ref ($cb->[1]) ne 'CODE')) {
      carp ("Warning binRegexp on $regexp :\nargument 3 (callback) is not correct and will be ignored\n");
      return ();
    }

    # on commence par tester si on a un id libere dans le tableau
    for ($id=0; $id <= ($#{$self->[recCbList]}+1); $id++)  {
      last unless  (defined $self->[recCbList][$id]) && @{$self->[recCbList][$id]->[1]};
    }
    $self->[recCbList][$id] =  [$regexp, $cb, $extraParam];

    # on envoie les messages regexps aux processus deja connectes
    _sendLastRegexpToAllreadyConnected ($self, $id) ;
  }
  else {
    # on vire le callback, et on se desabonne de cette regexp
    for (my $id=0; $id <= $#{$self->[recCbList]}; $id++)  {

      next unless (defined $self->[recCbList][$id]) &&
	@{$self->[recCbList][$id]->[1]};

      if ($self->[recCbList][$id]->[0] eq $regexp) {

	$self->[recCbList][$id]->[1] = [];
	# on envoie le mesage delregexp
	foreach my $fd (values %{$self->[sockList]}) {
	  _univSend ($self, $fd, sprintf (MSG_FMT, DELREGEXP, $id, ""));
	}
      }
    }
  }
  return ($id);
} # end bindRegexp

############### PROCEDURE	BIND REGEXP ONCE
sub bindRegexpOneShot ($$$)
{
  Ivy::bindRegexp ($_[0], $_[1], $_[2], 0, 1);
}

############### PROCEDURE	CHANGE REGEXP
sub changeRegexp ($$$)
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my ($regexpId, $regexp) = @_;

  $regexp =~ s|
     \(\?I	# l'extension (?I
     ([\d-]+)   # la borne inférieure
     \#         # l'operateur d'intervalle
     ([\d-]+)   # la borne supérieure
     ([if]?)    # le caractère de codage f pour flottant, i pour integer, flottant par defaut
     \)         # la parenthèse fermante
      |
	_regexpGen ($1, $2, $3);
	  |xge;

  # print ("DBG> regexp = $regexp\n");
  if ($^W) {
    eval {my $test = "a" =~ /$regexp/ } ;  # testing the regexp for avoiding
    if ($@) {
      carp "Warning in Ivy::changeRegexp, ill-formed regexp: '$regexp'" ; 
      return;
    };
  }

  unless (exists $self->[recCbList][$regexpId]) {
    warn ("Warning in Ivy::changeRegexp, invalid regexpId\n");
    return (-1);
  } else {
    $self->[recCbList][$regexpId]->[0] =  $regexp;
    _sendLastRegexpToAllreadyConnected ($self, $regexpId) ;
    return ($regexpId);
  }
} # end changeRegexp

############### METHODE	BIND REGEXP
sub bindDirect
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my ($id, $cb) = @_;

  if ($cb) {
    # on rajoute la $cb dans la liste des messages
    # qu'on prend
    $self->[directCbList][$id] =  $cb;
  } else {
    # on vire le callback
    undef $self->[directCbList][$id];
  }
} # end bindDirect


############### PROCEDURE	SEND MSGS
sub sendMsgs
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my $appSock;
  my $total = 0;

  # pour tous les messages
  foreach my $msg (@_) {
  study ($msg);
  carp "Warning in Ivy::sendMsgs, a message contains a '\\n'. " .
    "You should correct it:\n'$msg'" if ($^W && ($msg =~ /\n/)) ;

  # pour routes les connections
  foreach  $appSock (values %{$self->[sockList]}) {
    # pour toutes les fonctions de filtrage de regexp
     $total += _sendMsgTo ($self, $appSock, $msg);
    }
  }
  #    print "DBG> sended $total times\n";
  return $total;
} # end sendMsgs







############### PROCEDURE	SEND MSGS
sub sendAppNameMsgs
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  return $self->sendMsgs (map ("$self->[appName] $_", @_));
}


# sub sendAppNameMsgs
# {
#   my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
#   my @msgs = @_;
#   my $total = 0;

#   # pour tous les messages
#   foreach (@msgs) {
#     carp "Warning in Ivy::sendAppNameMsgs, a message contains a '\\n'. Skipping it:\n'$_'" if ($_ =~ /\n/);

#     my $msg = "$self->[appName] $_";
#     study ($msg);

#     # pour toutes les connections
#     foreach my $fd (keys %{$self->[sockList]}) {

#       # pour toutes les fonctions de filtrage de regexp
#       foreach my $regexpFunc (@{$self->[sendRegList]{$fd}}) {
# 	$total += &{$regexpFunc} (\$msg) if defined $regexpFunc;
#       }
#     }
#   }
#   #    print "DBG> sended $total times\n";
#   return $total;
# } # end sendAppNameMsgs



############### PROCEDURE	SEND DIRECT MSGS
sub sendDirectMsgs
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my ($to, $id, @msgs) = @_;

  if (defined $to and defined ($self->[appliList]{$to})) {
    my @fds = @{$self->[appliList]{$to}};
    # pour tous les messages
    foreach my $msg (@msgs) {
      carp "Warning in Ivy::sendDirectMsgs, a message contains a '\\n'. Skipping it:\n'$msg'" if ($msg =~ /\n/);

      foreach my $fd (@fds) {
	_univSend ($self, $fd, sprintf (MSG_FMT, DIRECT_MSG, $id, "$msg"));;
      }
    }
    return 1;
  }
  else {
    my $to_appli = (defined $to) ? $to : '';
    carp "Warning in Ivy::sendDirectMsgs, application $to_appli unknown";
    return 0;
  }
} # end sendDirectMsgs


############### METHOD	SEND DIE TO
sub sendDieTo
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my ($to) = @_;

  if (defined $to and defined $self->[appliList]{$to}) {
    my @fds = @{$self->[appliList]{$to}};

    carp "Attention : in Ivy::sendDieTo big BUG \@fds is empty"
      if (scalar (@fds) == 0);

    # pour tous les messages
    foreach my $fd (@fds) {
      $self->_sendDieTo($fd);
    }
    return 1;
  }
  else {
    my $to_appli = (defined $to) ? $to : '';
    carp "Warning in Ivy::sendDieTo, application '$to_appli' is unknown" if $^W;
    return 0;
  }
} # end sendDieTo


############### METHOD	PING
sub ping ($$\&)
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my ($to, $pongCbRef) = @_;
  my @fds;

  return unless defined $to;

  if (defined ($self->[appliList]{$to})) {
    @fds = @{$self->[appliList]{$to}};
  } else {
    my %handleByName = reverse %nameByHandle;
#    printf "DBG>>> all names : %s\n", join (', ', keys %handleByName);
    if (exists $handleByName{$to}) {
      @fds = ($handleByName{$to});
    } else {
      carp "Warning in Ivy::ping, application '$to' is unknown" if $^W;
      return 0;
    }
  }

  # pour tous les messages
  foreach my $fd (@fds) {
#    print ("DBG>> ping : send to fd $fd\n");
    $self->[pongQueue]->{$fd} = [$pingId, Time::HiRes::time(), $pongCbRef];
    _univSend ($self, $self->[sockList]->{$fd}, sprintf (MSG_FMT, PING, $pingId, ""));
  }
  return ($pingId++);
} # end ping

############### METHOD	DUMP_TABLE
sub dumpTable ($$\&)
{
  my $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my ($to, $dumpTableCbRef) = @_;
  my @fds;

  return unless defined $to;

  if (defined ($self->[appliList]{$to})) {
    @fds = @{$self->[appliList]{$to}};
  } else {
    my %handleByName = reverse %nameByHandle;
#    printf "DBG>>> all names : %s\n", join (', ', keys %handleByName);
    if (exists $handleByName{$to}) {
      @fds = ($handleByName{$to});
    } else {
      carp "Warning in Ivy::ping, application '$to' is unknown" if $^W;
      return 0;
    }
  }

  # pour tous les messages
  foreach my $fd (@fds) {
#    print ("DBG>> dumpTable : send to fd $fd\n");
    $self->[pongQueue]->{$fd} = [$pingId, $dumpTableCbRef];
    _univSend ($self, $self->[sockList]->{$fd}, sprintf (MSG_FMT, DUMP_TABLES, $pingId, ""));
  }
  return ($pingId++);
} # end ping
###############   METHODE     MAINLOOP
sub mainLoop (;$)
{
  my  $self = ref($_[0]) eq __PACKAGE__  ? shift : $globalIvy;
  my ($fd, @selRes, @allDesc);


  if ($loopMode == TK_MAINLOOP) {
    eval {Tk::MainLoop ()};
    return;
  }

  croak "Error in Ivy::mainLoop, Ivy not properly initialized\n"
    unless defined  $localLoopSelRead;

  while (defined  $localLoopSelRead) {
# READ
    @selRes = IO::Select::select ($localLoopSelRead, $localLoopSelWrite, undef ,
 				  $selectTimout) ;
    _scanAfter () ;

    foreach $fd (@{$selRes[0]}) {
      if (ref $localBindByHandle{$fd} eq 'CODE') {
	&{$localBindByHandle{$fd}} ;
      }
      else {
	my ($cb, @arg) = @{$localBindByHandle{$fd}} ;
	&$cb (@arg)
      }
    }

#WRITE 
    my $bufEmiRef;
    my $sent;

    foreach $fd (@{$selRes[1]}) {
      $bufEmiRef = \($self->[bufEmiByCnnx]->{$fd});
      $sent = send ($fd, $$bufEmiRef, 0);
      unless (defined $sent) {
	# y a rien à faire
      } elsif ($sent ==   length ($$bufEmiRef)) {
	$$bufEmiRef = "";
	_callCongestionCb ($self, $fd, 0);
      } elsif ($sent >= 0) {
	substr ($$bufEmiRef, 0, $sent, '');
      } else {
	$self->_removeFileDescriptor ($fd, 'mainLoop[WRITE]') unless ($!{EAGAIN} || $!{EWOULDBLOCK}|| $!{EINTR} || $!{EMSGSIZE} || $!{ENOBUFS})
      }
    }
  }
} # end mainLoop


############### METHODE	AFTER
sub after ($$;$)
{
  # test du premier argument au cas où la fonction est
  # appelee de maniere objet : premier argument = class ou une instance
  # de classe
  shift if ((ref ($_[0]) eq __PACKAGE__) || ($_[0] eq __PACKAGE__)) ;

  my ($timeAfter,  $cbListRef) = @_;
  $timeAfter /= 1000;
  $selectTimout = $timeAfter if $timeAfter < $selectTimout;

  # si la valeur de timout est negative : c'est un after sinon
  # c'est un repeat
  $afterList{++$afterId} = [AFTER, $timeAfter,
			    gettimeofday()+$timeAfter, $cbListRef];

  return ($afterId);
} # end after

############### METHODE	REPEAT
sub repeat ($$;$)
{
  # test du premier argument au cas où la fonction est
  # appelee de maniere objet : premier argument = class ou une instance
  # de classe
  shift if ((ref ($_[0]) eq __PACKAGE__) || ($_[0] eq __PACKAGE__)) ;

  # on passe le temps en secondes pour le select
  my ($timeAfter,  $cbListRef) = @_;
  $timeAfter /= 1000;
  $selectTimout = $timeAfter if $timeAfter < $selectTimout;

  $afterList{++$afterId}= [REPEAT, $timeAfter, gettimeofday()+$timeAfter,
			   $cbListRef];
  return ($afterId);
} # end repeat

############### METHODE	AFTER CANCEL
sub afterCancel ($;$)
{
  # test du premier argument au cas où la fonction est
  # appelee de maniere objet : premier argument = class ou une instance
  # de classe
  shift if ((ref ($_[0]) eq __PACKAGE__) || ($_[0] eq __PACKAGE__)) ;

  my $id = shift;

  if (defined ($id) && defined $afterList{$id}) {
    if ($afterList{$id}->[1] <= $selectTimout) {
      delete $afterList{$id} ;
      # le timout de l'after/repeat etait le plus petit des timout
      # on cherche donc le plus petit parmi ceux qui restent;
      $selectTimout = MAX_TIMOUT;
      foreach my $af (values %afterList) {
	$selectTimout = $af->[1] if $af->[1] < $selectTimout ;
      }
    }
    else {
      delete $afterList{$id} ;
    }
  }
} # end afterCancel

############### METHODE AFTER RESET TIMER
# permet de gérer des timout plus facilement en permettant de
# réarmer un after non encore déclenché à sa valeur initiale
# cela évite le aftercancel suivi d'un nouvel after
sub afterResetTimer ($;$)
{
 # test du premier argument au cas où la fonction est
  # appelee de maniere objet : premier argument = class ou une instance
  # de classe
  shift if ((ref ($_[0]) eq __PACKAGE__) || ($_[0] eq __PACKAGE__)) ;

  my $id = shift;
  if (defined ($id) && defined $afterList{$id}) {
    $afterList{$id}->[2] = $afterList{$id}->[1] + gettimeofday();
  }
} # end afterResetTimer


############### METHODE	FILE EVENT 
sub fileEvent ($$;$)
{
  # test du premier argument au cas où la fonction est
  # appelee de maniere objet : premier argument = class ou une instance
  # de classe
  shift if ((ref ($_[0]) eq __PACKAGE__) || ($_[0] eq __PACKAGE__)) ;

  my ($fd, $cb) = @_;


  unless (defined $localLoopSelRead) {
    croak ("Error in Ivy::fileEvent, Ivy should have been initialised in LOCAL loop mode\n");
  }

  if ($cb) {
    # adding the handler
    $localBindByHandle{$fd} = $cb;
    $localLoopSelRead->add ($fd);
  } else  {
    # deleting the handler
    delete $localBindByHandle{$fd};
#    print ("DBG> Ivy::fileEvent : removing fd from the select\n");
    $localLoopSelRead->remove ($fd);
    $localLoopSelWrite->remove ($fd);
  }
} # end fileEvent




sub getUuid ($)
{
   my $self = shift;
   return $self->[uuid];
}
#############################################################################
####                     METHODES       PRIVEES                       #####
#############################################################################


############### METHODE	GET BONJOUR
sub _getBonjour ($)
{
  my $self = shift;
#  my $DTS = sprintf ("%2d:%2d:%2d", (localtime())[2,1,0]);

  my $bonjourMsg = '';

  # l'hote distant
  my $inetAddr = $self->[supSock]->recv ($bonjourMsg, 1024, 0);

  unless (length $inetAddr) {
    carp "Warning in Ivy::_getBonjour, recv error, Hello message discarded";
    return;
  }

  my $addr = (unpack_sockaddr_in ($inetAddr))[1];

  my $peerName = _getHostByAddr ($addr);

  # on force $peerPort a etre vu comme une valeur numerique
  my ($version, $peerPort, $uuid, $udpAppName) = 
#    $bonjourMsg =~ /^(\d+)\s+(\d+)\s+(?:(\w+)\s+(.*))?/;
    $bonjourMsg =~ /^(\d+)\s+(\d+)(?:\s+(\S+)\s+(.*))?\n/;

  $udpAppName = 1 unless defined $udpAppName;
# syswrite ($trace,  "DBG<$DTS>[$appName]> bonjourMsg = '$bonjourMsg'\n");
# syswrite ($trace,  "DBG<$DTS>[$appName]> reception de $peerName : bonjour $peerPort uuid = $uuid\n");

  unless (defined ($version) && defined ($peerPort)) {
    carp "Warning[$appName] in Ivy::_getBonjour, ill-formed Hello message \"$bonjourMsg\"" ;
    return;
  }

  if ($version != IVY_PROTOCOLE_VERSION) {
    carp "Warning[$appName] in Ivy::_getBonjour,  connection request from ".
	"$peerName with protocol version $version,\ncurrent version is " .
	IVY_PROTOCOLE_VERSION ;
    return;
  }


  # on verifie qu'on ne se repond pas et qu'on ne
  # se reconnecte pas a un process deja connecte
  if (exists ($self->[cnnxion]{"$addr:$peerPort"})) {
#   syswrite ($trace, "DBG<$DTS>[$appName]> from $self->[appName] DISCARD bonjour de $peerName:$peerPort [$udpAppName]: DEJA CONNECTE\n") ;
    return ;
  } elsif ((defined $uuid) && ($uuid eq $self->[uuid])) {
#   syswrite ($trace, "DBG<$DTS>[$appName]> from $self->[appName] DISCARD bonjour de $peerName:$peerPort [$udpAppName]: $uuid  c'est MOI\n") ;
    return;
  } elsif ((defined $uuid) && (exists ($self->[connectedUuid]->{$uuid}))) {
#   syswrite ($trace, "DBG<$DTS>[$appName]> from $self->[appName] DISCARD bonjour de $peerName:$peerPort:$uuid [$udpAppName] DEJA CONNECTE\n") ;
    return;
  } else {
#     syswrite ($trace, "DBG<$DTS>[$appName]> reception de $peerName : bonjour $udpAppName:$peerPort") ;
#     syswrite ($trace, " uuid=$uuid") if (defined $uuid);
#     syswrite ($trace, "\n");
#     syswrite ($trace, "DBG<$DTS>>[$appName] from $self->[appName]  ACCEPT bonjour de $peerName:$peerPort:$uuid [$udpAppName]\n") ;
    $self->[connectedUuid]->{$uuid} = 1 if (defined $uuid);
  }

  # on verifie que l'adresse fasse partie de l'ensemble de reseau
  # definis par ivybus
  my $addrInIvyBus = 0;
  my @ivyBusAddrList = map ( (unpack_sockaddr_in ($_))[1],
				@{$self->[broadcastBuses]});
  # Bon dans cette version on reponds aux bonjour emis par
  # la machine locale, on ne peut donc pas avoir
  # une appli qui ne causerait qu'a des machines sur une
  # autre reseau, si ca embete qqun, qu'il me le dise
  push (@ivyBusAddrList, pack ("CCCC", 127,255,255,255));
  push (@ivyBusAddrList, (gethostbyname (hostname()))[4]);

  use bytes;
  foreach my $ivyBusAddr (@ivyBusAddrList) {
    $addrInIvyBus = 1 unless (grep ($_ != 0, unpack ("C4",
			     ($addr & $ivyBusAddr) ^ $addr)));
  }
  no bytes;

  if ($addrInIvyBus == 0) {
    carp "Warning[$appName]: Hello message from $peerName ignored,\n".
	"this guy is outside our emission zone\n" if $^W;
    return;
  }

  # ouverture du canal de communication
  my $appSock = IO::Socket::INET->new (PeerAddr => inet_ntoa ($addr),#$peerName,
				       PeerPort => $peerPort,
				       Proto    => 'tcp');

  if ($appSock) {
    my $flags = fcntl($appSock, F_GETFL, 0);
    unless (fcntl($appSock, F_SETFL, $flags | O_NONBLOCK)) {
      carp "Warning[$appName] Can't set flags for the socket: $!\n";
      return;
    }
    $appSock->sockopt(Socket::TCP_NODELAY, TCP_NO_DELAY_ACTIVATED);


    binmode ($appSock);
    # on cree une entree pour $appSock dans la liste des regexp
    $nameByHandle{$appSock}=_getHostByAddr($addr) .":$peerPort";
    $self->[cnnxion]{"$addr:$peerPort"} = $udpAppName;
    $self->[sendRegList]{$appSock} = [];
    $self->[sendRegListSrc]{$appSock} = [];
    $self->[bufRecByCnnx]{$appSock} = '';
    $self->[bufEmiByCnnx]{$appSock} = '';
    $self->[sockList]{$appSock} = $appSock;


#     syswrite ($trace, sprintf ("_getBonjour : ajout dans le fdset de %s[%s]:%d\n",
# 			       (gethostbyaddr ($appSock->peeraddr(),AF_INET))[0],
# 			       join (':', unpack ('C4', $appSock->peeraddr())),
# 			       $appSock->peerport()));

    &$fileEventFunc ($appSock, [\&_getMessages, $self, $appSock]) ;

    # on balance les regexps qui nous interessent a l'appli distante
    $self->_sendWantedRegexp ($appSock);
  }  else {
    carp "Warning[$appName] in Ivy::_getBonjour, connection to " .
      "$peerName:$peerPort is impossible" ;
  }
} # end _getBonjour


############### PROCEDURE	GET CONNECTIONS
sub _getConnections ($)
{
  my $self = shift;

  my $appSock = $self->[connSock]->accept();

  unless (defined $appSock) {
    carp "Warning in Ivy::_getConnections, \$appSock not defined";
    return;
  } else {
    my $flags = fcntl($appSock, F_GETFL, 0);
    unless (fcntl($appSock, F_SETFL, $flags | O_NONBLOCK)) {
      carp "Can't set flags for the socket: $!\n";
      return;
    }

    $appSock->sockopt(Socket::TCP_NODELAY, TCP_NO_DELAY_ACTIVATED);
    binmode ($appSock);
  }


#   syswrite ($trace, sprintf ("_getConnections : ajout dans le fdset de %s[%s]:%d\n",
# 			     (gethostbyaddr ($appSock->peeraddr(),AF_INET))[0],
# 			     join (':', unpack ('C4', $appSock->peeraddr())),
# 			     $appSock->peerport()));


  # callback pour traiter la reception des messages
  &$fileEventFunc ($appSock, [\&_getMessages,  $self, $appSock]) ;

  # on cree une entree pour $appSock dans la liste des regexp
  $self->[sendRegList]{$appSock} = [];
  $self->[sendRegListSrc]{$appSock} = [];
  $self->[bufRecByCnnx]{$appSock} = '';
  $self->[bufEmiByCnnx]{$appSock} = '';
  $self->[sockList]{$appSock} = $appSock;
  # on balance les regexps qui nous interessent a l'appli distante
  $self->_sendWantedRegexp ($appSock);
} # end _getConnections


############### METHODE	GET MESSAGES
sub _getMessages ($$)
{
    my ($self, $appSock) = @_;

    unless (defined $appSock) {
      carp "Warning in Ivy::_getMessages : *UN*inititialized appSock, don't do anything\n" if $^W;
      return;
    }

    my $bufferRef = \$self->[bufRecByCnnx]{$appSock};
    my ($addr, $peerPort, $senderName);
    my $nlIndex;
    my $mess;


#     syswrite ($trace,  sprintf ("_getMessages from %s[%s]:%d\n",
# 			     (gethostbyaddr ($appSock->peeraddr(),AF_INET))[0],
# 			     join (':', unpack ('C4', $appSock->peeraddr())),
# 			     $appSock->peerport()));

    # on recupere le message
    unless (sysread ($appSock, $$bufferRef, 65536, length ($$bufferRef))) {
      # message null : broken pipe, ça s'est deconnecte a l'autre bout
      # on vire ce fd de la boucle d'evenements
      # print ("DBG> _getMessages, recv err, calling removeFileDesc.\n");
      # Bon la il faudra un jour clarifier ce bordel, lister toutes
      # les facons dont un couple d'applis connectées peuvent sortir et
      # eviter les dead lock qui doivent subsister.
#      syswrite ($trace,   sprintf ("_getMessage : bad FD[%d] detected errno=%d\n",  $appSock->peerport(), $!));
      $self->_removeFileDescriptor ($appSock, '_getMessages') unless ($!{EAGAIN});
      return;
    }

    $addr = $appSock->peeraddr();
    $peerPort = $appSock->peerport() ;
    $senderName = $self->[cnnxion]{"$addr:$peerPort"} ;
    $senderName = "NONAME" unless $senderName;
    $senderName =~ s/^\004//g;

    while (($nlIndex= index ($$bufferRef, "\n")) > 0) {
      $mess = substr ($$bufferRef, 0, $nlIndex, '');
      substr ($$bufferRef, 0, 1, '');
      # on recupere les 3 champs : le type, le numero de regexp, les valeurs
      my ($type, $id, $valeurs) = $mess =~ /^(\d+)
	\s+
	  (\d+)
	    \002
	      (.*)/x ;

	# si ca a chie on rale
      (carp "Warning in Ivy::_getMessages, ill-formated message \'$mess\'" and return) unless defined $type ;

#      syswrite ($trace, "_getMessage type = $type\n");

      # sinon on fait en fonction du type de message
      if ($type == MSG) {				       # M S G
	# on recupere le couple call back, regexp correspondant
	# a l'identifiant et on appelle la fonction avec les parametres
	# traites par la regexp

#	if ((ref ($self->[recCbList][$id]) eq 'ARRAY') &&
#	    (my @cb = @{$self->[recCbList][$id]->[1]}))  {


	if (my @cb = @{$self->[recCbList][$id]->[1]}) {
	  my $cb = shift @cb;
	
	  # cleaning $sendername with previous \004 used for connection status
	  # bindRegexp avancé : on envoie une liste nom adresse port au lieu du nom
	  $senderName = [$senderName, _getHostByAddr ($addr), $peerPort]
	    if ($self->[recCbList][$id]->[2] == CALL_BY_REF);

	  if (ref($cb) ne 'CODE') {
	    my $method = shift @cb;
	    # on split sur ETX
	    $cb->$method($senderName, @cb, split ("\003", $valeurs)) ;
	  }  else {
	    &$cb ($senderName, @cb, split ("\003", $valeurs)) ;
	  }

	  if ($self->[recCbList][$id]->[2] == BIND_ONCE) {
	    # on vire la regexp des regexps verifiées
	    # print ("DBG> receive BIND ONCE message\n");
	    $self->[recCbList][$id]->[1] = [];
	    # on envoie le mesage delregexp
	    foreach my $fd (values %{$self->[sockList]}) {
	      _univSend ($self, $fd, sprintf (MSG_FMT, DELREGEXP, $id, ""));
	    }
	  }
	} else {
	  #_sendErrorTo ($appSock, "REEGXP ID $id inconnue");
	  carp ("Warning in Ivy::_getMessages, received an unknown message or double one shot message ".
		"with id $id from $senderName :\n\"$mess\"") if $^W;
	}
      }

      elsif ($type == BYE) {
#	syswrite ($trace,  "reception d'un bye\n");
	$self->_removeFileDescriptor ($appSock, '_getMessages[BYE]');		        # B Y E
      }

      elsif ($type == REGEXP) {			        # R E G E X P
	# on ajoute une fonction traitant la regexp et envoyant le
	# message sur le bon fd dans la liste des fonctions de filtrage
	# ca permet de compiler les regexp avec 'once' donc une
	# fois pour toute, et ainsi optimiser la vitesse de
	# filtrage des messages a envoyer
#	  print "DBG> REGEXP from $senderName '$id' '$valeurs'\n";
	my $host = _getHostByAddr ($addr);
	if ($self->_toBePruned ($senderName, $valeurs)) {
	  &_scanConnStatus ($self, $senderName, 'filtered', "$host:$peerPort" , $valeurs);
	  next;
	} else {
	  &_scanConnStatus ($self, $senderName, 'subscribing', "$host:$peerPort" , $valeurs);
	}
	# on affecte la nouvelle regexp a un id
	$self->[sendRegListSrc]{$appSock}->[$id] = $valeurs;
#	printf ("DBG> add id $id regexps=[$valeurs]\n");
	$self->[sendRegList]{$appSock}->[$id] =
	  eval ('sub {@{$_[1]} = ${$_[0]} =~ /($valeurs)/io;}');


#        $self->[sendRegList]{$appSock}->[$id] = eval <<'_EOL_';
# 	  sub  {
# 	    use strict;
# 	    if (my @args = ${$_[0]} =~ /($valeurs)/io) {
# 	      shift @args;
# 	      $args[$#args] .= "\003" if @args;
#               $self->[bufEmiByCnnx]->{$appSock} .=
#                      sprintf (MSG_FMT, MSG, $id, join ("\003",@args));
#               my $sent = send ($appSock, $self->[bufEmiByCnnx]->{$appSock}, 0);
#               unless (defined $sent) {
#                 # y a rien à faire
#               } elsif ($sent ==   length ($self->[bufEmiByCnnx]->{$appSock})) {
# 		$self->[bufEmiByCnnx]->{$appSock} = "";
# 	      } elsif ($sent >= 0) {
# 		substr ($self->[bufEmiByCnnx]->{$appSock}, 0, $sent, '');
# 	      } else {
# 		$self->_removeFileDescriptor ($appSock) ;
# 	      }
# 	    }
# 	    return 1;
# 	  }
#        _EOL_
      }

      elsif ($type == ERROR) {				# E R R O R
	carp ("Warning in Ivy::_getMessages, error message received from ".
	    "$senderName : \"$valeurs\"");
      }

      elsif ($type == DELREGEXP) {			# D E L   R E G E X P
	# on vire la regexp des regexps verifiées
#	  printf ("DBG> delete id $id\n");
	  $self->[sendRegList]{$appSock}->[$id] = undef ;
	  my $regexp = $self->[sendRegListSrc]{$appSock}->[$id];
          $self->[sendRegListSrc]{$appSock}->[$id] = undef;
	  my $host = _getHostByAddr ($addr);
          &_scanConnStatus ($self, $senderName, 'unsubscribing', "$host:$peerPort" , $regexp);
	}

      elsif ($type == ENDREGEXP) {			# E N D   R E G E X P
	# on envoie le message ready uniquement a celui qui nous
	# a envoye le message endregexp, et uniquement si on a
	# à la fois envoyé le end regexp, et reçu le endregexp de l'autre
	$self->[readyToSend]->{"$addr:$peerPort"} = 0 unless
	  exists $self->[readyToSend]->{"$addr:$peerPort"};
	if (++($self->[readyToSend]->{"$addr:$peerPort"}) == 2) {
	  $self->_sendMsgTo ($appSock, \$self->[messWhenReady]);
	}

	# on passe de l'etat Connecte a l'etat Ready
	$self->[cnnxion]{"$addr:$peerPort"} =~ s/^\004//g;
	$senderName = $self->[cnnxion]{"$addr:$peerPort"};
	unless (exists $self->[appliList]{$senderName}) {
	  $self->[appliList]{$senderName} = [$appSock];
	}
	else {
	  push @{$self->[appliList]{$senderName}},  $appSock;
	}

        my $host = _getHostByAddr ($addr);
        $self->_scanConnStatus ($senderName, "new", "$host:$peerPort", undef);
      }

      elsif ($type == APP_NAME) {
	# etat Connecte1558
	if (($self->[appName] eq $valeurs) && $^W) {
	  carp "\033[1mWarning in Ivy::_getMessages, there is already an instance of ".
	    "$self->[appName] \033[m" ;
	}

	$senderName = $valeurs;
	$self->[cnnxion]{"$addr:$peerPort"} = "\004$valeurs";
	$nameByHandle{$appSock}=_getHostByAddr($addr) .":$peerPort";
      }

      elsif ($type == DIRECT_MSG) {

	if (defined $self->[directCbList][$id]) {
	  my @cb = @{$self->[directCbList][$id]};
	  my $cb = shift @cb;
	  if (ref($cb) ne 'CODE') {
	    my $method = shift @cb;
	    $cb->$method(@cb, $valeurs);
	  }
	  else {
	    &$cb (@cb, $valeurs);
	  }
	}
	else {
	  $self->_sendErrorTo ($appSock, "DIRECT ID $id inconnue");
	  carp "Warning in Ivy::_getMessages, received a DIRECT message with ".
	    "unknown id $id from $senderName :\n\"$mess\"";
	}
      }

      elsif ($type == DIE) {
	# il faut quitter
	# on commence par appeler la callback de fin
	my @cb = @{$onDieFunc};
	my $cb = shift @cb;
	if (ref($cb) ne 'CODE') {
	  my $method = shift @cb;
	  $cb->$method(@cb);
	}
	else {
	  &$cb (@cb);
	}
	# on avertit les autres qu'on se barre
#	my $adr = $self->_inetAdrByName ($senderName) ;
#	carp "Notice in Ivy::_getMessages,  received a suicide request from " . "$senderName ($adr) ... exiting" if $^W;
	# adios
	Ivy::exit ();

      }

      elsif ($type == PING) {
	# si on recois un ping, on envoie un pong
	$self->_pong ($appSock, $id);
      }

     elsif ($type == PONG) {
	if (exists $self->[pongQueue]->{$appSock}) {
	  my ($pingid, $time, $funcRef) =  @{$self->[pongQueue]->{$appSock}};
#	  printf ("DBG>>> stocked Id = $pingid;; message id = $id\n");
	  &$funcRef ((Time::HiRes::time()-$time)*1000, $nameByHandle{$appSock})
	    if ($pingid == $id);
	  delete $self->[pongQueue]->{$appSock};
	}
      }

     elsif ($type == DUMP_TABLES) {
	# si on recois un ping, on envoie un pong
	$self->_dumpTable ($appSock, $id);
      }

     elsif ($type == DUMP_TABLES_FILE) {
	if (exists $self->[pongQueue]->{$appSock}) {
	  my ($pingid, $funcRef) =  @{$self->[pongQueue]->{$appSock}};
#	  printf ("DBG>>> stocked Id = $pingid;; message id = $id valeur=$valeurs\n");
	  &$funcRef ($valeurs, $nameByHandle{$appSock})
	    if ($pingid == $id);
	  delete $self->[pongQueue]->{$appSock};
	}
      }


      else  {
	_$self->sendErrorTo ($appSock, "TYPE DE MESS $type inconnu");
	warn ("Warning in Ivy::_getMessages, received a message of unknown ".
		" type $type from $senderName :\n\"$mess\"");
      }
    }
    return 0;
} # end _getMessages

############### METHODE	SEND WANTED REGEXP
sub _sendWantedRegexp ($$)
{
  my ($self, $appSock) = @_;
  my $connSock = $self->[connSock] ;
  my $msg;
  # on envoie le message "Nom appli"
  _univSend ($self, $appSock, sprintf (MSG_FMT, APP_NAME, $connSock->sockport,
				       $self->[appName]));
  # on envoie les regexps
  for (my $id = 0; $id <= $#{$self->[recCbList]}; $id++) {
    next unless defined $self->[recCbList][$id]->[1]->[0];
    $msg = sprintf (MSG_FMT, REGEXP, $id, $self->[recCbList][$id]->[0]);
    _univSend ($self, $appSock, \$msg);
    #	print sprintf ("DBG> %s %d %s\n",
    #			 'REGEXP', $id, $self->[recCbList][$id]->[0]);
  }
  # on envoie le message de fin d'envoi de regexps
  _univSend ($self, $appSock, sprintf (MSG_FMT, ENDREGEXP, 0, ""));

  # on envoie le message ready uniquement a celui qui nous
  # a envoye le message endregexp, et uniquement si on a
  # à la fois envoyé le end regexp, et reçu le endregexp de l'autre
  my $addr = $appSock->peeraddr();
  my $peerPort = $appSock->peerport() ;
  $self->[readyToSend]->{"$addr:$peerPort"} = 0 unless
    exists $self->[readyToSend]->{"$addr:$peerPort"};
  if (++($self->[readyToSend]->{"$addr:$peerPort"}) == 2) {
    $self->_sendMsgTo ($appSock, \$self->[messWhenReady]);
  }
} # end _sendWantedRegexp

############### METHODE	SEND LAST REGEXP TO ALLREADY CONNECTED
sub _sendLastRegexpToAllreadyConnected ($$)
{
  my ($self, $id) = @_;
  my $msg =  sprintf (MSG_FMT, REGEXP, $id, $self->[recCbList][$id]->[0]);
  foreach my $fd (values %{$self->[sockList]}) {
    _univSend ($self, $fd, \$msg);
  }
} # end _sendLastRegexpToAllreadyConnected

############### METHODE	INET ADR BY NAME
sub _inetAdrByName ($$) {

  my ($self, $appName) = @_;

  my $addrInet = (grep ($self->[cnnxion]{$_} eq $appName,
			keys %{$self->[cnnxion]}))[0];

  return ("unknow") unless defined $addrInet;

  my ($port) = $addrInet =~ /:(.*)/;
  my $addr = substr ($addrInet,0,4);
  my $host =  _getHostByAddr ($addr);
  return "$host:$port";
} # end _inetAdrByName


############### PROCEDURE	REMOVE FILE DESCRIPTOR
sub _removeFileDescriptor ($$$)
{
  my ($self, $fd, $callBy) = @_;

  unless (defined $fd) {
#    syswrite ($trace, "_removeFileDescriptor : *UN*inititialized fd, don't do anything\n");
    return;
  }



  # on s'est deja occupe de lui
  return unless exists $self->[sockList]->{$fd};
  my $diedAppName = _getNameByFileDes ($self, $fd);

  # on efface les structures de donnees associees au fd
  # on vire ce fd des fd a scruter dans la bcle d'evenements
  # uniquement si on est dans le thread principal
  # sinon le select merde salement sur ce coup
  my $peerPort = $fd->peerport() ;
  $peerPort = 0 unless defined  $peerPort;

#   syswrite ($trace, sprintf ("_removeFileDescriptor  : suppression dans le fdset de %s[%s]:%d\n",
# 			     (gethostbyaddr ($fd->peeraddr(),AF_INET))[0],
# 			     join (':', unpack ('C4', $fd->peeraddr())),
# 			     $fd->peerport()));
  &$fileEventFunc ($fd, '') ;
  delete $self->[sendRegList]->{$fd};
  delete $self->[sendRegListSrc]->{$fd};
  delete $self->[sockList]->{$fd};
  delete $self->[bufRecByCnnx]->{$fd};
  delete $self->[bufEmiByCnnx]->{$fd};

  $fd->close();

  # remove all occurence of fd from $self->[appliList]
  foreach my $name (keys %{$self->[appliList]}) {
    for (my $i=0; $i < scalar (@{$self->[appliList]{$name}}); $i++) {
      my $fdp = $self->[appliList]{$name}->[$i];
      if ($fd eq $fdp) {
	delete ($self->[appliList]{$name}->[$i]);
      }
    }
  }

  unless (defined $diedAppName) {
    warn "Ivy::__removeFileDescriptor (called by $callBy) : disconnection of NONAME\n" if $^W;
    return;
  }

  my $addrInet = (grep ($self->[cnnxion]{$_} eq $diedAppName,
			keys %{$self->[cnnxion]}))[0];

  unless (defined $addrInet) {
    carp "Warning in Ivy::_removeFileDescriptor (called by $callBy) : disconnection of $diedAppName with ".
      "addrInet not defined\n";
    return;
  }

#   syswrite ($trace, 
# 	    sprintf ("DBG> _removeFileDescriptor : deconnection de %s ($diedAppName)\n", $self->_inetAdrByName ($diedAppName)));

  delete $self->[cnnxion]{$addrInet};
  delete $nameByHandle{$fd};

  # on vire l'entree correspondant a ce canal dans la liste des
  # regexps par canal

  my $addr = substr ($addrInet,0,4);
  my $host =  _getHostByAddr ($addr);
  $self->_scanConnStatus ($diedAppName, "died", "$host:$peerPort", undef) ;
} # end _removeFileDescriptor


############### METHODE	SEND ERROR TO
sub _sendErrorTo ($$$)
{
  my ($self, $fd, $error) = @_;

  _univSend ($self, $fd, join (' ', ERROR, "0\002$error\n"));
} # end _sendErrorTo


############### METHODE	PONG
sub _pong ($$$)
{
  my ($self, $fd, $pongId) = @_;

#  printf ("DBG>>> PONG Id =   $pongId\n"); 
  _univSend ($self, $fd,  sprintf (MSG_FMT, PONG, $pongId, ""));
} # end _pong

############### METHODE	_DUMP_TABLE
sub _dumpTable ($$$)
{
  my ($self, $fd, $pongId) = @_;
  my ($fh, $tmpFileName) = File::Temp::tempfile("IvyTable_XXXXX",  SUFFIX => '.txt', DIR => $ENV{'TMPDIR'});
#  print ("DBG> tmpFileName=$tmpFileName\n");
  my $ofs = $,;
  $, = ', ';
  print $fh <<EOF;
ivyBus = $ivyBus
appName = $self->[appName]
selectTimout = $selectTimout
loopMode = $loopMode
messWhenReady = $self->[messWhenReady]
blockOnSlowAgent = $self->[blockOnSlowAgent]
topicRegexps = @{$self->[topicRegexps]}
useMulticast = $self->[useMulticast]
broadcastPort = $self->[broadcastPort]
readyToSend = %{$self->[readyToSend]}
neededApp = @{$self->[neededApp]}

TABLE DE REGEXPS :

EOF

  foreach my $appSock (keys  %{$self->[sendRegListSrc]}) {
    my $appName = $self->_getNameByFileDes ($appSock);
#    my $ap2 = 	$self->[cnnxion]{$nameByHandle{$appSock}};
#    $ap2 =~ s/^\004//g;
    print $fh "POUR l'application ${appName}[$nameByHandle{$appSock}] : \n";
    for (my $idx=0; $idx < scalar (@{$self->[sendRegListSrc]{$appSock}}); $idx++) {
      printf $fh ("id[$idx] => '%s'\n",
	      defined ($self->[sendRegListSrc]{$appSock}->[$idx])
	      ? $self->[sendRegListSrc]{$appSock}->[$idx]
	      : "DELETED or UNDEFINED");
    }
    #    print $fh join ("\n", @{$self->[sendRegListSrc]{$appSock}});
    print $fh "\n-----------------------------------\n";
  }

  close ($fh);
  $, = $ofs;


#  printf ("DBG>>> DUMP_TABLE Id = $pongId tmpFile= $tmpFileName\n"); 
  _univSend ($self, $fd,  sprintf (MSG_FMT, DUMP_TABLES_FILE, $pongId, $tmpFileName));
} # end _pong


############### METHODE	SEND ERROR TO
sub _sendDieTo ($$)
{
  my ($self, $fd) = @_;

  _univSend ($self, $fd, join (' ', DIE, "0\002\n"));
} # end _sendDieTo


############### METHODE	SEND MSG TO
sub _sendMsgTo ($$\$)
{
  my ($self, $fd, $msg) = @_;
  my $id = -1;
  my $total = 0;
  my @args = (); # tableau passé en reference aux fonctions
		 # anonymes compilées par eval pour receuillir les arguments filtrés
                 # par les regexp à envoyer sur le tuyau
  my $sent;
  my $regexpFunc;
  my $bufEmiRef;

  # pour toutes les fonctions de filtrage de regexp
  foreach  $regexpFunc (@{$self->[sendRegList]{$fd}}) {
    $id++;
    #	$total += &{$regexpFunc} (\$msg) if defined $regexpFunc;
    next unless defined $regexpFunc;
    next unless &{$regexpFunc} ($msg, \@args);
    next unless @args;
    $total ++;
    shift @args;
    $args[$#args] .= "\003" if @args;
    $bufEmiRef = \($self->[bufEmiByCnnx]->{$fd});
    my $enCongestion = $$bufEmiRef ? 1 : 0;
    $$bufEmiRef .= sprintf (MSG_FMT, MSG, $id, join ("\003",@args));
    next if $enCongestion;
    $sent = send ($fd, $$bufEmiRef, 0);
    unless (defined $sent) {
      # y a rien à faire
    } elsif ($sent ==   length ($$bufEmiRef)) {
      $$bufEmiRef = "";
    } elsif ($sent >= 0) {
      substr ($$bufEmiRef, 0, $sent, '');
	  _callCongestionCb ($self, $fd, 1);
      if  ($self->[blockOnSlowAgent]) {
	my $win = '';
	vec($win, fileno ($fd), 1) = 1;
	select (undef, $win, undef, undef);
      }
    } else {
      $self->_removeFileDescriptor ($fd, '_sendMsgTo') ;
    }
  }

  return ($total);
} # end _sendMsgTo



############### METHODE	UNIV SEND
sub _univSend ($$$)
{
  my ($self, $fd, $msg) = @_;

  unless (defined $fd) {
    carp "WARN _univSend fd is undefined, message will not be sent\n" if $^W;
    afficher la call stack
  }
  my $bufEmiRef = \($self->[bufEmiByCnnx]->{$fd});
  my $enCongestion = $$bufEmiRef ? 1 : 0;
  if (ref $msg) {
    $$bufEmiRef .= $$msg;
  } else {
    $$bufEmiRef .= $msg;
  }
  return if $enCongestion;
  my $sent = send ($fd, $$bufEmiRef, 0);
  unless (defined $sent) {
    # y a rien à faire
  } elsif ($sent ==   length ($$bufEmiRef)) {
    $$bufEmiRef = "";
  } elsif ($sent >= 0) {
    substr ($$bufEmiRef, 0, $sent, '');
    _callCongestionCb ($self, $fd, 1);
    if  ($self->[blockOnSlowAgent]) {
      my $win = '';
      vec($win, fileno ($fd), 1) = 1;
      select (undef, $win, undef, undef);
    }
  } else {
    if ($!{EWOULDBLOCK}) {
      # Aucun octet n'a été envoyé, mais le send ne rend pas 0
      # car 0 peut être une longueur passée au send, donc dans ce cas
      # send renvoie -1 et met errno a EWOULDBLOCK
      _callCongestionCb ($self, $fd, 1);
      if  ($self->[blockOnSlowAgent]) {
	my $win = '';
	vec($win, fileno ($fd), 1) = 1;
	select (undef, $win, undef, undef);
      }
    }
    $self->_removeFileDescriptor ($fd, '_univSend') unless ($!{EAGAIN} || $!{EWOULDBLOCK}|| $!{EINTR} || $!{EMSGSIZE} || $!{ENOBUFS})
  }
}


############### PROCEDURE TK FILE EVENT
sub _tkFileEvent ($$)
{
  my ($fd, $cb) = @_;

  Tk::fileevent ('', $fd, 'readable', $cb) ;
} # end _tkFileEvent


############### PROCEDURE SCAN AFTER
sub _scanAfter ()
{
  my $stamp = gettimeofday ();
  $selectTimout = MAX_TIMOUT;
  foreach my $afk (keys %afterList) {
    my $af = $afterList{$afk};
    # si ce timer est a declencher
    if ($af->[2] <= $stamp) {
      # on traite : le temps de declencher le cb est arrive
      if (ref  $af->[3] eq 'CODE') {
	&{$af->[3]};
      }
      else {
	my ($cb, @args) = @{$af->[3]};
	&$cb (@args);
      }
      # si c'est un repeat on le reconduit
      if ($af->[0]) {
	$af->[2] = $stamp + $af->[1] ;
	$selectTimout = $af->[1] if $af->[1] < $selectTimout;
      }
      else {
	# si c'est un after on le vire
	afterCancel ($afk);
      }
    }
    else {
      my $timeTotrigg = $af->[2] - $stamp;
      $selectTimout = $timeTotrigg if $timeTotrigg < $selectTimout;
    }
  }
} # end _scanAfter


############### METHODE	SCAN CONN STATUS
sub _scanConnStatus ($$$@)
{
  my ($self, $appname, $status, @addr) = @_;

  my (%readyApp, @nonReadyApp);

  foreach (values %{$self->[cnnxion]}) {
    next if $_ eq "1";
    $readyApp{$_}++ unless /^\004/; # connecte mais pas ready
  }

  foreach (@{$self->[neededApp]}) {
    push (@nonReadyApp, $_) unless exists $readyApp{$_};
  }

  # par compatibilite avec l'ancienne version, on envoie comme
  # deux premiers arguments une ref sur la liste des applis presentes,
  # une ref de la liste des applis absentes, mais on rajoute comme troisieme
  # argument une ref sur une table de hash : comme clef les
  # applis presentes, comme valeur le nombre d'applis ayant ce nom,
  # de facon a detecter plus facilement quand il y a trop d'applis
  # de meme nom sur le meme bus.
  # les nouveaux arguments sont:
  # le 4eme arg est l'appli nouvelle, deconnecté, qui s'abonne ou se desabonne
  # le 5eme arg est le statut (actuellement: 'subscribing'|'filtered'|'unsubscribing'|'died'|'new')
  # le 6eme arg est l'addresse de la machine sur laquelle tourne l'agent
  # le 7eme arg est la regexp si c'est un abonnement ou desabonnement
  &{$self->[statusFunc]} ([keys %readyApp], \@nonReadyApp, \%readyApp, $appname, $status, @addr);
} # end _scanConnStatus


############### METHODE	TO BE PRUNED
sub _toBePruned ($$$)
{
  my ($self, $from, $regexp) = @_;

  # for message purposes, removing the \004 which indicates the connection status

  my ($cleaned_from) = $from =~ /\004?(.*)/ ;
  # print "DBG> $from s'abonne à nouvelle regexp '$regexp'\n";

  # testing the received regexp for avoiding illformed regexp
  eval {my $test = "a" =~ /$regexp/ } ;
  if ($@) {
      warn "Warning in Ivy::_toBePruned, receiving ill-formed regexp: '$regexp' from '$cleaned_from'" ;
      return 1};


  # si il n'y a pas de liste de sujets, on ne
  # filtre pas
  return 0 unless @{$self->[topicRegexps]};

  unless ($regexp =~ /^\^/) {
    #print "DBG> regexp non ANCREE de $from : $regexp\n";
    return (0);
  }

  if ($regexp =~ /^\^(\w+)/) {
    my $topic = $1;
    if (grep (/$topic/, @{$self->[topicRegexps]})) {
      # on a trouve ce topic : on ne filtre pas la regexp
      #print "DBG> on garde de $from : $regexp\n";
      return (0);
    }
    #print "DBG> on ELIMINE de $from : $regexp\n";
    return (1);
  }
  else {
    #print "DBG> on garde de $from : $regexp\n";
    return (0);
  }
} # end _toBePruned


############### PROCEDURE PARSE IVY BUS PARAM
sub _parseIvyBusParam ($)
{
  my $ivyBus = shift;

  my ($ivyNetworks, $ivyPort) = $ivyBus =~ /^(.*):(.*)/;

  my $useMulticast = 0;

  croak ("Error in Ivy::_parseIvyBusParam, illegal bus address format: $ivyBus\n")
    unless  $ivyPort =~ /^\d+$/;

  my @ivyAddrInet = ();

  $ivyNetworks =~ s/ //g;
  my @broadcastAddrs = split (',', $ivyNetworks);

  foreach my $netAddr (@broadcastAddrs) {
    $netAddr = BROADCAST_ADDRS if
    (($netAddr eq '') || ($netAddr =~ /^127/) || ($netAddr =~ /^loopback/));

    # deux cas de figure : on a un nom de sous reseau, ou
    # une adresse ip de la forme \d+.\d+....
    my $netAddrInet;

    if ($netAddr !~ /^\d+(.\d+)+/) {
      # on a un nom de reseau, il faut trouver son adresse ip
      # on contourne un bug : Si les adresses sont incompletes
      # dans la map network : 143.196.53 au lieu de 143.196.53.255
      # getbyname renvoie une adresse de type 0.143.196.53.
      # on doit donc faire un decalage des octets vers la gauche,
      # chaque 0 qui sort a gauche est remplace par un 255 a droite.
      if ($useMulticast) {
	  carp "Warning in Ivy::_parseIvyBusParam, cannot mix broadcast and multicast.\n\tSkipping broadcast address $netAddr";
	  next;
      }
      my $networkAddr = getnetbyname ($netAddr);
      unless (defined $networkAddr) {
	warn ("Warning in Ivy::_parseIvyBusParam, network $netAddr is unknown\n");
	next;
      }

      my @dummyNetAddr = unpack ("CCCC", pack ('N', $networkAddr));
      while (!$dummyNetAddr[0]) {
	# tant que le premier octet est 0, on decale vers la gauche et
	# ont fait rentrer un 255 sur la droite
	shift @dummyNetAddr;
	push (@dummyNetAddr, 255);
      }
      $netAddrInet = pack ("CCCC", @dummyNetAddr);
    } else {
      # on a deja une adresse ip, on rajoute les .255
      # a la fin s'ils ont ete omis.
      ($netAddr .= ".255.255.255") =~ s/^((\d+\.){3}\d+).*/$1/;
      my ($dClass) = $netAddr =~ /^(\d*)/ ;
      if ($dClass >= 224 and $dClass <= 239) { # this adress is for multicast
	  if (!$useMulticast) { # removing all broadcast addresses
	      carp "Warning in Ivy::_parseIvyBusParam, cannot mix broadcast and multicast.\n\tSkipping previous broadcast address" if (scalar @ivyAddrInet);
	      @ivyAddrInet = ();
	      $useMulticast = 1;
	  }
      }
      else { # this adress is for broadcast
	  if ($useMulticast) { # we are already in multicast, forget this new address
	      carp "Warning in Ivy::_parseIvyBusParam, cannot mix broadcast and multicast.\n\tSkipping broadcast address $netAddr";
	      next;
	  }
      }
      $netAddrInet =  inet_aton ($netAddr);
      push (@ivyAddrInet,  pack_sockaddr_in ($ivyPort, $netAddrInet));
  }
}
  unless (scalar @ivyAddrInet) {
      push (@ivyAddrInet,  pack_sockaddr_in ($ivyPort, inet_aton(BROADCAST_ADDRS)));
  }

  foreach my $ivyAddr (@ivyAddrInet) {
      my ($port, $iaddr) = unpack_sockaddr_in($ivyAddr);
      my $iaddrH = unpack ("H8",$iaddr);
      my $iaddrInt = inet_ntoa($iaddr);
      if ($useMulticast) {print "Multicasting"}
      else {print "Broadcasting"}
      print " on network $iaddrInt ($iaddrH) on port $ivyPort\n";
  }

  return ($useMulticast, $ivyPort, \@ivyAddrInet);
} # end _parseIvyBusParam

############# Procedure _SUBSTITUTE ESCAPED CHAR
sub _substituteEscapedChar ($$)
{
  my ($scope, $reg) = @_;

  my %escapeRegexp = REG_PERLISSISME;
  # Si on fait la substitution dans une classe de caractere
  # on elimine les crochets.
  grep ($escapeRegexp{$_} =~ s/[\[\]]//g, keys %escapeRegexp)
    if ($scope eq 'inside') ;

  $reg =~ s/\\([wWsSdDne])/$escapeRegexp{$1}/ge;
  return $reg;
} # end _substituteEscapedChar

############# Procedure __CALL CONGESTION CALLBACK
sub _callCongestionCb ($$$)
{
  my ($self, $fd, $congestion) = @_;
  my $addrInet;
  my $appName = _getNameByFileDes ($self, $fd);

  if ($loopMode == LOCAL_MAINLOOP) {
    if ($congestion) {
      $localLoopSelWrite->add ($fd);
    } else {
      $localLoopSelWrite->remove ($fd);
    }
  } else {
    if ($congestion) {
      Tk::fileevent ('', $fd, 'writable',
		     sub {
		       my $bufEmiRef = \($self->[bufEmiByCnnx]->{$fd});
		       my $sent = send ($fd, $$bufEmiRef, 0);
		       unless (defined $sent) {
			 # y a rien à faire
		       } elsif ($sent ==   length ($$bufEmiRef)) {
			 $$bufEmiRef = "";
			 _callCongestionCb ($self, $fd, 0);
		       } elsif ($sent >= 0) {
			 substr ($$bufEmiRef, 0, $sent, '');
		       } else {
			 $self->_removeFileDescriptor ($fd, '_callCongestionCb') unless ($!{EAGAIN} || $!{EWOULDBLOCK}|| 
											 $!{EINTR} || $!{EMSGSIZE} || $!{ENOBUFS});
		       }
		     });
    } else {
      Tk::fileevent ('', $fd, 'writable', '');
    }
  }

  if (defined $appName) {
    $addrInet = $self->_inetAdrByName ($appName);
  } else {
    $appName = 'NONAME'; $addrInet = 'undef';
  }

  &{$self->[slowAgentFunc]} ($appName, $addrInet, $congestion);
}

############# Procedure __GET NAME BY FILEDES

sub _getNameByFileDes ($$)
{
  my ($self, $fd) = @_;
  my $appName = 'NONAME';
 EXT_LOOP:
  foreach my $name (keys %{$self->[appliList]}) {
    foreach my $fdp (@{$self->[appliList]{$name}}) {
      if ($fd eq $fdp) {
	$appName = $name;
	last EXT_LOOP;
      }
    }
  }
  return $appName;
}


sub  _getHostByAddr ($)
{
  my $addr = shift;

  unless (defined $addr) {
    warn "_getHostByAddr : no argument\n";
    return "EMPTY_ADDR";
  } elsif ((length ($addr)) != 4) {
    warn "_getHostByAddr : bad argument (len != 4)\n";
    return "BAD_ADDR";
  }

  $hostNameByAddr{$addr} = (gethostbyaddr ($addr, AF_INET))[0] || inet_ntoa($addr)
    unless exists $hostNameByAddr{$addr};

  return $hostNameByAddr{$addr};
}



sub _regexpGen ($$$)
{
  my ($min, $max, $withDecimal) = @_;

#  print ("DBG> min=$min max=$max withDecimal=$withDecimal\n");

  ($min, $max) =  ($max, $min) if $min > $max;

  $min = int ($min);
  $max = int ($max);

  my ($decimalPart,$boundDecimalPart, $reg) = ('') x 3;

  if ((!defined $withDecimal) || ($withDecimal ne 'i')) {
    $decimalPart = '(?:\.\d+)?';
    $boundDecimalPart = '(?:\.0+)?';
  }

  if ($min == $max) {
    $reg= $min;
  } elsif  ($min < 0) {
    if  ($max < 0) {
      $reg = '\-(?:' .  _strictPosRegexpGen (-$max, -$min, $decimalPart, $boundDecimalPart). ')';
    } elsif  ($max == 0) {
      $reg = "(?:0${boundDecimalPart})|(?:-0${decimalPart})|-(?:" .
	_strictPosRegexpGen (1, -$min, $decimalPart,$boundDecimalPart ) . ')';
    } else {
      $reg ='(?:' . regexpGen ($min, 0,$withDecimal) . '|' .  regexpGen (0, $max, $withDecimal). ')' ;
    }
  } elsif  ($min == 0) {
    $reg = "(?:0${decimalPart})|" . _strictPosRegexpGen (1, $max, $decimalPart,$boundDecimalPart) ;
  } else {
    $reg = _strictPosRegexpGen ($min, $max, $decimalPart,$boundDecimalPart);
  }
  return ("(?:$reg)(?![\\d.])");
}



sub _strictPosRegexpGen ($$$$)
{
  my ($min, $max, $decimalPart,$boundDecimalPart) = @_;
  carp "min[$min] sould be <=  max[$max]\n " unless ($min <= $max);
  carp "min[$min] and max[$max] should be strictly positive\n " unless (($min >0) && ($max > 0));

#  my $fixBound ;
#  $max -- if ($fixBound = ($decimalPart ne '') && ((int ($max /10) *10) != $max));

  if ($min == $max) {
    return ($min);
  }

  $max -- ;

  my @regexps ;
  my $nbRank = length ($max);
  my ($rank, $lmax) ;

  do {
    ($lmax, $rank) = _nextMax ($min, $max);
    push (@regexps, _genAtRank ($min, $lmax, $rank));
    $min = $lmax+1;
  }   while ($lmax != $max) ;

  my $regexp = join ('|', map ("(?:$_$decimalPart)", @regexps));
  $max ++;
  $regexp .= "|(?:${max}$boundDecimalPart)";

  return ($regexp);
}


sub _genAtRank ($$$)
{
  my ($min, $max, $rank) = @_;
  my $reg = _genPreRank ($min, $max, $rank);
  $reg .= _genRank ($min, $max, $rank);
  $reg .= _genPostRank ($rank);
  return ($reg);
}


sub _nextMax ($$)
{
 my ($min, $max) = @_;
 my $nextMax;

 # on a les unités au debut
 my (@min) = reverse split ('', $min);
 my (@max) = reverse split ('', $max);
 my $nbDigit = scalar (@max);
 my ($rankRev, $rankForw, $rank) = (0, $nbDigit-1, 0) ;

 # on rajoute des 0 en face si min n'a pas le même nombre de digits que max
 push (@min, ('0') x ($#max - $#min)) if ($#min != $#max);

 # on calcule le rang concerné par le prochain intervale

 # en partant des unitées (digit de poids faible), premier champ de min != 0
 while (($min[$rankRev] == 0) && $rankRev < $nbDigit) {$rankRev++} ;
 # printf ("DBG> min = $min[0]|$min[1]|$min[2] rankRev=$rankRev, nbDigit=$nbDigit\n");

 # en partant du digit de poids fort, premier champ de max != du même champ
 while (($min[$rankForw] == $max[$rankForw]) && $rankForw > 0)  {$rankForw--};
# printf ("DBG> min = $min[0]|$min[1]|$min[2] rankForw=$rankForw, nbDigit=$nbDigit\n");

 if ($rankForw <= $rankRev) {
   $rank = $rankForw;
   $min[$rankForw]= $max[$rankForw] - ($rankForw ? 1 : 0);
   @min[0 .. $rankForw-1]= (9) x ($rankForw);
 } else {
   $rank =  $rankRev;
   @min[0 .. $rankRev]= (9) x ($rankRev+1);
 }

# print ("DBG> NEWmin = $min[0]|$min[1]|$min[2]\n");
 $nextMax = join ('',reverse @min);
 $nextMax = $max if $nextMax > $max;

 return ($nextMax, $rank+1);
}



sub _genPreRank ($$$)
# les invariants du min
{
  my ($min, $max, $rank) = @_;

  $min = $min + 0; # force scalar to be evaluated as numérical
  $max = $max + 0; # instead string (eliminate leading zeroes)
  my $a = substr ($min, 0, (length ($min) - $rank));
  my $b = substr ($max, 0, (length ($max) - $rank));
  carp "genPreRank error $min, $max are not invariant @ rank $rank\n" if $a ne $b;
  return $a;
}


sub _genRank ($$$)
{
  my ($min, $max, $rank) = @_;
  my $syl ;

  my $a = substr ($min, (length ($min) - $rank), 1);
  my $b = substr ($max, (length ($max) - $rank), 1);

  $min = _min ($a, $b);
  $max = _max ($a, $b);

  if (($min == 0) && ($max == 9)) {
    $syl = '\d';
  } elsif ($min == $max) {
    $syl = $min;
  } elsif ($max == $min+1) {
    $syl = "[${min}${max}]"
  } else {
    $syl =  "[$min-$max]";
  }

  return ($syl);
}


sub _genPostRank ($)
{
  my $rank = shift;

  return "" if ($rank <= 1);
  return ($rank == 2) ? '\d' : sprintf ('\d{%d}', $rank -1);
}

sub _max ($$)
{
  my ($a,$b) = @_;
  return ($a > $b) ? $a : $b;
}

sub _min ($$)
{
  my ($a,$b) = @_;
  return ($a > $b) ? $b : $a;
}


1;

__END__

=head1 NAME

Ivy - Perl extension for implementing a software bus

=head1 SYNOPSIS

use Ivy;

=head1 DESCRIPTION

The Ivy perl module implements a software bus that provides easy
communication between applications. Messages are broadcast as ASCII strings
over a network defined by a list of domains and a port.
Messages are received if they match a regular expressions and if your application
is on the same network as remote ones.
Before receive or send message you must call 'init', and 'new' class methods,
followed by 'start' method.
When you quit your application don't forget to call 'exit' class methods.

=head1 CLASS METHODS

=over 2

=item B<init>

 Ivy->init(...);
 Ivy::init(...);

Allows one to define global parameters which may be used as default ones
at object creation time.

Parameters are :

=over 4

=item B<-loopMode =E<gt> 'TK'|'LOCAL'>

Mode of events loop among TK or LOCAL. According to this mode, you must
use Ivy->mainLoop or Tk::MainLoop(3)

=item B<-appName =E<gt> 'your app ivy name'>

Name of your application used to identify on ivy bus.

=item B<-ivyBus =E<gt> 'domain 1,...,domain n:port number'>

A list of domains (may be empty), followed by a port number where to broadcast messages.
If the domain list is empty (i.e. parameter is ':port number'), broadcast will be done
on localhost (i.e. '127:port number'). Default is the value of the environment variable
IVYBUS and if it is not defined the default is 127:2010.

Since V4.12, it is possible to use multicast (ie. with a domain between 224.0.0.0 and 239.255.255.255). You must be aware than when multicast is used, udp broadcast (defined in the B<-ivyBus> paramter) are skipped. You should also probably avoid using the 244.x.x.x domain often used for networking management.

=item B<-messWhenReady =E<gt> 'your message when ready'>

Synchronisation message sent when application is ready to receive and send
messages.

=item B<-onDieFunc =E<gt> [\&yourdiefunc, @parameters]>

=item B<-onDieFunc =E<gt> [$an_object, \&a_method, @parameters]>

A callback or method to call when your application receive a suicide request.
Do not call exit() in the callback, Ivy will do it for you.

The prototype of your callback must be as follows:

 sub MyCallback {
   my @parameters = @_;
   ...
 }

The prototype of your method must be as follows:

 sub MyMethod {
   my ($self, @parameters) = @_;
   ...
 }

=item B<-filterRegexp =E<gt> ['subject 1', ..., 'subject n']>

Optimize communication using this option. Regexps
which don't match these subjects are removed.

=item B<Example:>

  Ivy->init(-loopMode	  => 'TK',
	    -appName	  => 'MyWonderfulApp',
            -onDieFunc	  => [\&restorecontext] ,
            -filterRegexp => ['MyWonderfulApp', 'ClockStart', 'ClockStop']);

=back

=item B<new>

 Ivy::new(...);
 Ivy->new(...);

Check parameters, and create an Ivy bus object. You must call
Ivy->init before creating a bus.

Parameters are :

=over 4

=item B<-appName =E<gt> 'your application name'>

Name of your application used to identify it with other applications
connected on the same bus.

=item B<-ivyBus =E<gt> 'domain 1,...,domain n:port number'>

A list of domains, followed by port number where to broadcast messages.
Default is 127:2010

=item B<-messWhenReady =E<gt> 'your message when ready'>

Synchronisation message sent when your application is ready to receive and send
messages.

=item B<-onDieFunc =E<gt> [\&yourdiefunc, @parameters]>

=item B<-onDieFunc =E<gt> [$an_object, \&a_method, @parameters]>

A callback or method called when your application receives a suicide request.
DO NOT CALL exit() in the callback, Ivy will do it for you.
The prototype of your callback must be as follows:

 sub MyCallback {
   my @parameters = @_;
   ...
 }

The prototype of your method must be as follows:

 sub MyMethod {
   my ($self, @parameters) = @_;
   ...
 }

=item B<-filterRegexp =E<gt> ['subject 1', ..., 'subject n']>

Optimize communication using this option. Regexps which don't match these subjects are removed.

=item B<-neededApp =E<gt> ['app 1', ..., 'app n']>

A list of applications that your application needs present on the bus
before running.

=item B<-statusFunc =E<gt> sub {}>

A callback which is called every time an agent C connects on the bus,
disconnects from the bus, subscribes to a regexp, or unsubscribes to a
regexp. When the agent A is stopping, this function is also called
inside the agent A for every other agents C on the bus, as they are
disconnecting. The first 3 parameters are a reference to an array of
connected agents Ci, a reference to an array of not connected agents
(according to the "-neededApp" argument of the new method / function),
a reference to a hash table of connected agents Ci (giving the number
of each agent). These 3 parameters are maintained for upwards
compatibility but should no more be used, since the following four
parameters are much easier to use: the name of an appearing /
disapearing or subscribing / filtered / unsubscribing agent C, its status either
"new" or "died" or "subscribing" or "unsubscribing", and the hostname
where this agent C is running / dying OR the subscribed / unsubscribed
regexp. If the hostname of this agent C is not known, it will be
replaced by its IP address.



Your callback could be:

 sub MyCallback {
   my ($ref_array_present, $ref_array_absent, $ref_hash_present,
       $appname, $status, $host, $regexp) = @_;

   # $status is either new or died

   my %present=%$ref_hash_present;
   foreach my $remoteapp (keys %present) {
     if ($present{$remoteapp} > 1) {
       print "n apps $remoteapp are presents on bus\n";
     }
   }
   if ($status eq "new") {
     print "$appname connected from $host\n";
   }
   elsif ($status eq "died") {
     print "$appname disconnected from $host\n";
   }
   elsif ($status eq "subscribing") {
	print "$appname subscribes to $regexp\n";
   }
   elsif ($status eq "filtered") {
	print "$appname subscribes to FILTERED $regexp check -filterRegexp option\n";
   }
   elsif ($status eq "unsubscribing") {
	print "$appname unsubscribed to $regexp\n";
   }
 }




=item B<-blockOnSlowAgent =E<gt> 0 or 1>

Behavior when the bus is being congested due to an ivy agent which
doesn't read messages sufficiently quickly. In blocking mode the local
app will block on a send, so it won't be interactive until the send
return, and it will at his turn don't read his pending message,
leading to a global sluggishness of the entire ivy bus.  In non
blocking mode the messages are stocked until they could be sent, so
the problem is the uncontrolled memory consumption.


=item B<-slowAgentFunc=E<gt> \&congestionFunc >

 A callback which is called every time a congestion event occurs. A
 congestion event is emitted each time an agent is being congested,
 or, after being congested is able to read his messages again.  The
 parameters are the name of the app, his address (hostname+port
 number), and the state, 1 for congested, 0 for able to read.

 Your callback could be:

sub congestionFunc ($$$)
{
  my ($name, $addr, $state) = @_;
  printf ("$name [$addr] %s\n", $state ? "CONGESTION" : "OK");
}



=item B<Example:>

 Ivy->new(-ivyBus => '156,157:2204',
          -onDieFunc => [\&restorecontext],
          -neededApp => ["DataServer", "HMI"],
	  -slowAgentFunc=> \&congestionFunc,
	  -blockOnSlowAgent => 1,
          -statusFunc => \&MyCallback);

=back






=item B<mainLoop>

 Ivy->mainLoop;
 Ivy::mainLoop;
 $ivyobj->mainLoop;

 main events loop, call local mainloop or Tk::MainLoop according so specified mode

=item B<stop>

 $ivyobj->stop;
 Ivy::stop;

To stop the Ivy main loop.

=back

=head1 OBJECT METHODS

=over 2

=item B<start>

 $ivyobj->start;
 Ivy::start;

You must call this after you are ready to communicate through an Ivy bus
and before you really communicate. The method returns the $ivyobj.

=item B<sendMsgs>

 $ivyobj->sendMsgs(@messages);
 Ivy::sendMsgs(@messages);

Send a list of messages. A message should not contain a '\n' or it will not be delivered.

 Example :
   $ivyobj->sendMsgs("Hello", "Don't Bother", "Y2K is behind us");

=item B<sendAppNameMsgs>

 $ivyobj->sendAppNameMsgs(@messages);
 Ivy::sendAppNameMsgs(@messages);

Send a list of messages preceded by your application's name.  A message should not contain a '\n' or it will not be delivered.

 Example :
   $ivyobj->sendMsgs("Hello World");
   # it will send "$appName Hello World" over the Ivy bus

=item B<bindRegexp>

 $ivyobject->bindRegexp($regexp, [\&callback, @cb_parameters]);
 Ivy::bindRegexp($regexp, [\&callback, @cb_parameters]);

 $ivyobject->bindRegexp($regexp, [$an_obj, \&method, @cb_parameters]);
 Ivy::bindRegexp($regexp, [$an_obj, \&method, @cb_parameters]);

This allows you to bind a regular expression to a
callback or method. The callback or method will be called for every
message that matches the regexp (case insensitive).
See perlre(1) to find how to write regexps.
Use the bracketing construct ( ... ) so that your callback is
called with the captured bits of text as parameters.
To unbind callback(s) associated to a regexp use bindRegexp with only
one argument, the regexp. Note that doing the same binding more than
once will induce multiple call of the same callback (this is usually a bug).

there is a special syntax for specifying numeric interval, in this case
the interval is locally transformed in a pcre regexp.
syntax is (?Imin#max[fi]). min and max are the bounds,
by default the regexp match decimal number, but if max bound is
followed by 'i', the regexp match only integers ex : (?I-10#20), (?I20#25i)
Note that due to the regexp generator limitation (which will perhaps be raised eventually)
the bounds are always integer.

Return value : regexpId

 Example :
   $ivyobject->bindRegexp("\w+ (\d+)", [\&callback, @cb_parameters]);
   $ivyobject->bindRegexp("\w+ ((?I-10#20i))", [\&callback, @cb_parameters]);

   # Your callback will be called with one more parameter which will be
   # the name of appli which send the message

   # Your callback and method must be like:
   sub callback {
     my ($sendername, @cb_parameters,
	 @matched_regexps_in_brackets) = @_;
     ...
   }

   sub method {
     my ($self, $sendername, @cb_parameters,
	        @matched_regexps_in_brackets) = @_;
     ...
   }

   # to unbind:
   $ivyobject->bindRegexp("\w+ (\d+)");

=item B<bindRegexpOneShot>

 $ivyobject->bindRegexpOneShot($regexp, [\&callback, @cb_parameters]);
 Ivy::bindRegexpOneShot($regexp, [\&callback, @cb_parameters]);

 bindRegexpOneShot behavior is similar at bindRegexp one, except that
 the callback is called once, it is similar as a  bindRegexp with an unbind in the callback
 but is simpler to write.

=item B<changeRegexp>

 $regexpId = $ivyobject->bindRegexp("initialRegexp", [\&callback, @cb_parameters]);
 $ivyobject->changeRegexp($regexpId, "newRegexp");
  or
 Ivy::changeRegexp($regexpId, "newRegexp");

This allow you to change the regexp of a previously made bindRegexp, the callback
will remain the same. This is equivalent to unbinding current regexp and binding the new regexp,
but in this last case the 2 op are not done in atomic manner, and you can miss a message
or receive it twice.


=item B<sendDirectMsgs>

 $ivyobj->sendDirectMsgs($to, $id, @msgs);
 Ivy::sendDirectMsgs($to, $id, @msgs);

Send a message a message to appli $to. This appli must have done a bindDirect before to accept this message. regexp matching is not used with direct Messages.  A message should not contain a '\n' or it will not be delivered.

=item B<bindDirect>

 $ivyobj->bindDirect($regexp, $id, [\&callback, @cb_parameters]);
 Ivy::bindDirect($id, [\&callback, @cb_parameters]);

The callback will be called with both the @msgs and the @cb_parameters.

 Example :
   $ivyobject->bindDirectMessage("id1", [\&callback, @cb_parameters]);

   # Your callback and method must be like:
   sub cb {
     my (@cb_parameters, @msgs) = @_;
     ...
   }

   sub method {
     my ($self, @cb_parameters, @msgs) = @_;
     ...
   }

=item B<sendDieTo>

 $ivyobj->sendDieTo($to);
 Ivy::sendDieTo($to);

Send a suicide to the application named $to.

=item B<ping>

 $ivyobj->ping($to, $timeout);
 Ivy::ping($to, \&callBack);

Send a ping message, callBack will be called on response.

=item B<after>

 $after_id = $ivyobj->after($timeAfter, \@callbacks_list);
 $after_id = Ivy::after($timeAfter, \@callbacks_list);

Call a list of callbacks after $timeAfter milliseconds. To be used only in conjonction with the 'LOCAL' Mainloop (see the Init method). When using the TK mainloop, you must use the Tk::after method.

=item B<repeat>

 $repeat_id = $ivyobj->repeat($timeAfter, \@callbacks_list);
 $repeat_id = Ivy:repeat($timeAfter, \@callbacks_list);

Have a list of callbacks repeatedly called every $timeAfter milliseconds. To be used only in conjonction with the 'LOCAL' Mainloop (see the Init method). When using the TK mainloop, you must use the Tk::repeat method.

=item B<afterCancel>

 $ivyobj->afterCancel($after_or_repeat_id);
 Ivy::afterCancel($after_or_repeat_id);

Cancel an after callback call. To be used only in conjonction with the 'LOCAL' Mainloop (see the Init method). When using the TK mainloop, you must use the Tk::afterCancel method.

=item B<afterResetTimer>

 $ivyobj->afterResetTimer($after_id);
 Ivy::afterResetTimer($after_id);

Reset a timer if this timer has not yet been triggered. To be used only in conjonction with the 'LOCAL' Mainloop (see the Init method).

=item B<fileEvent>

 $ivyobj->fileEvent($fd, $cb);
 Ivy::fileEvent($fd, $cb);

Add a fileEvent handler (or remove any handler associated to $fd if $cb paramter is omitted).
The callback $cb will get the filehandle $fd as parameter.  To be used only in conjonction with the 'LOCAL' Mainloop (see the Init method). When using the TK mainloop, you must use the Tk::fileevent method.

=item B<DESTROY>

 $ivyobj->DESTROY;

Destroy the $ivyobj object. No other method should be applied to the reference of this deleted object. This method should not be used directly.

=back

=head1 BUGS

The stop method does not work!

In the statusFunc, an agent is identified by its name which is not garantted as unique

A message to be sent should not contain '\n' char, because the '\n' is the message separator. Ivy.pm will detect and skip such messages.

No other known bugs at this time. If you find one, please report them to the authors.

=head1 SEE ALSO

perl(1), perlre(1), ivyprobe.pl(1)

=head1 AUTHORS
		Alexandre Bustico <alexandre.bustico@cena.fr>
		Stéphane Chatty <chatty@intuilab.com>
		Hervé Damiano <herve.damiano@aviation-civile.gouv.fr>
		Christophe Mertz <mertz@intuilab.com>

=head1 COPYRIGHT

CENA (C) 1997-2006

=head1 HISTORY

=cut
