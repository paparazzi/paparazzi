#!/usr/bin/perl -w

package PaparazziSpeak;

my @paparazzi_lib;
BEGIN {
  @paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
    ($ENV{PAPARAZZI_SRC}."/sw/lib/perl"):();
}
use lib (@paparazzi_lib);

use strict;
use Paparazzi::Environment;

use constant APP_ID => "Paparazzi Speaker";
use constant MESSAGE_WHEN_READY => APP_ID." : READY";

use strict;

use IO::Socket;
use POSIX;
use Getopt::Long;
use Ivy;

use Paparazzi::IvyProtocol;


sub new() {

  my ($proto, $festd_host, $festd_port) = @_;
  my $self = {
	      'ivy' => undef,
	      'festival_handle' => undef,
	      'vbat' => 0,
	      'cur_wp' => -1,
	      'cnt_nav' => 0,
	     };
  $self->{options} = {
#		      paparazzi_home => $paparazzi_home,
		      ivy_bus  => "127.255.255.255:2010",
		     };

  bless $self;
  $self->parse_args();
  $self->start_ivy();
  $self->connect_to_festival($festd_host, $festd_port);
  $self->say_hello();

  # Trap signal in order to exit cleanly
  $SIG{TERM} = \&catchSigTerm ;

  return $self;
}

sub parse_args {
  my ($self) = @_;
  my $options = $self->{options};
  GetOptions ("b=s" => \$options->{ivy_bus},
	      "t=s" => \$options->{paparazzi_home},
	     );
}

sub start_ivy() {
  my ($self) = @_;
  Ivy->init (-ivyBus        => $self->{options}->{ivy_bus},
	     -appName       => APP_ID,
	     -loopMode      => 'LOCAL',
	     -messWhenReady => MESSAGE_WHEN_READY,
	    ) ;
  my $paparazzi_home = Paparazzi::Environment::paparazzi_home();
  Paparazzi::IvyProtocol::read_protocol($paparazzi_home."/conf/messages.xml", "ground");
  Paparazzi::IvyProtocol::read_protocol($paparazzi_home."/conf/messages.xml", "aircraft_info");
  
  $self->{ivy} = Ivy->new (-statusFunc => \&ivyStatusCbk) ;
#  $self->{ivy}->bindRegexp (IvyMsgs::CALIB_START_Regexp(), [$self, \&ivyOnCalibStart]);
#  $self->{ivy}->bindRegexp (IvyMsgs::CALIB_CONTRAST_Regexp(), [$self, \&ivyOnCalibContrast]);
#  $self->{ivy}->bindRegexp (IvyMsgs::NAVIGATION_Regexp(), [$self, \&ivyOnNavigation]);
#  $self->{ivy}->bindRegexp (IvyMsgs::BAT_Regexp(), [$self, \&ivyOnBat]);
#  $self->{ivy}->bindRegexp (IvyMsgs::PPRZ_MODE_Regexp(), [$self, \&ivyOnPprzMode]);
#  $self->{ivy}->bindRegexp (IvyMsgs::TAKEOFF_Regexp(), [$self, \&ivyOnTakeOff]);
  $self->{ivy}->start() ;
}

sub catchSigTerm() {
  print ("in catchSigTerm\n");

}

sub ivyStatusCbk {
  print ("in ivyStatusCbk\n");
}

sub say_hello() {
  my ($self) = @_;
  $self->speak('<EMP>Hello.</EMP> Welcome to <PRON SUB=\"Paparraddzee\">Paparazzi.</PRON>');
}

sub ivyOnNavigation() {
  my ($self, $sender, $cur_wp, $pos_x, $pos_y, $desired_course, $dist2_wp, $course_pgain) = @_;
  #  printf("NAVIGATION wp $cur_wp, x $pos_x, y $pos_y, dc $desired_course, d2wp $dist2_wp, cpg $course_pgain\n");

  if ($self->{cur_wp} != $cur_wp) {
    $self->speak(sprintf("current waypoint : <EMP>%s.</EMP>", $cur_wp));
    $self->{cur_wp} = $cur_wp;
    $self->{cnt_nav} = 0;
  }
  else {
    my $rdist = floor(sqrt($dist2_wp)/10)*10;
    printf "dist2wp $rdist\n";
    if (($rdist ge 100 and $self->{cnt_nav} == 16) or
        ($rdist ge 20 and $rdist le 100 and ($self->{cnt_nav})%5 == 0)) {
      $self->speak(sprintf("distance to waypoint : <EMP>%s.</EMP> meters", $rdist));
    }
  }
  $self->{cnt_nav}++;
}

sub ivyOnBat() {
  my ($self, $sender, $voltage, $flight_time, $low_battery) = @_;
  my $vbat = $voltage/10;

  if ($voltage le $low_battery) {
    if ($self->{vbat} != $vbat) {
      $self->speak(sprintf("battery : <EMP> Warning : battery low : %s volts.</EMP>", $vbat));
      $self->{vbat} = $vbat;
    }
  }
  else {
    if (abs($self->{vbat} - $vbat) ge 0.2) {
      $self->speak(sprintf("battery : <EMP>%s</EMP> volts.", $vbat));
      $self->{vbat} = $vbat;
    }
  }
}

sub ivyOnPprzMode() {
  my @autopilot_mode_name=("manual", "auto one", "auto two", "home");
  my ($self, $sender, $ap_mode, $ap_altitude, $if_calib_mode, $mcu1_status, $lls_calib) = @_;
  if ($self->{ap_mode} != $ap_mode) {
    my $ap_str = $autopilot_mode_name[$ap_mode];
    $self->speak(sprintf("autopilot mode : <EMP>%s.</EMP>", $ap_str));
    $self->{ap_mode} = $ap_mode;
  }
}

sub ivyOnCalibStart() {
  my ($self, $sender) = @_;
  $self->speak("<EMP>contrast</EMP> calibration triggered");
}

sub ivyOnCalibContrast() {
  my ($self, $sender, $adc) = @_;
  my $pc_contrast = ceil($adc / 1024 * 100);
  my $txt = sprintf("contrast <EMP>%s</EMP>per cent", $pc_contrast);
  print "txt $txt\n";
  $self->speak($txt);
}

sub ivyOnTakeOff() {
  my ($self, $sender) = @_;
  $self->speak('<VOLUME LEVEL=\"loud\"><EMP>Take Off</EMP></VOLUME>');
}

sub speak() {
  my ($self, $what) = @_;
  my $handle = $self->{festival_handle};
  my $sable_fmt =
    '<!DOCTYPE SABLE PUBLIC \"-//SABLE//DTD SABLE speech mark up//EN\" \"Sable.v0_2.dtd\" []>
   <SABLE>
     <SPEAKER NAME=\"male1\">

     %s

     </SPEAKER>
   </SABLE>';

  my $cmd_fmt = sprintf("(tts_text \"%s\" \'sable)\n", $sable_fmt);
  my $fest_cmd =  sprintf $cmd_fmt, $what;
  print $handle $fest_cmd;
}

sub connect_to_festival() {
  my ($self, $host, $port) = @_;
  $self->{festival_handle} = IO::Socket::INET->new(Proto     => "tcp",
						   PeerAddr  => $host,
						   PeerPort  => $port);
  $self->{festival_handle}->autoflush(1);
  Ivy->fileEvent($self->{festival_handle}, [\&FestivalOnReceive, $self]);
  print STDERR "[Connected to $host:$port]\n";
}

sub FestivalOnReceive {
  my ($self) = @_;
  my $file_stuff_key = "ft_StUfF_key";    # defined in speech tools
  print "FestivalOnReceive\n";
  my $handle = $self->{festival_handle};
  my $line =  <$handle>;
  
  #  print "line: [$line]\n";
#  if ($line eq "WV\n") { # we have a waveform coming
#    print "Waveform\n";  
#  }
  
#  if ($line eq "LP\n") { # we have a waveform coming
#    print "Lisp\n";  
#  }
#  if ($line =~ s/$file_stuff_key(.*)$//s) {
#    print STDOUT $line;
#  }
}



PaparazziSpeak->new("localhost", 1314);
Ivy->mainLoop();

