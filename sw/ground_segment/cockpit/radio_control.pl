#!/usr/bin/perl -w
package RadioControl;

my @paparazzi_lib;
BEGIN {
  @paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
    ($ENV{PAPARAZZI_SRC}."/sw/lib/perl", $ENV{PAPARAZZI_SRC}."/sw/ground_segment/cockpit"):();
}
use lib (@paparazzi_lib);

use strict;
use Paparazzi::Environment;
use Paparazzi::RCTransmitter;
use Paparazzi::IvyProtocol;


use Getopt::Long;
use Tk;
use Ivy;

use constant APP_ID => "Paparazzi RadioControl";
use constant MESSAGE_WHEN_READY => APP_ID.': READY';

my $options = {
	       radio_file => "fc28.xml",
	       ivy_bus  => "127.255.255.255:2010",
	      };

GetOptions (
	    "r=s" => \$options->{radio_file},
	   );


my $mw = MainWindow->new();
Paparazzi::RCTransmitter->new(
		   $mw,
		   -filename => Paparazzi::Environment::paparazzi_home()."/conf/radios/".$options->{radio_file}
		  )->pack();
start_ivy();
MainLoop();

my $self = {};

sub start_ivy {
#  my ($self) = @_;

  Ivy->init (-ivyBus        => $options->{ivy_bus},
	     -appName       => APP_ID,
	     -loopMode      => 'TK',
	     -messWhenReady => MESSAGE_WHEN_READY,
	    ) ;
  $self->{ivy} = Ivy->new (-statusFunc => \&ivyStatusCbk);
  $self->{ivy}->start();
  my $paparazzi_home = Paparazzi::Environment::paparazzi_home();
  Paparazzi::IvyProtocol::read_protocol($paparazzi_home."/conf/messages.xml", "telemetry_fbw");
  Paparazzi::IvyProtocol::bind_message("telemetry_fbw", "RC", {}, $self->{ivy}, [$self, \&ivyOnRc]);
}

sub ivyStatusCbk {

}

sub ivyOnRc {
#  my ($self) = @_;



}
