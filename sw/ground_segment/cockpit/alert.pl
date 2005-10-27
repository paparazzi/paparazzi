#!/usr/bin/perl 

my @paparazzi_lib;
BEGIN {
  @paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
    ($ENV{PAPARAZZI_SRC}."/sw/lib/perl", $ENV{PAPARAZZI_SRC}."/sw/ground_segment/cockpit"):();
}
use lib (@paparazzi_lib);

use Tk;
use Tk::Zinc;

use XML::DOM;
use Data::Dumper;

use strict;
use warnings;

use Paparazzi::Traces;
use Paparazzi::Utils;
use Paparazzi::GuiConfig;
use Paparazzi::Alert;


build_gui();

sub build_gui {
  my $mw = MainWindow->new();
  my $zinc = $mw->Zinc(
    -width	=> 400,
    -height	=> 400,
    -render	=> 1,
  );
  $zinc->pack();

  my $alert = new Paparazzi::Alert(-zinc => $zinc);
  
  Tk->MainLoop();
}
