#!/usr/bin/perl -w

my $paparazzi_home;
BEGIN {
  $paparazzi_home = "/home/drouin/savannah/paparazzi2";
  $paparazzi_home = $ENV{PAPARAZZI_HOME} if defined $ENV{PAPARAZZI_HOME};
};
use lib ($paparazzi_home.'/sw/lib/perl', $paparazzi_home.'/sw/ground_segment/cockpit');

use Getopt::Long;
use Tk;
use Ivy;

use strict;

use MapView_Ivy;

my $options = {
	       paparazzi_home => $paparazzi_home,
	       ivy_bus  => "127.255.255.255:2010",
	       data_dir => $paparazzi_home."/data",
	       map_file => "defaultUTM.xml",
	       conf_dir => $paparazzi_home."/conf",
	       mission_file => "default.xml",
	      };

GetOptions ("b=s" => \$options->{ivy_bus},
	    "t=s" => \$options->{paparazzi_home},
	    "d=s" => \$options->{data_dir},
	    "m=s" => \$options->{map_file},
	    "c=s" => \$options->{conf_dir},
	    "f=s" => \$options->{mission_file},
	   );

use Data::Dumper;
print Dumper($options);

use constant APP_ID => "Paparazzi Map";
use constant MESSAGE_WHEN_READY => APP_ID." : READY";

Ivy->init (-ivyBus        => $options->{ivy_bus},
	   -appName       => APP_ID,
	   -loopMode      => 'TK',
	   -messWhenReady => MESSAGE_WHEN_READY,
	  );
my $ivy = Ivy->new();

my $mw = MainWindow->new();
$mw->title("Paparazzi map : $options->{map_file}");
my $win_size = [770, 600];
my $win_pos = [310, 140];
$mw->geometry(sprintf("%dx%d+%d+%d", $win_size->[0], $win_size->[1], $win_pos->[0], $win_pos->[1]));
my $mv = $mw->MapView_Ivy(-ivy => $ivy);
$mv->pack(-fill => 'both', -expand => "1");

$mv->default_palette();
$mv->load_user_palette($options->{conf_dir}."/ground_segment.xml");
$mv->default_configuration();
$mv->load_configuration($options->{conf_dir}."/ground_segment.xml");
$mv->load_map($options->{data_dir}."/maps/".$options->{map_file}, $win_size);
my $flight_plan_name = $options->{conf_dir}."/".$options->{mission_file};
my $flight_plan = `$paparazzi_home/sw/tools/gen_flight_plan.out -dump $ flight_plan_name`;
$mv->load_flight_plan($flight_plan, $win_size);
$mv->create_circle();
$mv->create_segment();

$ivy->start();

MainLoop();

