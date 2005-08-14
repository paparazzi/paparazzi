#!/usr/bin/perl
# quick and dirty test for papastrip::strip
use PapaStrip::Strip;
use Tk;
use Tk::Zinc;
use XML::DOM;

use Ivy;
our $paparazzi_home = $ENV{PAPARAZZI_HOME};
our $options = {
		paparazzi_home => $paparazzi_home,
		ivy_bus  => "127.255.255.255:2010",
		data_dir => $paparazzi_home."/data",
		map_file => "magny_UTM.xml",
		conf_dir => $paparazzi_home."/conf",
		mission_file => "flight_plans/muret1.xml",
	   };

###############################################################################
#
# Ivy
#
###############################################################################
sub start_ivy {
  Ivy->init(
	    -ivyBus        => $options->{ivy_bus},
	    -appName       => "Paparazzi Ground Station",
	    -loopMode      => 'TK',
	    -messWhenReady => 'MESSAGE_WHEN_READY',
	   );
  my $ivy = Ivy->new (-statusFunc => \&ivyStatusCbk);
  $ivy->start();
  return $ivy;
}

sub ivyStatusCbk {
  #  printf("in ivyStatusCbk\n");
}


sub parse_config {
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($options->{conf_dir}."/conf.xml");
  my $files = $doc->getElementsByTagName('files')->[0];
  my $conf_files = { radio => $files->getAttribute('radio'),
		     airframe => $files->getAttribute('airframe'),
		     flight_plan => $files->getAttribute('flight_plan'),
		     ground_segment => $files->getAttribute('ground_segment')
		   };
  ## parsing the ground segment
  $doc = $parser->parsefile($options->{conf_dir}."/".$conf_files->{ground_segment});
  my $network = $doc->getElementsByTagName('network')->[0];
  my $map = $doc->getElementsByTagName('map')->[0];
  $options->{mission_file} = $conf_files->{flight_plan};
  $options->{ivy_bus} = $network->getAttribute('ivy_bus');
  $options->{map_file} = $map->getAttribute('location');
}

###############################################################################
#
# GUI
#
###############################################################################

sub build_gui {
  my $mw = MainWindow->new(-width=>800, -height=>600);
  my $zinc = $mw->Zinc(-width=>800, -height=>600, -render=>1)->pack();
  my $ivy = start_ivy;
  my $flight_plan_name = $options->{conf_dir}."/".$options->{mission_file};
  my $strip_ac0 = new PapaStrip::Strip(-mw => $mw,
			    -zinc=> $zinc,
			    -ident=> "Tartine",
			    -pos_x=>0,
			    -pos_y=>0,
			    -flight_plan=>$flight_plan_name
			   );
  $strip_ac0->draw();
  my $strip_pipo = new PapaStrip::Strip(-mw => $mw,
			    -zinc=> $zinc,
			    -ident=> "Nutella",
			    -pos_x=>0,
			    -pos_y=>100,
			    -flight_plan=>$flight_plan_name);
  $strip_pipo->draw();

  Tk::MainLoop();
}

build_gui;
