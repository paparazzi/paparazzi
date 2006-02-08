package Paparazzi::Configuration;

use strict;
use XML::Parser;
use XML::DOM;

use Paparazzi::Environment;


sub read_current {
  my $filename =  Paparazzi::Environment::paparazzi_home()."/conf/conf.xml";
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  my $conf = $doc->getElementsByTagName('conf')->[0];
  return parse($conf);
}

sub parse {
  my ($conf) = @_;
  my @ret = ();
  my @aircrafts = $conf->getElementsByTagName('aircraft');
  foreach my $aircraft (@aircrafts) {
    push @ret, parse_aircraft($aircraft);
  }
  my $xml_ground = $conf->getElementsByTagName('ground')->[0];
  my $ground = parse_ground($xml_ground);
  return { ground => $ground, aircrafts => \@ret};
}

sub parse_aircraft {
  my ($aircraft) = @_;
  return {
	  ac_id => $aircraft->getAttribute('ac_id'),
	  name => $aircraft->getAttribute('name'),
	  airframe => $aircraft->getAttribute('airframe'),
	  radio => $aircraft->getAttribute('radio'),
	  flight_plan => $aircraft->getAttribute('flight_plan'),
	 };
}

sub parse_ground {
  my ($ground) = @_;
  return {
	  name => $ground->getAttribute('name'),
	  ivy_bus => $ground->getAttribute('ivy_bus')
	 };
}

1;
