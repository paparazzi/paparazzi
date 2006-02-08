#!/usr/bin/perl

# displays the current configuration

use CGI ':standard';

my $paparazzi_lib;
BEGIN {
#  $paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
#    $ENV{PAPARAZZI_SRC}."/sw/lib/perl" : "/usr/lib/paparazzi/";
  $paparazzi_lib = "/home/poine/work/paparazzi_savannah/paparazzi3/sw/lib/perl";
}
use lib ($paparazzi_lib);

use strict;
use warnings;

use Paparazzi::Environment;
Paparazzi::Environment::set_env("/home/poine/work/paparazzi_savannah/paparazzi3",
			       "/home/poine/work/paparazzi_savannah/paparazzi3");

use Paparazzi::Configuration;
print CGI::header;
CGI::start_html("Paparazzi configuration");

my $configuration = Paparazzi::Configuration::read_current();

print "\n<H1> Paparazzi Configuration.</H1>\n";
print "<H2> Aircrafts.</H2>\n";
print "<TABLE CELLSPACING=4>\n";
print_table_header(["Id", "Name", "Airframe", "Radio", "Flight plan"]);

foreach my $ac (@{$configuration->{aircrafts}}) {
  print "  <TR>\n";
  print "    <TD>";
  print "$ac->{ac_id}";
  print "</TD>\n";
  print "    <TD>";
  print "$ac->{name}";
  print "</TD>\n";
  print "    <TD>";
  print_link("../conf/".$ac->{airframe}, foo($ac->{airframe}));
  print "</TD>\n";
  print "    <TD>";
  print_link("../conf/".$ac->{radio}, foo($ac->{radio}));
  print "</TD>\n";
  print "    <TD>";
  print_link("../conf/".$ac->{flight_plan}, foo($ac->{flight_plan}));
  print "</TD>\n";
  print "  </TR>\n";
}
print "</TABLE>\n";
print "<H2> Ground.</H2>\n";
print "<li>name : ".$configuration->{ground}->{name}."</li>\n";
print "<li>ivy bus : ".$configuration->{ground}->{ivy_bus}."</li>\n";
print "<HR>\n";
print "<a href=\"../index.html\">home</a><BR>\n";
print "<HR>\n";
print "<ADDRESS>Poine.</ADDRESS><BR>\n";

sub print_table_header {
  my ($headers) = @_;
  print "  <TR>\n";
  foreach (@{$headers}) {
    print "    <TH>$_</TH>\n";
  }
  print "  </TR>\n";
}

sub foo {
  return ($_[0] =~ /([^\/]*)$/ );
}

sub print_link {
  my ($link, $text) = @_;
  print "<a href=\"$link\">$text</a>";
}
