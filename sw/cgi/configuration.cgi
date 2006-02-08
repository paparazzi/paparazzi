#!/usr/bin/perl

# displays the current configuration

use strict;
use warnings;

use Cwd;
use CGI ':standard', '*table';

my $paparazzi_src;
my $paparazzi_lib;
BEGIN {
  $paparazzi_src = getcwd()."/../..";
  $paparazzi_lib = $paparazzi_src."/sw/lib/perl";
}
use lib ($paparazzi_lib);
my $paparazzi_home = $paparazzi_src;

use Paparazzi::Environment;
Paparazzi::Environment::set_env($paparazzi_src, $paparazzi_home);

use Paparazzi::Configuration;

print
  header(),
  start_html("Paparazzi configuration");

my $configuration = Paparazzi::Configuration::read_current();
print
  h1 ("Paparazzi Configuration"),
  h2 ("Aircrafts");
print
  start_table({border => undef}),
  Tr(th(["Id", "Name", "Airframe", "Radio", "Flight plan"]));
foreach my $ac (@{$configuration->{aircrafts}}) {
  print
    Tr(td([$ac->{ac_id}, $ac->{name}, 
	   a({href=>"../conf/$ac->{airframe}"}, foo($ac->{airframe}) ),
	   a({href=>"../conf/$ac->{radio}"}, foo($ac->{radio}) ),
	   a({href=>"../conf/$ac->{flight_plan}"}, foo($ac->{flight_plan}) )]));
}
print end_table();

print
  h2 ("Ground"),
  ul(li (["name : ".$configuration->{ground}->{name},
       "ivy bus : ".$configuration->{ground}->{ivy_bus}
      ])),
  hr(),
  a({href=>"../index.html"}, "home"),
  end_html();


sub foo {
  return ($_[0] =~ /([^\/]*)$/ );
}
