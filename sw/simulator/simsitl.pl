#!/usr/bin/perl -w

use Getopt::Long;
my @paparazzi_lib;
BEGIN {
  @paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
    ($ENV{PAPARAZZI_SRC}."/sw/lib/perl", $ENV{PAPARAZZI_SRC}."/sw/ground_segment/cockpit"):();
}
use lib (@paparazzi_lib);

use strict;
use Paparazzi::Environment;

my $options = {};
GetOptions (
	    "b=s" => \$options->{ivy_bus},
	    "a=s" => \$options->{aircraft},
	   );
my @args = ();
push @args, "-b", $options->{ivy_bus};
my $sim_binary = Paparazzi::Environment::paparazzi_home()."/var/".$options->{aircraft}."/sim/simsitl.out";
die "$sim_binary not found. try make AIRCRAFT=$options->{aircraft} ac\n" unless -e $sim_binary;
exec ($sim_binary, @args)








