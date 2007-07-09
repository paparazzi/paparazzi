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
	    "fg=s" => \$options->{fg},
	    "boot" => \$options->{boot},
	    "norc" => \$options->{norc},
	    "launch" => \$options->{launch},
	   );
my @args = ();
push @args, "-b", $options->{ivy_bus} if defined $options->{ivy_bus};
push @args, "-fg", $options->{fg} if defined $options->{fg};
push @args, "-norc" if defined $options->{norc};
push @args, "-boot" if defined $options->{boot};
push @args, "-launch" if defined $options->{launch};
my $sim_binary = Paparazzi::Environment::paparazzi_home()."/var/".$options->{aircraft}."/sim/simsitl";
die "Error: $sim_binary not found. Build target 'sim' for $options->{aircraft} (make AIRCRAFT=$options->{aircraft} sim)\n" unless -e $sim_binary;
exec ($sim_binary, @args)
