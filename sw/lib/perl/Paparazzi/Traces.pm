package Paparazzi::Traces;

@export = qw (TRACE_ERROR TRACE_DEBUG TRACE_JUNK);

use strict;

use constant TRACE_ERROR => 1;
use constant TRACE_DEBUG => 2;
use constant TRACE_JUNK  => 3;

my $tracelevel = 2;

sub trace {
  my ($level, $msg) = @_;
  print $msg if $level >= $tracelevel;
}


1;
