package Paparazzi::Traces;
@ISA = qw (Exporter);
@EXPORT = qw (trace TRACE_ERROR TRACE_DEBUG TRACE_JUNK);

use strict;

use constant TRACE_ERROR => 1;
use constant TRACE_DEBUG => 2;
use constant TRACE_JUNK  => 3;

my $tracelevel = 1;

sub init {
  my ($level) = @_;
  $tracelevel = $level;
}

sub trace {
  my ($level, $msg) = @_;
  print $msg."\n" if $level <= $tracelevel;
}


1;
