package Utils;

use Data::Dumper;
use Math::Trig;

sub trim {
  my ($x, $min, $max) = @_;
  return $min if ($x < $min);
  return $max if ($x > $max);
  return $x;
}

sub diff_array {
  my ($a, $b) = @_;
#  print "diff_array [ @{$a} ] - [ @{$b} ] => ";
  my @aonly;
  my %seen;
  @seen{@{$b}} = ();
  foreach my $ac (@{$a}) {
    push(@aonly, $ac) unless exists $seen{$ac};
  }
#  print "[ @aonly ]\n";
  return @aonly;
}

sub rad_of_deg {
  return (shift @_) * Math::Trig::pip2() /90.;
}

sub deg_of_rad {
  return (shift @_) * 90. / Math::Trig::pip2();
}

sub min {
  my ($a, $b) = @_;
  return $a if ($a lt $b);
  return $b;
}

1;
