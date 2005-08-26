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

sub hhmmss_of_s {
  my ($t) = @_;
  my $hour = int($t/3600);
  my $min = int(($t-$hour*3600)/60);
  my $sec = $t-(3600*$hour)-($min*60);
  sprintf("%02d:%02d:%02d",$hour, $min, $sec);
}

1;
