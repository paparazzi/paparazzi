package Paparazzi::Geometry;

use Math::Trig;

use constant PI_TWO => (pi / 2);
use constant TWO_PI => (2 * pi);

sub angle_of_heading_rad {
  my ($angle)=@_;
  return norm_angle_rad( 5 * PI_TWO - $angle );
}

sub heading_of_angle_rad {

}


sub norm_heading_rad {
  my ($val) = @_;
  while ($val > TWO_PI ) {$val -= TWO_PI}
  while ($val < 0) {$val += TWO_PI }
  return $val;
}

sub norm_angle_rad {
  my ($val) = @_;
  while ($val >   pi)  {$val -= TWO_PI}
  while ($val < - pi) {$val += TWO_PI}
  return $val;
}


sub cart_of_polar {
  my ($r, $theta) = @_;
  return ($r * cos $theta, $r * sin $theta);
}

sub polar_of_cart {
  my ($x, $y) = @_;
  return (sqrt($x*$x+$y*$y), atan2($y, $x));
}


1;
