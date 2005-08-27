package Paparazzi::AutopilotPage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;
use Data::Dumper;

use Paparazzi::Utils;

use constant TITLE => "Autopilot";

my @fields = ('ap_mode', 'lat_mode', 'horiz_mode', 'gaz_mode', 'gps_mode', 'flight_time');

sub populate {
  my ($self, $args) = @_;
  $args->{-title} = TITLE;
  $self->SUPER::populate($args);
}

sub set_aircraft {
  my ($self, $prev_ac, $new_ac) = @_;
  foreach my $field  (@fields) {
    $prev_ac->detach($self, $field, [\&update_field]) if ($prev_ac);
    $new_ac->attach($self, $field, [\&update_field]) if ($new_ac);
  }
}

sub update_field {
  my ($self, $aircraft, $field, $new_value) = @_;
  my $text = $new_value;
  $text = Utils::hhmmss_of_s($text) if ($field eq 'flight_time');
    $self->get('-zinc')->itemconfigure($self->{'text_'.$field}, -text => $text);
}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my ($y, $dy) = (10, $self->get('-height')/10);
  my ($x, $dx) = (10, 100);
  foreach my $field (@fields) {
    $zinc->add('text', $self->{main_group},
	       -position => [$x, $y+$self->{vmargin}],
	       -color => 'white',
	       -anchor => 'w',
	       -font => $self->{big_font},
	       -text => $field);
    $self->{'text_'.$field} = $zinc->add('text', $self->{main_group},
					 -position => [$x + 100, $y+$self->{vmargin}],
					 -color => 'white',
					 -anchor => 'w',
					 -text => 'NA');
    $y+=$dy;
  }
}

