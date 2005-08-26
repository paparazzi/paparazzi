package Paparazzi::SettingsPage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;
use Data::Dumper;

use constant TITLE => "Settings";

my @fields = ('if_mode', 'if_value1', 'if_value2');

sub populate {
  my ($self, $args) = @_;
  $args->{-title} = TITLE;
  $self->SUPER::populate($args);
  $self->configspec(
		    -fields => [S_NEEDINIT, S_PASSIVE,  S_RDONLY,   S_OVRWRT, S_NOPRPG, []],
		   );
}

sub set_aircraft {
  my ($self, $prev_ac, $new_ac) = @_;
  foreach my $field  (@fields) {
    $prev_ac->detach($self, $field, [\&update_field]) if ($prev_ac);
    $new_ac->attach($self, $field, [\&update_field]) if ($new_ac);
  }
  
  my $fligh_plan = $new_ac->get('flight_plan');
  
  use Data::Dumper;
  print "#####".Dumper(scalar $fligh_plan->get('-rc_control'));

}

sub update_field {
  my ($self, $aircraft, $field, $new_value) = @_;
  $self->get('-zinc')->itemconfigure($self->{'text_'.$field}, -text => $new_value);
}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my $dy = $self->get('-height')/10;
  my $y=10;
  my $x=10;
  foreach my $field (@fields) {
    $zinc->add('text', $self->{main_group},
	       -position => [$x, $y+$self->{vmargin}],
	       -color => 'white',
	       -anchor => 'w',
	       -text => $field);
    $self->{'text_'.$field} = $zinc->add('text', $self->{main_group},
					 -position => [$x + 100, $y+$self->{vmargin}],
					 -color => 'white',
					 -anchor => 'w',
					 -text => 'NA');
    $y+=$dy;
  }
}

