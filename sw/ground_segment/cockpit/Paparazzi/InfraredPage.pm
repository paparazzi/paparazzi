package Paparazzi::InfraredPage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;
use DigiKit::Button;

use Paparazzi::HistoryView;


use constant TITLE => "Infrared";
use constant UPDATE_REPEAT => 2000;
my @fields = ('contrast_status', 'contrast_value', 'gps_hybrid_mode', 'gps_hybrid_factor');

sub populate {
  my ($self, $args) = @_;
  $args->{-title} = TITLE;
  $self->SUPER::populate($args);
  $self->configspec(
		    -lls => [S_NOINIT, S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.0015],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->{timer_id} = $self->get('-zinc')->repeat(UPDATE_REPEAT, [\&onTimer, $self]);
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
  $self->get('-zinc')->itemconfigure($self->{'value_'.$field}, -text => $text);
}

sub onTimer {
  my ( $self) = @_;
  $self->{history}->put_value(scalar $self->get('-lls'));
}

sub lls {
  my ($self, $old_val, $new_val) = @_;
  return unless defined $new_val and defined $self->{history};
  $self->get('-zinc')->itemconfigure ($self->{'text_auto gain'},
				      -text => sprintf("auto gain : %.5f", $new_val));
}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my $parent_grp = $self->get('-parent_grp');
  my $main_group = $self->{main_group};
  
  my ($y, $dy) = ( 35, 20);
  my ($x, $dx) = (10, 150);
  foreach my $field (@fields) {
    $self->{'label_'.$field} = $zinc->add('text', $main_group,
					  -position => [$x, $y],
					  -color => 'white',
					  -anchor => 'w',
					  -text => "$field");
    $self->{'value_'.$field} = $zinc->add('text', $main_group,
					  -position => [$x+$dx, $y],
					  -color => 'white',
					  -anchor => 'w',
					  -text => "N/A");
    $y+=$dy;
  }

  my $nb_bars = 125;
  $dy = 50;
  $self->{history} = 
    Paparazzi::HistoryView->new(-zinc => $zinc,
				-width => 250,
				-height => $dy,
				-origin => [20, $y],
				-parent_grp => $self->{main_group},
				-nb_bars => $nb_bars,
				-initial_range => 0.003,
			       );
  $y += $dy+10;
  $zinc->add('text', $main_group,
	     -position => [20, $y ],
	     -color => 'white',
	     -anchor => 'w',
	     -text => sprintf("%d seconds", UPDATE_REPEAT * $nb_bars / 1000),
	     -font => $self->{small_font},
	    );
}

