package Paparazzi::IRPage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;
use DigiKit::Button;

use Paparazzi::HistoryView;


use constant TITLE => "Infrared";
use constant UPDATE_REPEAT => 2000;

sub populate {
  my ($self, $args) = @_;
  $args->{-title} = TITLE;
  $self->SUPER::populate($args);
  $self->configspec(
		    -wind => [S_NOINIT, S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		    -lls => [S_NOINIT, S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.0015],
		   );
}

sub completeinit {
  my $self = shift;
  $self->{wind_running} = 0;
  $self->SUPER::completeinit();
#  $self->build_gui();
  $self->configure('-pubevts' => 'WIND_COMMAND');
  $self->{timer_id} = $self->get('-zinc')->repeat(UPDATE_REPEAT, [\&onTimer, $self]);
}

sub onTimer {
  my ( $self) = @_;
  $self->{history}->put_value(scalar $self->get('-lls'));
}

sub wind {
  my ($self, $old_val, $new_val) = @_;
  foreach my $field (keys %{$new_val}) {
    $self->get('-zinc')->itemconfigure ($self->{'text_'.$field},
					-text => sprintf("%s : %.4f", $field, $new_val->{$field})) if defined $self->{'text_'.$field};
  }
}

sub lls {
  my ($self, $old_val, $new_val) = @_;
  return unless defined $new_val and defined $self->{history};
#  $self->{history}->put_value($new_val);
  $self->get('-zinc')->itemconfigure ($self->{'text_auto gain'},
				      -text => sprintf("auto gain : %.5f", $new_val));
}

#sub put_lls {
#  my ($self, $value) = @_;
##  $self->{history}->put_value($value);
#}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my $parent_grp = $self->get('-parent_grp');
  my $main_group = $self->{main_group};
  
  my $fields = ["contrast", "gain","auto gain"];
  my ($y, $dy) = ( 35, 20);
  foreach my $field (@{$fields}) {
    $self->{'text_'.$field} = $zinc->add('text', $main_group,
					 -position => [20, $y],
					 -color => 'white',
					 -anchor => 'w',
					 -text => "$field");
    $y+=$dy;
  }

  my $nb_bars = 125;
  $self->{history} = 
    Paparazzi::HistoryView->new(-zinc => $zinc,
				-width => 250,
				-height => 50,
				-origin => [20, 90],
				-parent_grp => $self->{main_group},
				-nb_bars => $nb_bars,
				-initial_range => 0.003,
			       );
  $zinc->add('text', $main_group,
	     -position => [20, 145],
	     -color => 'white',
	     -anchor => 'w',
	     -text => sprintf("%d seconds", UPDATE_REPEAT * $nb_bars / 1000),
	    );

  $zinc->add('text', $main_group,
	     -position => [10, 170],
	     -color => 'white',
	     -anchor => 'w',
	     -text => "Wind");
  
  $fields = ['dir', 'speed','mean_aspeed', 'stddev'];
  ($y, $dy) = ( 190, 20);
  foreach my $field (@{$fields}) {
    $self->{'text_'.$field} = $zinc->add('text', $main_group,
					 -position => [20, $y],
					 -color => 'white',
					 -anchor => 'w',
					 -text => "$field");
    $y+=$dy;
  }
  
  $self->{button_clear_wind} = DigiKit::Button->new(-widget => $zinc,
						     -parentgroup => $main_group,
						     -style => ['Aqualike',
								-width => 60,
								-height => 20,
								-color => 'green',
								-text => 'clear',
								-trunc => 'right',
							       ],
						     -position => [10, 262],
						    );
  $self->{button_toggle_wind} = DigiKit::Button->new(-widget => $zinc,
						     -parentgroup => $main_group,
						     -style => ['Aqualike',
								-width => 60,
								-height => 20,
								-color => 'green',
								-text => 'start',
								-trunc => 'left',
							       ],
						     -position => [70, 262],
						    );
  $self->{button_clear_wind}->configure(-releasecommand => sub {
					  $self->notify('WIND_COMMAND', 'clear');
					}
				       );
  $self->{button_toggle_wind}->configure(-releasecommand => sub {
					   if ($self->{wind_running}) {
					     $self->{button_toggle_wind}->value('start');
					     $self->notify('WIND_COMMAND', 'stop');
					     $self->{wind_running} = 0;
					   }
					   else {
					     $self->{button_toggle_wind}->value('stop');
					     $self->notify('WIND_COMMAND', 'start');
					     $self->{wind_running} = 1;
					   }
					 }
					);

}

