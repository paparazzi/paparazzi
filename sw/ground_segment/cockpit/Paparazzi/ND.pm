package Paparazzi::ND;
use Subject;
@ISA = ("Subject");

use strict;

use Math::Trig;
use Tk;
use Tk::Zinc;
use Paparazzi::SatPage;
use Paparazzi::EnginePage;
use Paparazzi::APPage;
use Paparazzi::IRPage;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc     => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -selected_ac => [S_NOINIT,  S_METHOD, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -page     => [S_NOINIT,  S_METHOD, S_RDWR, S_OVRWRT, S_NOPRPG, "gps"],
		    -engine_status => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, undef],
		    -ap_status => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, undef],
		    -wind      => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, undef],
		    -lls       => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, undef],
		    -sats      => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, undef],
		    -fix       => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, undef],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit(); 
  $self->build_gui();
  $self->configure('-pubevts' => 'WIND_COMMAND');
}

sub page {
  my ($self, $old_val, $new_val) = @_;
  print "in ND::page [$new_val]\n";
  return unless defined $new_val and defined $self->{sat_view};
#  $self->{sat_view}->configure('-visible' => $i%2);
#  $i++;
}

sub put_lls {
  my ($self, $value) = @_;
#  $self->{IR}->put_lls($value);
}

sub selected_ac {
  my ($self, $previous_ac, $new_ac) = @_;
  $previous_ac->detach($self, '-svsinfo', [\&foo_cbk, '-svsinfo']) if ($previous_ac);
  $new_ac->attach($self, '-svsinfo', [\&foo_cbk, '-svsinfo']);
  
}

sub foo_cbk {
  my ($self, $field, $aircraft, $event, $new_value) = @_;
  $self->configure('-sats', $new_value);
}

sub build_gui() {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  my $origin = $self->get('-origin');

  $self->{main_group} = $zinc->add('group', 1, -visible => 1);
  $zinc->coords($self->{main_group}, $origin);
  $zinc->add('rectangle',  $self->{main_group},
	     [1, 1, $width-2, $height-2],
	     -visible => 1,
	     -filled => 0,
	     -linecolor => 'red');
  my ($margin, $page_width) = (5, 300);
  my $real_width = $page_width - 2*$margin; 
  my ($page_per_row, $row, $col) = (2, 0, 0);

  my $pages = ['Sat', 'Engine','AP', 'IR'];
  foreach my $page (@{$pages}) {
    $self->{$page} = $self->component('Paparazzi::'.$page.'Page',
				      -zinc => $zinc,
				      -parent_grp => $self->{main_group},
				      -origin  => [ $margin+$col*$page_width, $margin+$row*$page_width],
				      -width   => $real_width,
				      -height  => $real_width,
				      -visible => 1,
				     );
    if ($page eq "IR") {
      $self->{$page}->attach($self, 'WIND_COMMAND', [sub { my ($self, $component, $signal, $arg) = @_;
							   $self->notify('WIND_COMMAND', $arg)}]);
    }
    $col++;
    unless ($col lt $page_per_row) { $col=0; $row++ };
  }




#  my $engine_h = { -nb_engine => 2,
#		   -engine    => [{throttle => 50, -rpm => 3500, -temp => 39},
#				  {throttle => 50, -rpm => 3400, -temp => 37}],
#		   -bat       => 11.5,
#		   -energy    => 25.2
#		 };
  
#  my $ap_h = {
#	      -mode => 1,
#              -h_mode => 2,
#              -v_mode => 0,
#              -target_climb => 1.,
#              -target_alt => 200.,
#              -target_heading => 36.,
#	     };
#  $self->{AP}->configure( -ap_status => $ap_h);
}

1;
