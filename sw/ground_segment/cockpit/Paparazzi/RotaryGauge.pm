package Paparazzi::RotaryGauge;
use Subject;
@ISA = ("Subject");
use strict;


use Tk; 
use Tk::Zinc;
use Math::Trig;
use Data::Dumper;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  my ($min_val, $max_val, $tick_spacing, $legend_spacing, $min_val_angle, $dead_sector) = (0, 100, 10, 20, 0, 45);
  $self->configspec(-zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -parent_grp  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -radius  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -min_val        => [S_NOINIT,   S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, $min_val],
		    -max_val        => [S_NOINIT,   S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, $max_val],
		    -tick_spacing   => [S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, $tick_spacing],
		    -legend_spacing => [S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, $legend_spacing],
		    -min_val_angle  => [S_NOINIT,   S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, $min_val_angle],
		    -dead_sector    => [S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, $dead_sector],
		    -text           => [S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, ""],
		    -format         => [S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, "%.1f"],
		    -value   => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->{ang_by_val} =  Utils::rad_of_deg((360. - $self->get('-dead_sector')) / 
					   ( $self->get('-max_val') - $self->get('-min_val')));
  $self->build_gui();
}

sub angle_of_value {
  my ($self, $value) = @_;
  my $min_val_angle = $self->get('-min_val_angle');
  my $aov = Utils::rad_of_deg($min_val_angle) + $self->{ang_by_val} * ($value - $self->get('-min_val'));
#  print "angle_of_value $value -> $aov\n";
  return $aov;
}

sub value {
  my ($self, $old_val, $new_val) = @_;
  return unless defined $new_val and defined $self->{ang_by_val};
  my $zinc = $self->get('-zinc');
  my $angle = $self->angle_of_value($new_val);
  $zinc->treset($self->{rotate_group});
  $zinc->rotate($self->{rotate_group}, $angle + 1.57, 0, 0);
  return unless defined $self->{value_label} and defined $self->get('-format');
  my $name = $self->get('-text');
  $zinc->itemconfigure($self->{value_label}, -text => sprintf($self->get('-format'), $new_val));
}

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $origin = $self->get('-origin');
  my $parent_grp =  $self->get('-parent_grp');
  my $radius = $self->get('-radius');
  $self->{main_group} = $zinc->add('group', $parent_grp, -visible => 1);
  $zinc->coords($self->{main_group}, [$origin->[0]+$radius, $origin->[1]+$radius] );
  my $rad = 0.7*$radius;
  $zinc->add('rectangle',  $self->{main_group}, [-1.15*$radius, -1.15*$radius, 1.15*$radius, 1.15*$radius+25],
	     -visible => 1, 
	     -linecolor => 'white',
	    );
  $zinc->add('arc',  $self->{main_group},
	     [-$rad, -$rad, $rad, $rad],
	     -visible => 1,
	     -linecolor => 'white',
	     -filled => 0,
	    );
  $self->{rotate_group} =  $zinc->add('group', $self->{main_group}, -visible => 1);
  my $min_val_angle = $self->angle_of_value(scalar $self->get('-min_val'));
  my ($p1x, $p1y) = ($rad*cos($min_val_angle), $rad*sin($min_val_angle));
  $zinc->add('curve', $self->{rotate_group},
	     [0.1*$p1x, 0.1*$p1y, $p1x, $p1y],
	     -linewidth => 2,
	     -linecolor => 'white',
	     -filled => 0);

  # ticks
  for (my $val = $self->get('-min_val'); $val < $self->get('-max_val'); $val+= $self->get('-tick_spacing')) {
    my $angle = $self->angle_of_value($val);
#    print "tick angle_of_value $val -> $angle\n" if ($self->get('-text') eq "bat");
    my ($px, $py) = ($radius * cos($angle), $radius * sin($angle));
    $zinc->add('curve', $self->{main_group},
	       [0.7*$px, 0.7*$py, 0.8*$px, 0.8*$py],
	       -linewidth => 2,
	       -linecolor => 'white',
	      );
  }
  # legend
  for (my $val = $self->get('-min_val'); $val < $self->get('-max_val'); $val+= $self->get('-legend_spacing')) {
    my $angle = $self->angle_of_value($val);
    $self->{label} = $zinc->add('text', $self->{main_group},
				-position => [$radius * cos($angle), $radius * sin($angle)],
				-text => sprintf("%.0f", $val),
				-color => 'white',
				-anchor => 'c',
			       );
  }
  # title
  my $text = $self->get('-text');
  $self->{text_label} = $zinc->add('text', $self->{main_group},
				   -position => [0, $radius+8],
				   -text => $text,
				   -color => 'white',
				   -anchor => 'c',
				  );
  # value  
  $self->{value_label} = $zinc->add('text', $self->{main_group},
				    -position => [0, $radius + 20],
				    -text => "",
				    -color => 'white',
				    -anchor => 'c',
				   );
}
