#=============================================================================
#                 Scale  Class
#=============================================================================
package Paparazzi::Scale;
use Subject;
@ISA = ("Subject");
use strict;

use Tk; 
use Tk::Zinc;
use Math::Trig;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -parent_grp  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -direction => [S_NOINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 1.],
		    -periodic => [S_NOINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 0],
		    -min_val => [S_NOINIT,   S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 0.],
		    -max_val => [S_NOINIT,   S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 100.],
		    -disp_tick =>[S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 10.],
		    -tick_scale =>[S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 1.],
		    -repeat_legend =>[S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 2.],
		    -fig_clm_pc =>[S_NOINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 0.7],
		    -value   => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
		    -target_value => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
  );
  $self->{value_y} = 0;
  $self->{y_per_tick} = 1.;
}

sub completeinit {
#  print "in Scale::completeinit\n";
  my $self = shift;
  $self->SUPER::completeinit;
  $self->build_gui();
}

sub value() {
  my ($self, $previous_value, $new_value) = @_;
  my $zinc = $self->get('-zinc');

  my $nb_ticks = ($new_value - $self->get('-min_val')) / $self->get('-tick_scale');
#  print "nb_ticks $nb_ticks\n";
  my $new_y = $nb_ticks * $self->{y_per_tick};
  $new_y = -$new_y   if ( $self->get('-direction') < 0);

#  my $new_y = ($new_value / $self->get('-tick_scale')) * $self->{y_per_tick};
  $self->{value_y} = $new_y;
  $zinc->treset($self->{moving_group});
  $zinc->translate($self->{moving_group}, 0, $new_y);
}

sub target_value() {
  my ($self, $previous_value, $new_value) = @_;
  my $zinc = $self->get('-zinc');
  my $y = $self->get_y_from_value($new_value);
  $zinc->treset($self->{target_marker});
  $zinc->translate($self->{target_marker}, 0, $y);
  $zinc->itemconfigure ($self->{target_up_label},
			-text => sprintf("%.1f", $new_value),
		       );
}

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');

  my $h; my $w;
if ($self->get('-height') >  $self->get('-width')) {
  $h = $self->get('-height');
  $w = $self->get('-width');
}
  else {
  $w = $self->get('-height');
  $h = $self->get('-width');
}
  $self->{y_per_tick} = $h/$self->get('-disp_tick')+1;
#  printf ("y_per_tick %d\n" ,   $self->{y_per_tick});
  my $rc_pc = $self->get('-fig_clm_pc');  # figures column per cent width
  my $rc_x  = $rc_pc * $w;                # figures column x coordinate
  my $tick_pc = 0.1;                  # tick per cent widht
  my $tick_x = ($rc_pc-$tick_pc)*$w;# tick x coordinate

  my $arrow_height = 10;
  my $arrow_width  = 10;

  my $parent_grp = $self->get('-parent_grp');
  $self->{main_group} = $zinc->add('group', $parent_grp, -visible => 1);
  my @origin = $self->get('-origin');
  $zinc->coords($self->{main_group}, \@origin);
#  $zinc->translate($self->{main_group}, 0, $h/2);

  $self->{fixed_group} = $zinc->add('group', $self->{main_group}, -visible => 1);

  $self->{clipping_group} = $zinc->add('group',$self->{main_group}, -visible => 1);
  $self->{itemclip} = $zinc->add('rectangle',  $self->{clipping_group}, [-10, 0, $w, $h],
				 -visible => 0);
  $zinc->itemconfigure($self->{clipping_group}, -clip => $self->{itemclip});

  $self->{moving_group} = $zinc->add('group',$self->{clipping_group}, -visible => 1);

  $zinc->add('rectangle', $self->{fixed_group} ,
		     [0, 0, $rc_x, $h],
		     -linewidth => 0,
		     -filled => 1,
		     -fillcolor => 'gray60');
  $zinc->add('rectangle', $self->{fixed_group} ,
		     [ $rc_x, 0, $w, $h],
		     -linewidth => 0,
		     -filled => 1,
		     -fillcolor => 'black');
  $zinc->add('curve', $self->{fixed_group},
		     [0, 0, $w, 0],
		     -linewidth => 2,
		     -linecolor => 'white',
		     -filled => 0);
  $zinc->add('curve', $self->{fixed_group},
		     [0, $h, $w, $h],
		     -linewidth => 2,
		     -linecolor => 'white',
		     -filled => 0);
  $zinc->add('curve', $self->{fixed_group},
		     [$rc_x, 0, $rc_x, $h],
		     -linewidth => 2,
		     -linecolor => 'white',
		     -filled => 0);
  $zinc->add('curve', $self->{fixed_group},
		     [$rc_x, $h/2 , 
		      $rc_x + $arrow_width, $h/2 + $arrow_height/2,
		      $rc_x + $arrow_width, $h/2 - $arrow_height/2],
		     -linewidth => 2,
		     -linecolor => 'yellow',
		     -filled => 1,
		     -fillcolor => 'yellow',
		     -closed => 1);
  
  $self->{target_marker} = $zinc->add('curve', $self->{moving_group},
				      [$rc_x-1, 0,
				       $rc_x + $arrow_width+1, $arrow_height/2+1,
				       $rc_x + $arrow_width+1,-$arrow_height/2-1],
				      -linewidth => 2,
				      -linecolor => 'HotPink1',
				      -filled => 0,
				      -closed => 1);



  $zinc->add('curve', $self->{fixed_group},
		     [0, $h/2, $rc_x, $h/2],
		     -linewidth => 2,
		     -linecolor => 'yellow',
		    );

  my $nb_ticks = ($self->get('-max_val') - $self->get('-min_val')) / $self->get('-tick_scale');
  my $tick_font = '-adobe-helvetica-bold-o-normal--18-240-100-100-p-182-iso8859-1';
  my $first_tick = $self->get('-periodic') ? -$nb_ticks:0;
  my $last_tick = $self->get('-periodic') ? 2*$nb_ticks:$nb_ticks;
  for (my $tick=$first_tick; $tick<=$last_tick; $tick++) {
    my $value = ($self->get('-min_val') + $tick) * $self->get('-tick_scale');
    my $y = $self->get_y_from_value($value);
    $zinc->add('curve', $self->{moving_group},
		       [$tick_x, $y, $rc_x, $y],
		     -linewidth => 1,
		     -linecolor => 'white');
    if (!($tick%$self->get('-repeat_legend'))) {
      my $text = sprintf("%d", $value % $self->get('-max_val'));
      $zinc->add('text', $self->{moving_group},
		 -position => [$tick_x/2, $y],
		 -color => 'white',
		 -font => $tick_font,
		 -anchor => 'c',
		 -text => $text);
    }
  } 

  $self->{target_up_label} = 
    $zinc->add('text', $self->{main_group},
	       -position => [$tick_x/2, 0],
	       -color => 'HotPink1',
	       -font => $tick_font,
	       -anchor => 's',
	       -text => "");
 
 
  if ($self->get('-width') > $self->get('-height')) {
    @origin = $self->get('-origin');
    $zinc->rotate($self->{main_group}, - Math::Trig::pip2(), $origin[0], $origin[1]);
  }
}


sub get_y_from_value {
  my ($self, $value) = @_;
  my $h = ($self->get('-height') >  $self->get('-width')) ?
    $self->get('-height') : $self->get('-width');
  my $nb_ticks = ($value - $self->get('-min_val')) / $self->get('-tick_scale');
  return $self->get('-direction') > 0 ? $h/2 - $nb_ticks * $self->{y_per_tick} :
    $h/2 + $nb_ticks * $self->{y_per_tick};
}

1;
