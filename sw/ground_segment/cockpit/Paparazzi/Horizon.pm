#=============================================================================
#                 Horizon  Class
#=============================================================================
package Paparazzi::Horizon;
use Subject;
@ISA = ("Subject");
use strict;

use Paparazzi::Utils;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc       => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -parent_grp => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin     => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -radius     => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -roll       => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
		    -pitch      => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
 );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->build_gui();
}

sub roll {
  my ($self, $old_angle, $new_angle) = @_;
  return unless defined $old_angle;
  my $dangle = Utils::rad_of_deg($old_angle - $new_angle);
  $self->get('-zinc')->rotate($self->{horizon_rotate_group}, $dangle, 0, 0);
}

sub pitch {
  my ($self, $old_angle, $new_angle) = @_;
  return unless defined $old_angle;
  my $dy = $self->dy_of_angle($new_angle - $old_angle);
  $self->get('-zinc')->translate($self->{horizon_translate_group}, 0, $dy);
}

sub dy_of_angle {
  my ($self, $angle) = @_;
  return $angle * $self->{y_per_deg};
}

sub build_gui {
  my ($self) = @_;
  my $parent_grp = $self->get('-parent_grp');
  my $zinc = $self->get('-zinc');
  my $radius = $self->get('-radius');

  $self->{horizon_group} = $zinc->add('group', $parent_grp, -visible => 1);
  my $origin = $self->get('-origin');
  $zinc->coords($self->{horizon_group}, $origin);


  $self->{horizon_rotate_group} = $zinc->add('group', $self->{horizon_group}, -visible => 1);
  $self->{horizon_translate_group} = $zinc->add('group', $self->{horizon_rotate_group}, -visible => 1);


  $self->{fixed_group} = $zinc->add('group', $self->{horizon_group}, -visible => 1);

  $self->{horizon_itemclip} = $zinc->add('arc',  $self->{horizon_rotate_group},
					 [-$radius, -$radius, $radius, $radius],
					 -visible => 0);
  $zinc->itemconfigure($self->{horizon_rotate_group}, -clip => $self->{horizon_itemclip});

  # horizon earth
  $zinc->add('rectangle', $self->{horizon_translate_group} ,
		 [-$radius, 0, $radius, 3 * $radius],
		 -linewidth => 0, -filled => 1,-fillcolor => '#986701', #'orange',
	    );
  # horizon sky
  $zinc->add('rectangle', $self->{horizon_translate_group} ,
	     [-$radius, -3 * $radius, $radius, 0],
	     -linewidth => 0, -filled => 1, -fillcolor => '#0099cb',  # 'blue'
	    );
  # center line
  $zinc->add('curve', $self->{horizon_translate_group},
	     [-$radius, 0, $radius, 0],
	     -linewidth => 2, -linecolor => 'white', -filled => 0);

  # pitch scale
  $self->{y_per_deg} = $radius / 30;
  my $v_tick_font = Paparazzi::GuiConfig::get_resource('default', 'small_font');
  my $i;
  for ($i=-16; $i <= 16; $i++) {
    my $angle = $i*2.5;
    my $y = $self->dy_of_angle($angle);
    my $x = $radius / 16;
    if (!($i%4)) {$x = $radius / 4;}
    elsif (!($i%2)) {$x = $radius / 8};
    $zinc->add('curve', $self->{horizon_translate_group},
	       [-$x, $y, $x, $y],
	       -linewidth => 1,
	       -linecolor => 'white',
	       -filled => 0);
 
    if (!($i%4) & ($i != 0)) {
      my $text_lab = sprintf("%d", $angle);
      my @text_attr = ( -color => 'white',
			-font => $v_tick_font,
			-text => $text_lab );
      $zinc->add('text', $self->{horizon_translate_group},
		 -position => [-$x-10, -$y],
		 -anchor => 'e',
		 @text_attr);
      $zinc->add('text', $self->{horizon_translate_group},
		 -position => [$x+10, -$y],
		 -anchor => 'w',
		 @text_attr);
    }
  }

  # arrow
  my $arrow_len = 10;
  $zinc->add('curve', $self->{horizon_rotate_group},
	     [0, -$radius+1, 
	      -$arrow_len+1, -$radius+$arrow_len,
	      $arrow_len-1, -$radius+$arrow_len],
	     -linewidth => 2,
	     -linecolor => 'yellow',
	     -filled => 0, 
	     -closed => 1);

  # roll scale
  $zinc->add('arc',  $self->{fixed_group},
	     [-$radius + 1, -$radius + 1, $radius - 1, $radius - 1],
	     -startangle => -120,
	     -extent => 60,
	     -linewidth => 2,
	     -linecolor => 'white',
	     -filled => 0);

  for ($i=-4; $i <= 4; $i++) {
    my $angle = Utils::rad_of_deg($i * 15);
    my $x1 = 0 * cos($angle) - $radius * sin($angle);
    my $y1 = 0 * sin($angle) - $radius * cos($angle);
    my $x2 = 0 * cos($angle) - ($radius+10) * sin($angle);
    my $y2 = 0 * sin($angle) - ($radius+10) * cos($angle);
    $zinc->add('curve', $self->{fixed_group},
	       [$x1, $y1, $x2, $y2],
	       -linewidth => 1,
	       -linecolor => 'white',
	       -filled => 0);
  }


# fixed indicator
  my @center_sign = [ -3, -3,
		      -3,  3,
		      3,   3,
		      3,  -3];
  
  my @left_sign = [-50, -3,
		   -80, -3,
		   -80, 1,
		   -50, 1,
		   -50, 15,
		   -46, 15,
		   -46, -3];
  my @right_sign = [50, -3,
		    80, -3,
		    80, 1,
		    50, 1,
		    50, 15,
		    46, 15,
		    46, -3];
  my @fixed_indic_attr = ( -linewidth => 1,
			 -linecolor => 'yellow',
			 -filled => 1,
			 -fillcolor => 'black',
			 -closed => 1
			 );
  foreach my $sign_section (\@center_sign, \@left_sign, \@right_sign) {
    $zinc->add('curve', $self->{fixed_group},
	       @{$sign_section},
	       @fixed_indic_attr);
  }

  #  side black masks
  my $pc_black = 0.18;
  my @side_masks_attr = ( -linewidth => 0,
			  -filled => 1,
			  -fillcolor => 'black' );
  $zinc->add('rectangle', $self->{fixed_group} ,
	     [(1-$pc_black)*$radius , -$radius,
	      $radius, $radius],
	     @side_masks_attr);
  $zinc->add('rectangle', $self->{fixed_group} ,
	     [-$radius , -$radius,
	      -(1-$pc_black)*$radius, $radius],
	     @side_masks_attr);
}


1;
