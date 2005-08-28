package Paparazzi::LensScale;
use Paparazzi::Scale;
@ISA = ("Paparazzi::Scale");
use strict;

use POSIX;
use Subject;

use constant DECIMAL_SPACING => 18;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec( 
		    -vz => [S_NOINIT, S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
		   );
}

use constant VZ_WIDTH => 8;
use constant MAX_VZ => 2. ;

sub vz {
  my ($self, $old_vz, $new_vz) = @_;
  return unless defined $old_vz;
  my $zinc = $self->get('-zinc');
  my $h = $self->get('-height');
  my $y = $new_vz * $h / 2 / MAX_VZ;
  $zinc->coords($self->{vz_itemclip}, [0, 0, VZ_WIDTH, -$y]);
}

sub value() {
  my ($self, $previous, $new) = @_;
  $self->SUPER::value($previous, $new);
  my $zinc = $self->get('-zinc');
  my $int_part = POSIX::floor($new);

  $zinc->itemconfigure ($self->{fixed_text},
			-text => sprintf("%.0f.", $int_part));
  my $decimal = POSIX::floor(($new*10))%10;
  my $new_y = - $decimal * DECIMAL_SPACING;
  $zinc->treset($self->{decimal_group});
  $zinc->translate($self->{decimal_group}, 0, $new_y);
}

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  $self->SUPER::build_gui;
  my $parent_grp = $self->get('-parent_grp');
  $self->{lens_group} = $zinc->add('group', $parent_grp, -visible => 1);

  my ($xorg, $yorg) = $self->get('-origin');
  my $w = $self->get('-width');
  my $h = $self->get('-height');
  my $rc_pc = $self->get('-fig_clm_pc');
  my $h1 = 30;
  my $h2 = 50;
  my $x2 = $rc_pc * $w;
  my $xt = $x2 + 3;
  my $y1 = ($h - $h2)/2;
  my $y2 = ($h - $h1)/2;
  my $y3 = ($h + $h1)/2;
  my $y4 = ($h + $h2)/2;

#  print "foo  $xorg $yorg $w $h\n";

  $zinc->coords($self->{lens_group}, [$xorg, $yorg]);

  $zinc->add('rectangle', $self->{lens_group} ,
	     [0, $y2, $w, $y3],
	     -linewidth => 0,
	     -filled => 1,
	     -fillcolor => 'black',
	    );

  $zinc->add('curve', $self->{lens_group},
	     [0, $y2,
	      $x2, $y2,
	      $x2, $y1,
	      $w - VZ_WIDTH, $y1,
	      $w - VZ_WIDTH, $y4,
	      $x2, $y4,
	      $x2, $y3,
	      0, $y3],
	     -linewidth => 2,
	     -linecolor => 'yellow',
	     -filled => 0);

  my $font = Paparazzi::GuiConfig::get_resource('default', 'normal_font');

  $self->{fixed_text} = $zinc->add('text', $self->{lens_group},
				   -position => [$xt, $h/2],
				   -color => 'white',
				   -font => $font,
				   -anchor => 'e',
				   -text => "00.");

  $self->{decimal_clipping_group} =  $zinc->add('group', $self->{lens_group}, -visible => 1);
  $self->{itemclip} = $zinc->add('rectangle',  $self->{decimal_clipping_group}, [0, $y1, $w, $y4],
				 -visible => 0);
  $zinc->itemconfigure($self->{decimal_clipping_group}, -clip => $self->{itemclip});
  $self->{decimal_group} = $zinc->add('group', $self->{decimal_clipping_group}, -visible => 1);
  for (my $i=-1; $i< 11; $i++) {
    $zinc->add('text', $self->{decimal_group},
	       -position => [$xt, $h/2 + $i * DECIMAL_SPACING],
	       -color => 'white',
	       -font => $font,
	       -anchor => 'w',
	       -text => sprintf("%1d", $i%10)
	      );
  }
  
  $self->{vz_clipping_group} =  $zinc->add('group', $self->{lens_group}, -visible => 1);
  $zinc->coords($self->{vz_clipping_group}, [$w - VZ_WIDTH + 1, $h/2]);
  $self->{vz_itemclip} = $zinc->add('rectangle',  $self->{vz_clipping_group}, [0, -$h/2, VZ_WIDTH, $h/2],
				 -visible => 0);
  $zinc->itemconfigure($self->{vz_clipping_group}, -clip => $self->{vz_itemclip});
  $self->{vz_group} = $zinc->add('group', $self->{vz_clipping_group}, -visible => 1);

  $zinc->add('rectangle', $self->{vz_group} ,
	     [0, -$h/2, VZ_WIDTH, $h/2],
	     -linewidth => 0,
	     -filled => 1,
	     -fillcolor => "=axial 90 |red;150 50|green;150 50", -filled => 1,
#	     -fillcolor => 'red',
	    );
  
  #  print "in LensScale::build_gui\n";
}
