package Paparazzi::SatPage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;

use Paparazzi::SatSigView;

use constant TITLE => "Satellites";
use constant MAX_CH => 16;

sub populate {
  my ($self, $args) = @_;
  $args->{-title} = TITLE;
  $self->SUPER::populate($args);
  $self->configspec(
		    -sats =>    [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, undef],
		    -fix  =>    [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, undef],
		   );
}


sub sats {
  my ($self, $old_val, $new_val) = @_;
  return unless defined $new_val;
 
  my $itow = $new_val->{-itow};
  my $nb_ch = $new_val->{-nch};
  my $sats =  $new_val->{-sats};
  my $zinc = $self->get('-zinc');

  foreach my $i (0..MAX_CH-1) {
    my $sat_obj = $self->{satellites}->[$i];
    unless ($new_val->{svid}->[$i] == 0) {
      my $pos = $self->get_pos($new_val->{elev}->[$i], $new_val->{azim}->[$i]);
#      print "in SatPage::sats $i $new_val->{svid}->[$i] $new_val->{elev}->[$i], $new_val->{azim}->[$i] @{$pos}\n";
      $zinc->coords($sat_obj->{-group}, $pos);
      $zinc->itemconfigure ($sat_obj->{-group}, -visible => 1 );
      $zinc->itemconfigure ($sat_obj->{-arc},
			    -fillcolor =>  $new_val->{flags}->[$i] & 0x01 ? "green3" : "red",
			   );
      $zinc->itemconfigure ($sat_obj->{-id_lab},
			    -text => sprintf("%d", $new_val->{svid}->[$i]),
			   );
      $sat_obj->{-sig_view}->configure(-svid => $new_val->{svid}->[$i], -cno => $new_val->{cno}->[$i]);
    }
    else { $zinc->itemconfigure ($sat_obj->{-group}, -visible => 0 )}
  }
}


sub fix {
  my ($self, $old_val, $new_val) = @_;
#  print "in fix\n";
  return unless defined $new_val and defined $self->{rg};
#  print "in fix2\n";
}


sub get_pos {
  my ($self, $elev, $azim) = @_;
  my $sky_radius = $self->{sky_radius};
  my $azim_rad = Utils::rad_of_deg($azim);

  use constant LIN => 1;

  my $k_elev = LIN ? 1 - $elev/90 : 1 - sin(Utils::rad_of_deg($elev));
  my $x = $sky_radius *  1 * sin($azim_rad) * $k_elev;
  my $y = $sky_radius * -1 * cos($azim_rad) * $k_elev;
  return [$x, $y];
}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  $self->{sky_group} = $zinc->add('group', $self->{main_group});
  my $margin = 10;
  my $sky_radius = Utils::min($width, $height)*0.55/2;
  $zinc->coords($self->{sky_group}, [$sky_radius+$margin, $sky_radius+$margin+$self->{vmargin}]);
  $self->{sky_radius} = $sky_radius;
  # elevation scale
  for (my $elev = 0; $elev < 90; $elev += 15) {
    my $rad = $self->get_pos($elev, 0)->[1];
    $zinc->add('arc',  $self->{sky_group},
	       [-$rad, -$rad, $rad, $rad],
	       -visible => 1,
	       -linecolor => 'white',
	       -filled => 0,
	      );
    $zinc->add('text', $self->{sky_group},
	       -position => $self->get_pos(90-$elev, 180-$elev),
	       -text => sprintf("%.0f", $elev),
	       -color => 'white',
	       -anchor => 'c',
	      );
  }
  # azimut scale
  my $tick_font = '-adobe-helvetica-bold-o-normal--24-240-100-100-p-182-iso8859-1';
  my $ticks = [["S", [0,  $sky_radius]], ["E", [ $sky_radius, 0]],
	       ["N", [0, -$sky_radius]], ["W", [-$sky_radius, 0]]];
  foreach my $tick (@{$ticks}) {
    my ($txt, $pos) = @{$tick};
    $zinc->add('text', $self->{sky_group},
	       -position => $pos,
	       -text => $txt,
	       -color => 'white',
	       -font => $tick_font,
	       -anchor => 'c',
	      );
  }
  my $pos_sig_x = 0.6*$width+$margin;;
  my $h_sig = $height/ (MAX_CH +1);
  my $satellites = [];
  my $sat_r = $sky_radius/8;
  for (my $chn=0; $chn < MAX_CH; $chn++) {
    my $sat_group =  $zinc->add('group', $self->{sky_group}, -visible => 0);
    my $sat_arc = $zinc->add('arc', $sat_group, [- $sat_r, - $sat_r, $sat_r, $sat_r],
			     -visible => 1,
			     -linecolor => 'white',
			     -filled => 1,
			    ); 
    my $id_lab = $zinc->add('text', $sat_group,
			    -position => [0, 0],
			    -text => "$chn",
			    -color => 'white',
			    -anchor => 'c',
			   );
    my $sat_sig_view = Paparazzi::SatSigView->new(-zinc => $zinc,
						  -width => $width/3,
						  -height => $h_sig,
						  -origin => [$pos_sig_x, ($chn+0.5) * $h_sig],
						  -parent_grp => $self->{main_group},
						 );

    push @{$satellites},  {
			   -sig_view => $sat_sig_view,
			   -group => $sat_group,
			   -arc => $sat_arc,
			   -id_lab => $id_lab,
			   -elev => 0,
			   -azim => 0,
			  };
    
  }
  $self->{satellites} = $satellites;
}
