
package Paparazzi::PFD;
use Subject;
@ISA = ("Subject");

use strict;

use Math::Trig;
use Tk;
use Tk::Zinc;

use Paparazzi::Scale;
use Paparazzi::LensScale;
use Paparazzi::Horizon;
use Paparazzi::PFD_Panel;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec
    (-zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
     -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
     -width   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
     -height  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
     
     -selected_ac =>  [S_NOINIT, S_METHOD, S_RDWR, S_OVRWRT, S_NOPRPG, undef],

     -roll           =>  [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, 0],
     -pitch          =>  [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN, 0],

     -speed          => [S_NOINIT,   S_PASSIVE, S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
     -target_speed   => [S_NOINIT,   S_PASSIVE, S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
     -alt            => [S_NOINIT,   S_PASSIVE, S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
     -target_alt     => [S_NOINIT,   S_PASSIVE, S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
     -heading        => [S_NOINIT,   S_PASSIVE, S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
     -target_heading => [S_NOINIT,   S_PASSIVE, S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
     -vz             => [S_NOINIT,   S_PASSIVE, S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
     
     -ap_mode     => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN,    0],
     -gps_mode    => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN,    0],
     -lls_mode    => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN,    0],
     -lls_value   => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN,    0],
     -ctrst_mode  => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN,    0],
     -ctrst_value => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN,    0],
     -rc_mode     => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN,    0],
     -if_mode     => [S_NOINIT, S_PRPGONLY, S_RDWR,   S_OVRWRT, S_CHILDREN,    0],
     
     -nav_dist_wp => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
     -wind        => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
    );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();

  $self->build_gui();
#  $self->configure( -speed => 9);
#  $self->configure( -target_speed => 10);
#  $self->configure( -alt => 140);
#  $self->configure( -target_alt => 150);
#  $self->configure( -roll => 20);
#  $self->configure( -pitch => 10);
  $self->configure('-pubevts' => 'SHOW_PAGE');
}

sub selected_ac {
  my ($self,  $previous_ac, $new_ac) = @_;
  print "###########";
  print "in PFD selected_ac $previous_ac $new_ac\n";

  my @fields = ('roll', 'alt', 'speed', 'course');
  
  foreach my $field ( @fields ) {
    $new_ac->attach($self, $field, [sub { my ($self, $aircraft, $event, $new_value) = @_;
					$self->configure('-'.$field, $new_value);
				      }]);
  }

  @fields = (['mode', '-ap_mode'],
	     ['course', '-heading'],
	    );
  foreach my $field ( @fields ) {
    $new_ac->attach($self, $field->[0], [sub { my ($self, $aircraft, $event, $new_value) = @_;
					$self->configure($field->[1], $new_value);
				      }]);
  }
}

#sub foo_cbk {
#  my ($self, $aircraft, $event, $new_value) = @_;
#  $self->configure('-roll', $new_value);
#  
#}

sub nav_dist_wp {
  my ($self, $previous_d, $new_d) = @_;
  my $str2 = sprintf("dtwp %.0f m", sqrt($new_d));
  $self->get('-zinc')->itemconfigure( $self->{nav_tab}, 2,
				      -text => $str2,
				      -color => 'white',
				    );
}

sub wind {
  my ($self, $previous_w, $new_w) = @_;
#  my ($dir, $speed, $mean_as, $stddev) = $new_w;

  if ($new_w != 0) {
  my $dir_deg = (rad2deg($new_w->{-dir}) + 180)% 360;
  my $wind1_str = sprintf ("%.0fdeg %.1f m/s", $dir_deg, $new_w->{-speed});
  my $wind2_str = sprintf ("mas %.1f m/s (%.2f)", $new_w->{-mean_as}, $new_w->{-stddev});
  $self->get('-zinc')->itemconfigure( $self->{wind_tab}, 1,
				      -text => $wind1_str,
				      -color => 'white',
				    );
  $self->get('-zinc')->itemconfigure( $self->{wind_tab}, 2,
				      -text => $wind2_str,
				      -color => 'white',
				    );
  }
}

sub onPanelCLicked {
  print "in PFD::onPanelClicked\n";
  my ($self, $component, $signal, $page) = @_;
  print "$signal, $page\n";
  $self->notify('SHOW_PAGE', $page);
}

sub min {
  my ($a, $b) = @_;
  return  $a le $b ? $a : $b;
}
sub build_gui() {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  my $origin = $self->get('-origin');

  $self->{main_group} = $zinc->add('group', 1, -visible => 1);
  $zinc->coords($self->{main_group}, $origin);
  my ($p_x, $p_y, $p_w, $p_h) = (0, 0.02*$width, $width, 0.12*$height);
  my $component =  $self->component('Paparazzi::PFD_Panel',
				    -zinc => $zinc,
				    -parent_grp => $self->{main_group},
				    -origin => [$p_x, $p_y],
				    -width => $p_w,
				    -height => $p_h,
		  );
  $component->attach($self, 'CLICKED', ['onPanelCLicked']);


  my ($c_x, $c_y, $radius) = ($width/2, $height/2, 0.3*min($width, $height));
  $self->component('Paparazzi::Horizon',
		   -zinc => $zinc,
		   -parent_grp => $self->{main_group},
		   -origin => [$c_x, $c_y],
		   -radius => $radius,
		  );

  $self->{speed_scale} = Paparazzi::Scale->new( -zinc => $zinc, 
						-parent_grp => $self->{main_group},
						-origin => [ 0.1*$width, 0.25*$height],
						-width => 0.15*$width,
						-height => 0.56*$height,
						-min_val => 0,
						-max_val => 40,
						-tick_scale => 1,
						-repeat_legend => 2,
					      );
  $self->connectoptions(-speed, S_TO, [$self->{speed_scale}, -value]);
  $self->connectoptions(-target_speed, S_TO, [$self->{speed_scale}, -target_value]);
  $self->{heading_scale} = Paparazzi::Scale->new( -zinc => $zinc, 
						  -parent_grp => $self->{main_group},
						  -origin => [ 0.25*$width, 0.96*$height],
						  -width =>    0.5*$width,
						  -height =>   0.08*$height,
						  -direction => -1,
						  -periodic => 1,
						  -min_val => 0,
						  -max_val => 360,
						  -tick_scale => 5,
						  -repeat_legend => 3,
						);
  $self->connectoptions(-heading, S_TO, [$self->{heading_scale}, -value]);
  $self->connectoptions(-target_heading, S_TO, [$self->{heading_scale}, -target_value]);
  $self->{alt_scale} = Paparazzi::LensScale->new( -zinc => $zinc, 
						  -parent_grp => $self->{main_group},
						  -origin => [0.8*$width, 0.25*$height],
						  -width => 0.15*$width,
						  -height => 0.56*$height,
						  -min_val => 0,
						  -max_val => 3000,
						  -tick_scale => 5,
						  -repeat_legend => 2,
						  -fig_clm_pc => 0.6,
						);
  $self->connectoptions(-alt, S_TO, [$self->{alt_scale}, -value]);
  $self->connectoptions(-target_alt, S_TO, [$self->{alt_scale}, -target_value]);
  $self->connectoptions(-vz, S_TO, [$self->{alt_scale}, -vz]);
  # wind informations
  my $labelformat = '150x250 x140x20+20+10 x140x20^0>0 x140x20^0>1 x140x20^0>2 x140x20^0>3';
  my $f = '-adobe-helvetica-bold-o-normal--16-240-100-100-p-182-iso8859-1';
  $self->{wind_tab} =  $zinc->add('tabular',$self->{main_group}, 5,
				  -position => [0,
						600],
				  -labelformat => $labelformat,
				 );
  $zinc->itemconfigure ( $self->{wind_tab}, 0,
			   -font => $f,
			   -color => 'white',
			   -text => 'Wind',
		      );
  # nav informations
  $self->{nav_tab} =  $zinc->add('tabular',$self->{main_group}, 5,
				  -position => [000, 
						600],
				  -labelformat => $labelformat,
				 );
  $zinc->itemconfigure ( $self->{nav_tab}, 0,
			 -font => $f,
			 -color => 'white',
			 -text => 'NAV',
		      );
}


1;
