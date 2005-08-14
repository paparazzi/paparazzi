package MapView_Ivy;


use Math::Trig;
use IvyMsgs;

use base "MapView";

Construct Tk::Widget 'MapView_Ivy';

sub ClassInit {
  my ($class, $mw) = @_;
  $class->SUPER::ClassInit($mw);
}

sub Populate {
  my ($self, $args) = @_;
  $self->SUPER::Populate($args);
  $self->ConfigSpecs( -ivy => ['PASSIVE', undef, undef, undef]);
  my $ivy = $args->{-ivy};
  $self->register_callbacks($ivy)
}

sub register_callbacks {
  my ($self, $ivy) = @_;
  $ivy->bindRegexp (IvyMsgs::GPS_Regexp(), [$self, \&ivyOnGPS]);
  $ivy->bindRegexp (IvyMsgs::DESIRED_Regexp(), [$self, \&ivyOnDesired]);
  $ivy->bindRegexp (IvyMsgs::NAVIGATION_REF_Regexp(), [$self, \&ivyOnNavigationRef]);
  $ivy->bindRegexp (IvyMsgs::NAVIGATION_Regexp(), [$self, \&ivyOnNavigation]);
  $ivy->bindRegexp (IvyMsgs::CIRCLE_Regexp(), [$self, \&ivyOnCircle]);
  $ivy->bindRegexp (IvyMsgs::SEGMENT_Regexp(), [$self, \&ivyOnSegment]);
}

my $gps_track_configured = 0;

sub ivyOnGPS {
  my ($self, $sender, $mode, $utm_x, $utm_y, $course, $alt, $speed, $climb) = @_;

  $utm_x = $utm_x/100;
  $utm_y = $utm_y/100;

  $self->set_pos_geo([$utm_x, $utm_y]);
  my $track_item = $self->set_track_geo("aircraft-0", [$utm_x, $utm_y]);

  #  print "$utm_x $utm_y \n";
  if ($gps_track_configured == 0) {
#    my $zinc = $self->Subwidget('zinc');
    my $zinc = $self->{map_widget};
    $zinc->itemconfigure($track_item,
			 -filledhistory => 0,
			 -circlehistory => 1,
			 -symbolcolor => $self->{palette}->{aircraft_symbol},
			 -leadercolor => $self->{palette}->{aircraft_symbol},
			 -markersize => '1',
			 -markercolor => $self->{palette}->{aircraft_symbol},
			 -historyvisible => 40,
			);
    $zinc->itemconfigure($track_item, 0,
			 -color => $self->{palette}->{aircraft_label},
			);
    $gps_track_configured = 1;
  }
	
	$self->set_view_on_plane([$utm_x, $utm_y]);
}

my $desired_track_configured = 0;
sub ivyOnDesired {
  my ($self, $sender, $roll, $pitch, $desired_x, $desired_y, $desired_altitude) = @_;
  my $track_item = $self->set_track_mission("ac0-desired", [$desired_x, $desired_y]);
  #  print ("ac0-desired $desired_x $desired_y\n");
  
  if ($desired_track_configured == 0) {
#    my $zinc = $self->Subwidget('zinc');
		my $zinc = $self->{map_widget};
    $zinc->itemconfigure($track_item,
			 -historyvisible => '0',
			 -filledhistory => 0,
			 -circlehistory => 0,
			 -symbolcolor => $self->{palette}->{carrot_symbol},
			 -leadercolor => $self->{palette}->{carrot_symbol},
			 #			 -markersize => '1',
			 #			 -markercolor => 'green',
			);
    $zinc->itemconfigure($track_item, 0,
			 -color => $self->{palette}->{carrot_label},
			);
    $desired_track_configured = 1;
  }
}

sub ivyOnNavigationRef {
  my ($self, $sender, $utm_east, $utm_north) = @_;
}

sub ivyOnNavigation {
  my ($self, $sender, $cur_block, $cur_stage, $pos_x, $pos_y, $desired_course, $dist2_wp, $course_pgain, $dist2_home) = @_;
  #  printf("NAVIGATION wp $cur_wp, x $pos_x, y $pos_y, dc $desired_course, d2wp $dist2_wp, cpg $course_pgain\n");
  $self->examine_stage_line($cur_block, $cur_stage);
	$self->verify_circle_mode($cur_block, $cur_stage);
	$self->verify_segment_mode($cur_block, $cur_stage);
}

sub ivyOnCircle {
  my ($self, $sender, $center_x, $center_y, $radius) = @_;
  $self->set_circle($center_x, $center_y, $radius);
}

sub ivyOnSegment {
  my ($self, $sender, $segment_x_1, $segment_y_1, $segment_x_2, $segment_y_2) = @_;
  $self->set_segment($segment_x_1, $segment_y_1, $segment_x_2, $segment_y_2);
}

1;
