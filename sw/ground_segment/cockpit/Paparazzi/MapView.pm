package Paparazzi::MapView;

use Tk;
#use Tk::widgets qw/PNG/;
#use Tk::JPEG;
use Tk::Zinc;
use XML::DOM;
use Math::Trig;
require File::Basename;

use base "Tk::Frame";
use strict;

Construct Tk::Widget 'MapView';

sub ClassInit {
  my ($class, $mw) = @_;
  $class->SUPER::ClassInit($mw);
}

sub Populate {
  my ($self, $args) = @_;
  my $render = $args->{-render};
  delete $args->{-render};
  $self->SUPER::Populate($args);
  my $zinc = $self->Zinc(-backcolor => 'black',
			 -borderwidth => 3,
			 -relief => 'sunken',
			 -render => $render,
			 -trackmanagedhistorysize => 500,
			 -trackvisiblehistorysize => 500
			);
  $zinc->pack(-fill => 'both', -expand => "1");
  $self->Advertise('zinc' => $zinc);
  $self->build_gui();
  $self->{tracks} = {};
  $self->set_bindings();
}

use constant SCALE_LEN => 200;
use constant SCALE_HEIGHT => 5;
use constant SCALE_X => 50;
use constant SCALE_Y => 1000;

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->Subwidget('zinc');
  $self->{main_group} = $zinc->add('group', 1, -visible => 1);

  $self->{pan_group} = $zinc->add('group', $self->{main_group}, -visible => 1);
  $self->{zoom_group} = $zinc->add('group', $self->{pan_group}, -visible => 1);
  # map
  $self->{map_picture_group} = $zinc->add('group', $self->{zoom_group}, -visible => 1);
  # waypoints
  $self->{map_wp_group} = $zinc->add('group', $self->{zoom_group}, -visible => 1 );
  # track
  $self->{map_trajectory_group} = $zinc->add('group', $self->{zoom_group}, -visible => 1);
  $self->{map_track_group} = $zinc->add('group', $self->{zoom_group}, -visible => 1);

  # scale
  $self->{scale_group} = $zinc->add('group', $self->{main_group}, -visible => 1);
  $self->{scale_item} =  $zinc->add('rectangle',  $self->{scale_group}, 
				    [SCALE_X, SCALE_Y, SCALE_X+SCALE_LEN, SCALE_Y + SCALE_HEIGHT],
				    -visible => 1,
				    -filled => 1,
				    -fillcolor => 'black');
  $self->{scale_text_item} = $zinc->add('text', $self->{main_group},
					-position => [SCALE_X, SCALE_Y - SCALE_HEIGHT],
					-anchor => 'w',
					-text => "unknown");
  $zinc->configure(-overlapmanager =>  $self->{map_track_group} );

}

sub set_bindings {
  my ($self) = @_;
  my $zinc = $self->Subwidget('zinc');
  my $map_grp = $self->{pan_group};
  $zinc->Tk::bind('<ButtonPress-4>', [\&mouse_zoom, $self, 1.15]);
  $zinc->Tk::bind('<ButtonPress-5>', [\&mouse_zoom, $self, 0.75]);
  $zinc->Tk::bind('<ButtonPress-2>', [\&drag_start, $map_grp, \&drag_motion]);
  $zinc->Tk::bind('<ButtonRelease-2>', [\&drag_stop]);
  $zinc->Tk::bind('<ButtonPress-1>', [\&show_pos_cbk, $self]);
  $self->parent()->Tk::bind('<Next>', [\&dec_zoom, $self]);
  $self->parent()->Tk::bind('<Prior>', [\&inc_zoom, $self]);
  $self->parent()->Tk::bind('<Control-Key-c>', [\&clear_track, $self]);
  $self->parent()->Tk::bind('<Escape>', [\&clear_track, $self]);
  $self->parent()->Tk::bind('<Left>', [\&scroll, $self]);
  $self->parent()->Tk::bind('<Right>', [\&scroll, $self]);
  $self->parent()->Tk::bind('<Up>', [\&scroll, $self]);
  $self->parent()->Tk::bind('<Down>', [\&scroll, $self]);
}

sub load_flight_plan {
  my ($self, $xmldata) = @_;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parse($xmldata);

  my $flight_plan = $doc->getElementsByTagName('flight_plan')->[0];

  my $waypoints = $doc->getElementsByTagName('waypoints')->[0];
  $self->{NAV_UTM_EAST0} = $waypoints->getAttribute('utm_x0');
  $self->{NAV_UTM_NORTH0} = $waypoints->getAttribute('utm_y0');

  foreach my $wp ($doc->getElementsByTagName('waypoint')) {
    my ($wp_name, $wp_x_mission, $wp_y_mission, $wp_alt) = 
      ( $wp->getAttribute('name'),
	$wp->getAttribute('x'),
	$wp->getAttribute('y'),
	$wp->getAttribute('alt'));
    my @wp_map = $self->map_of_mission([$wp_x_mission, $wp_y_mission]);

    my $item = $self->Subwidget('zinc')->add( 'waypoint', $self->{map_wp_group}, 1,
					      -position => \@wp_map,
					      -labelformat => 'x20x18+0+0',
					    );
    $self->Subwidget('zinc')->itemconfigure($item, 0,
					    -text => "$wp_name",
					   );
  }
}

sub load_map {
  my ($self, $xml_map) = @_;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($xml_map);
  my $map_node = $doc->getElementsByTagName('map')->[0];
  my $projection = $map_node->getAttribute('projection');
  my $points = $doc->getElementsByTagName('point');
  my @refpoints;
  foreach my $i (0..2) {
    my $p = $points->[$i];
    $refpoints[$i]->{map} = [$p->getAttribute('x'), $p->getAttribute('y')];
    $refpoints[$i]->{geo} = [$p->getAttribute('utm_x'), $p->getAttribute('utm_y')];
  }
  foreach my $i ('map', 'geo') {
    $self->{cal_0}->{$i} = $refpoints[0]->{$i};
    $self->{cal_0X}->{$i} = [subst_c2d($refpoints[1]->{$i}, $refpoints[0]->{$i})];
    $self->{cal_0Y}->{$i} = [subst_c2d($refpoints[2]->{$i}, $refpoints[0]->{$i})];
    $self->{cal_det_OX_0Y}->{$i} = vect_prod_c2d($self->{cal_0X}->{$i}, $self->{cal_0Y}->{$i});
  }

  my $data_dir = File::Basename::dirname($xml_map);
  $data_dir =~ /.*\/[^\/]$/;
  my $map_filename = $data_dir."/".$map_node->getAttribute('file');
  my $image = $self->Subwidget('zinc')->Photo("bg_picture", -file => $map_filename);
  my $img_item = $self->Subwidget('zinc')->add('icon', $self->{map_picture_group},
					       -image => $image);
#  $self->Subwidget('zinc')->coords($self->{pan_group}, [0, -$image->height()]);
  $self->Subwidget('zinc')->treset($self->{pan_group});
#  $self->Subwidget('zinc')->coords($self->{pan_group}, [0 , $image->height()]);
}

sub geo_of_map {
  my ($self, $p_map) = @_;
  my @OP = subst_c2d($p_map, $self->{cal_0}->{map});
#  print "OP [@OP]\n";
  my $cx = vect_prod_c2d(\@OP, $self->{cal_0Y}->{map}) / $self->{cal_det_OX_0Y}->{map};
  my $cy = -vect_prod_c2d(\@OP, $self->{cal_0X}->{map}) / $self->{cal_det_OX_0Y}->{map};
#  print "cx cy $cx $cy\n";
  my @result = add_c2d($self->{cal_0}->{geo},
		       [add_c2d([scale_c2d($self->{cal_0X}->{geo}, $cx)],
				[scale_c2d($self->{cal_0Y}->{geo}, $cy)])]);
#  print "result [@result]\n";
  return @result;
}

sub map_of_geo {
  my ($self, $p_geo) = @_;
  my @OP = subst_c2d($p_geo, $self->{cal_0}->{geo});
  my $cx = vect_prod_c2d(\@OP, $self->{cal_0Y}->{geo}) / $self->{cal_det_OX_0Y}->{geo};
  my $cy = -vect_prod_c2d(\@OP, $self->{cal_0X}->{geo}) / $self->{cal_det_OX_0Y}->{geo};
# print "cx cy $cx $cy\n";

  my @result = add_c2d( $self->{cal_0}->{map},
			[add_c2d([scale_c2d($self->{cal_0X}->{map}, $cx)],
				 [scale_c2d($self->{cal_0Y}->{map}, $cy)])]);
#  print "result [@{$p_geo}] [@result]\n";
  return @result;
}

sub map_of_mission {
  my ($self, $p_mission) = @_;
  my $geo_pos = [$p_mission->[0] + $self->{NAV_UTM_EAST0},
		 $p_mission->[1] + $self->{NAV_UTM_NORTH0}];
#  print "geo @$geo_pos\n";
  my @result = $self->map_of_geo($geo_pos);
#  print "result [@result}\n";
  return @result;
}

sub set_pos_geo {
  my ($self, $pos_utm) = @_;
  my $item = $self->Subwidget('zinc')->add('text', $self->{map_trajectory_group},
					   -position => [$self->map_of_geo($pos_utm)],
					   -color => 'blue',
					   -anchor => 'c',
					   -text => ".");
}

sub set_track_geo {
  my ($self, $name, $pos_utm) = @_;
  return $self->set_track_map($name, [$self->map_of_geo($pos_utm)]);
}

sub set_track_mission {
  my ($self, $name, $pos_xy) = @_;
  return $self->set_track_map($name, [$self->map_of_mission($pos_xy)]);
}


sub set_track_map {
  my ($self, $name, $pos_xy) = @_;
  my $zinc = $self->Subwidget('zinc');
  my $track_item = $self->{tracks}->{$name};
  if (not defined $track_item) {
    $track_item =  $zinc->add( 'track', $self->{map_track_group}, 2,
			       -position => $pos_xy,
			       -labelformat => 'x80x18+0+0',
			     );
    $zinc->itemconfigure($track_item, 0,
			 -text => "$name",
			);  
    $zinc->itemconfigure($track_item,
#			 -filledhistory => 1,
#			 -circlehistory => 1,
#			 -mixedhistory => 1,
			 -symbolcolor => 'green',
			 -leadercolor => 'green',
			 -markersize => '10',
			 -markercolor => 'green',
#			 -historyvisible => 100,
#			 -trackvisiblehistorysize => 100,
#			 -trackmanagedhistorysize => 100,
			 -historycolor =>'black',
	);  
    $zinc->itemconfigure($track_item, 0,
			 -text => "$name",
			);  
    $self->{tracks}->{$name} = $track_item;
  }
  else {
    $zinc->coords($track_item, $pos_xy);
  }
  return $track_item;
}

sub set_picture_mission {
  my ($self, $name, $pos_xy, $heading, $scale) = @_;
  return $self->set_picture_map($name, [$self->map_of_mission($pos_xy)], $heading, $scale);
}



sub set_picture_map {
  my ($self, $name, $pos_xy, $heading, $scale) = @_;
  my $zinc = $self->Subwidget('zinc');
  my $track_item = $self->{tracks}->{$name};
  if (not defined $track_item) {
    my $_w = 720; my $_h = 570;
    my $w = $scale*$_w;
    my $h = $scale*$_h;


    $track_item =  $zinc->add( 'rectangle', $self->{map_track_group},
			       [$pos_xy->[0] - $w/2, $pos_xy->[1] - $h/2, $pos_xy->[0]+$w/2, $pos_xy->[1]+$h/2]
			     );
    $zinc->rotate($track_item, $heading, $pos_xy->[0], $pos_xy->[1] );


    $self->{tracks}->{$name} = $track_item;

    my $image = $zinc->Photo("$name", -file => $name.".gif");
    my $img_item = $zinc->add('icon', $self->{map_track_group},
			      -image => $image);
    $zinc->scale($img_item, $scale, $scale);
    $zinc->translate($img_item ,$pos_xy->[0] - $w/2 , $pos_xy->[1] -$h/2);
    $zinc->rotate($img_item ,$heading, $pos_xy->[0] , $pos_xy->[1]);

  }
  else {
    $zinc->coords($track_item, $pos_xy);
  }
  return $track_item;
}

sub scroll_to_map {
  my ($self, $p1_map, $p2_dev) = @_;
  my $zinc = $self->Subwidget('zinc');

  print "p1_map @$p1_map p2_dev @$p2_dev\n";

  my ($x1_dev, $y1_dev) = $zinc->transform($self->{zoom_group}, 'device',[$p1_map]);
  print "$p1_map $x1_dev $y1_dev \n";
  my ($xt, $yt) = subst_c2d($p2_dev, [$x1_dev, $y1_dev]);
  $zinc->translate($self->{pan_group}, $xt, $yt);
}


sub map_of_dev {
  my ($self, $p_dev) = @_;
  my $zinc = $self->Subwidget('zinc');
  my ($x_m, $y_m) = $zinc->transform('device', $self->{zoom_group}, [$p_dev]);
  print "in map_of_dev @$p_dev -> $x_m, $y_m \n";  
  return ($x_m, $y_m);
}

#####
#
# Callbacks
#
#####

sub mouse_zoom {
  my ($_zinc, $self, $ratio) = @_;
  my $zinc = $self->Subwidget('zinc'); 
  my $ev = $zinc->XEvent();
  my $pointed_on_map = $self->map_of_dev([$ev->x, $ev->y]);
  $zinc->scale($self->{pan_group}, $ratio, $ratio);
#  $self->scroll_to_map([$pointed_on_map], [$ev->x, $ev->y]);
}

sub inc_zoom {
  my ($zinc, $self) = @_;
  my $zzz = $self->Subwidget('zinc'); 
#  my $ev = $zzz->XEvent();
  $zzz->scale($self->{pan_group}, 1.25, 1.25);
}

sub adjust_zoom {
  my ($binded, $self, $ratio) = @_;

  print ("$binded, $self, $ratio\n");

  my $zinc = $self->Subwidget('zinc'); # had to bind on frame for keyboard events ???
  $zinc->scale($self->{pan_group}, $ratio, $ratio);

  my $map_tgroup = $self->{zoom_group};
  my $p0 = $zinc->transform('device', $map_tgroup, [0, 0]);
  my $p1 = $zinc->transform('device', $map_tgroup, [100, 0]);

  print ("$p0 $p1\n");

#  my $v = subst_c2d([$p0], [$p1]);
#  my $mod = module_c2d([$v]);
#  my $r = $mod / SCALE_LEN;
#  print "SCALE_LEN -> $mod : $r\n"
}

sub clear_track {
  my ($zinc, $self) = @_;
  my $zzz = $self->Subwidget('zinc'); 
  $zzz->remove($self->{map_trajectory_group});
  $self->{map_trajectory_group} = $zzz->add('group', $self->{zoom_group}, -visible => 1);
#  $self->scroll_to_map([1000,200], [250,250]);

  printf ("clear\n");
}

my ($x_orig, $y_orig);
sub drag_start {
  print "drag start\n";
  my ($zinc, $item, $action) = @_;
  my $ev = $zinc->XEvent();
  $x_orig = $ev->x;
  $y_orig = $ev->y;
  $zinc->Tk::bind('<Motion>', [$action, $item]);
}

sub drag_motion {
#  print "motion\n";
  my ($zinc, $item) = @_;
  my $ev = $zinc->XEvent();
  my $x = $ev->x;
  my $y = $ev->y;
  $zinc->translate($item, $x-$x_orig, $y-$y_orig);
  $x_orig = $x;
  $y_orig = $y;

}

sub drag_stop {
  print "stop\n";
    my ($zinc) = @_;
    $zinc->Tk::bind('<Motion>', '');
}

sub show_pos_cbk {
  my ($zinc, $self) = @_;
  my $ev = $zinc->XEvent();
  my $x = $ev->x;
  my $y = $ev->y;

  my $map_tgroup = $self->{zoom_group};

  my ($x_m, $y_m) = $zinc->transform('device', $map_tgroup, [$x, $y]);

  print "in show_pos_cbk $x $y -> $x_m, $y_m \n";
}


sub scroll {


}



###
#
#  Geometry.pm
#
###

sub subst_c2d {
  my ($a, $b) = @_;
  my @result = map ( {$a->[$_] - $b->[$_]} 0,1);
  return @result;
}

sub add_c2d {
  my ($a, $b) = @_;
  my @result = map ( {$a->[$_] + $b->[$_]} 0,1);
  return @result;
}

sub module_c2d {
  my ($a) = @_;
  return  sqrt($a->[0]*$a->[0] + $a->[1]*$a->[1]);
}

sub scale_c2d {
  my ($a, $k) = @_;
  my @result = map ( {$a->[$_] * $k} 0,1);
}

sub vect_prod_c2d {
  my ($a, $b) = @_;
  return $a->[0] * $b->[1] - $b->[0] * $a->[1];
}

###
#
# Projections
#
###



#sub utm_of_geo {
#  my ($pos_lon, $pos_lat) = @_;

#  let ellipsoid =  ellipsoid_of geo in
#  my $k0 = 0.9996;
#  my $xs = 500000;
#  my $ys = (phi > 0.) ? 0 : 10000000;
#  let lambda_deg = truncate (floor ((Rad>>Deg)lambda)) in
#  let zone = (lambda_deg + 180) / 6 + 1 in
#  let lambda_c = (Deg>>Rad) (float (lambda_deg - lambda_deg mod 6 + 3)) in
#  let e = ellipsoid.e
#  and n = k0 *. ellipsoid.a in
#  let ll = latitude_isometrique phi e
#  and dl = lambda -. lambda_c in
#  let phi' = asin (sin dl /. cosh ll) in
#  let ll' = latitude_isometrique phi' 0. in
#  let lambda' = atan (sinh ll /. cos dl) in
#  let z = C.make lambda' ll'
#  and c = serie5 coeff_proj_mercator e in
#  let z' = ref (C.scal c.(0) z) in
#  for k = 1 to Array.length c - 1 do
#    z' := C.add !z' (C.scal c.(k) (C.sin (C.scal (float (2*k)) z)))
#  done;
#  z' := C.scal n !z';
#  { utm_zone = zone; utm_x = xs + truncate (C.im !z'); utm_y = ys + truncate (C.re !z') };;




1;
