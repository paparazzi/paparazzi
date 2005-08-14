# $Id$
#
# MapView object
# 
# Copyright (C) 2005 Antoine Drouin, Louis Dugrain
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

package Paparazzi::MapView;
use Subject;
@ISA = ("Subject");

use strict;

use Tk;
use Tk::PNG;
use Tk::JPEG;
use Tk::Zinc;
use XML::DOM;
use Math::Trig;
require File::Basename;



sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec (
		     -mw    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		     -height    => [S_NOINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 600],
		     -width			=> [S_NOINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 770],
		    );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();

  my $mw = $self->get('-mw');
  my $zinc = $mw->Zinc(-backcolor => 'black',
			 										-borderwidth => 3,
													-relief => 'sunken',
													-render => '0');
  $zinc->pack(-fill => 'both', -expand => "1");
  $self->{map_widget} = $zinc;
  $self->default_palette();
  my $ressource_file = Paparazzi::Environment::get_config("gui.xml");
  $self->load_user_palette($ressource_file);
  $self->default_configuration();
  $self->load_configuration($ressource_file);
  

  $self->build_gui();

  my $map_file = Paparazzi::Environment::get_default_map();

  $self->load_map($map_file, [scalar $self->get('-width'), scalar $self->get('-height')]);




  $self->{tracks} = {};
  $self->set_bindings();
}

use constant SCALE_LEN => 200;
use constant SCALE_HEIGHT => 5;
use constant SCALE_X => 50;
use constant SCALE_Y => 1000;

sub build_gui {
  my ($self) = @_;
	my $zinc = $self->{map_widget};
  $self->{main_group} = $zinc->add('group', 1, -visible => 1);

  $self->{pan_group} = $zinc->add('group', $self->{main_group}, -visible => 1);
  $self->{zoom_group} = $zinc->add('group', $self->{pan_group}, -visible => 1);
  # map
  $self->{map_picture_group} = $zinc->add('group', $self->{zoom_group},
														-visible => $self->{configuration}->{map});
  # waypoints
  $self->{map_wp_group} = $zinc->add('group', $self->{zoom_group}, -visible => 1, -priority => 5 );
  $self->{map_circle_group} = $zinc->add('group', $self->{zoom_group},
															-visible => 0, -priority => 5,
															-tags => 'circle_group' );
  $self->{map_segment_group} = $zinc->add('group', $self->{zoom_group},
															-visible => 0, -priority => 5,
															-tags => 'segment_group' );
  # track
  $self->{map_trajectory_group} = $zinc->add('group', $self->{zoom_group}, -visible => 1, -priority => 6 );
  $self->{map_track_group} = $zinc->add('group', $self->{zoom_group}, -visible => 1, -priority => 7 );

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
	
	# Max_dist_from_Home mask
  $self->{max_dist_mask} = $zinc->add('group', $self->{zoom_group},
					-visible => $self->{configuration}->{max_dist_from_home_mask}, -priority => 3 );
  $self->{max_dist_circle} = $zinc->add('group', $self->{zoom_group},
					-visible => $self->{configuration}->{max_dist_from_home_circle}, -priority => 4 );
	
  # grid
	$self->{grid_group} = $zinc->add('group', $self->{zoom_group},
					-visible => $self->{configuration}->{grid}, -priority => 2 );
}

sub set_bindings {
  my ($self) = @_;
  my $zinc = $self->{map_widget};
  my $mw = $self->get('-mw');
  my $map_grp = $self->{pan_group};
#	$zinc->Tk::bind('<Configure>', [\&resize]);
  $zinc->Tk::bind('<ButtonPress-4>', [\&mouse_zoom, $self, 1.15]);
  $zinc->Tk::bind('<ButtonPress-5>', [\&mouse_zoom, $self, 0.75]);
  $zinc->Tk::bind('<ButtonPress-2>', [\&drag_start, $map_grp, \&drag_motion]);
  $zinc->Tk::bind('<ButtonRelease-2>', [\&drag_stop]);
  $zinc->Tk::bind('<ButtonPress-1>', [\&show_pos_cbk, $self]);
  $mw->Tk::bind('<Next>', [\&dec_zoom, $self]);
  $mw->Tk::bind('<Prior>', [\&inc_zoom, $self]);
  $mw->Tk::bind('<Control-Key-c>', [\&clear_track, $self]);
  $mw->Tk::bind('<Escape>', [\&clear_track, $self]);
  $mw->Tk::bind('<Left>', [\&scroll, $self, 1, 0]);
  $mw->Tk::bind('<Right>', [\&scroll, $self, -1, 0]);
  $mw->Tk::bind('<Up>', [\&scroll, $self, 0, 1]);
  $mw->Tk::bind('<Down>', [\&scroll, $self, 0, -1]);
  $mw->Tk::bind('<m>', [\&toggle_map, $self]);
  $mw->Tk::bind('<g>', [\&toggle_grid, $self]);
#	$mw->Tk::bind('<c>', [\&toggle_max_dist_circle, $self]);
#	$mw->Tk::bind('<c>', [\&center_on_aircraft, $self, 0, -1]);
}

sub load_flight_plan {
  my ($self, $xmldata, $win_size) = @_;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parse($xmldata);
	my $zinc = $self->{map_widget};

  my $flight_plan = $doc->getElementsByTagName('flight_plan')->[0];

  my $waypoints = $doc->getElementsByTagName('waypoints')->[0];
  $self->{NAV_UTM_EAST0} = $waypoints->getAttribute('utm_x0');
  $self->{NAV_UTM_NORTH0} = $waypoints->getAttribute('utm_y0');

	# The sub which highlights overflight waypoint
	my $highlight_overflight_wp = sub {
#		$self->set_wp_color('current', $self->{palette}->{highlighted_waypoint});
		$self->set_wp_color('current', 'highlighted_waypoint');
		$zinc->itemconfigure('current', 1, -border => 'contour', -filled => 'true');
		$zinc->itemconfigure('current', -labelformat => 'x200x18+0+0 a0x18^0^0' );
		$zinc->addtag('above_tag', 'above', 'current');
		$zinc->raise('current');
		#	print "I fly over wp... \n";
	};
	# The sub which unhighlights overflight waypoint
	my $unhighlight_overflight_wp = sub {
		$self->clear_wp_color('current');
		$zinc->itemconfigure('current', 1, -border => '', -filled => 'false');
		$zinc->itemconfigure('current', -labelformat => 'x200x18+0+0 x50x18^0^0' );
		foreach my $aboved_item ($zinc->find('withtag', 'above_tag')) {
			$zinc->lower('current', $aboved_item); }
		$zinc->dtag('above_tag');
		#	print "I leave wp... \n";
	};


  foreach my $wp ($doc->getElementsByTagName('waypoint')) {
    my ($wp_name, $wp_x_mission, $wp_y_mission, $wp_alt) = 
      		( $wp->getAttribute('name'),
						$wp->getAttribute('x'),
						$wp->getAttribute('y'),
						$wp->getAttribute('alt'));
    my @wp_map = $self->map_of_mission([$wp_x_mission, $wp_y_mission]);
		
		#	We change the ta in order not to have '.' or '*' because it is use by zinc for pathTag
		my $wp_tag = $wp_name;
		$wp_tag =~ s/\./·/;
		$wp_tag =~ s/\*/~/;
		# We create 2 labels, first one is not visible but is used to let the automatic format of
		# the second one when mouse fly over the waypoint (unless, we get a lot of 'in' and 'out' events)
    my $item = $zinc->add( 'waypoint', $self->{map_wp_group}, 2,
													-position => \@wp_map,
					      					-labeldistance => 10,
													-leaderanchors => '%0x0',
													-labelformat => 'x200x18+0+0 x50x18^0^0',
													-tags => [$wp_tag, $wp_x_mission, $wp_y_mission]);
		#	print "Add waypoint $wp_name with tag $wp_tag\n";
    $zinc->itemconfigure($item, 1,
													-backcolor => $self->{palette}->{back_screen},
													-bordercolor => $self->{palette}->{highlighted_waypoint},
													-filled => 'false',
													-border => '',
					   							-text => "$wp_name" );
    $zinc->itemconfigure($item, 0,
					   							-visible => 0 );
#		$self->get_color($item, 'back_screen');
		if ($wp_tag =~ /^.*_\d*_\d*$/ or $wp_tag =~ /^.*__.*$/) {
			# This is a secondary waypoint (defined from another one)
#			$self->set_wp_color($item, $self->{palette}->{secondary_waypoint});
			$self->set_wp_color($item, 'secondary_waypoint');
			$zinc->addtag('secondary', 'withtag', $wp_tag);}
		else {
			# This is a primary waypoint
#			$self->set_wp_color($item, $self->{palette}->{waypoint});
			$self->set_wp_color($item, 'waypoint');
			$zinc->addtag('primary', 'withtag', $wp_tag);}
		$zinc->bind("$item:1", '<Enter>', $highlight_overflight_wp);
		$zinc->bind("$item:leader", '<Enter>', $highlight_overflight_wp);
		$zinc->bind("$item:position", '<Enter>', $highlight_overflight_wp);
		$zinc->bind("$item:1", '<Leave>', $unhighlight_overflight_wp);
		$zinc->bind("$item:leader", '<Leave>', $unhighlight_overflight_wp);
		$zinc->bind("$item:position", '<Leave>', $unhighlight_overflight_wp);
  }
	
	my $waypoint_home_tag = [$zinc->gettags($zinc->find('withtag', "HOME"))];
	my ($waypoint_home_x, $waypoint_home_y) = ($waypoint_home_tag->[1], $waypoint_home_tag->[2]);
#	print "find HOME (x, y) ($waypoint_home_x, $waypoint_home_y) \n";
	my $max_dist_from_home = $flight_plan->getAttribute('max_dist_from_home');
	$self->{flight_plan}->{Id1}->{max_dist_from_home} = $max_dist_from_home;
  my @minus_max_dist = $self->map_of_mission([-$max_dist_from_home + $waypoint_home_x,
																							-$max_dist_from_home + $waypoint_home_y]);
  my @plus_max_dist = $self->map_of_mission([$max_dist_from_home + $waypoint_home_x,
																						 $max_dist_from_home + $waypoint_home_y]);
	
	my $max_dist_circle = $zinc->add('arc', $self->{max_dist_circle},
																		[@minus_max_dist, @plus_max_dist],
																		-filled => 0,
																		-linewidth => 3,
																		-linecolor => $self->{palette}->{max_dist_from_home_circle});
																		
	my $max_dist_circle_mask = $zinc->add('arc', $self->{max_dist_mask},
																				[@minus_max_dist, @plus_max_dist],
																				-visible => 0, -filled => 1);
																							
	my $max_dist_rect_mask = $zinc->find('withtag', 'max_dist_rect_mask');
	my $max_dist_rect_mask_clone = $zinc->find('withtag', 'max_dist_rect_mask_clone');
	$zinc->contour($max_dist_rect_mask, 'add', -1, $max_dist_circle_mask);
	
	$zinc->itemconfigure($self->{max_dist_mask}, -clip => $max_dist_rect_mask_clone);
	
	if ($self->{is_centered_by_user} == 0) {
  	$self->scroll_to_map([$self->map_of_mission([0, 0])], [$win_size->[0]/2, $win_size->[1]/2]);
	}
	$self->trace_grid();


#	That is the copy of MissionD.pm
  my ($blocks, $blocks_stages);

  foreach my $stage ($doc->getElementsByTagName('stage')) {
    my $block_name = $stage->getAttribute('block_name');
    my $block_no = $stage->getAttribute('block');
    my $stage_no = $stage->getAttribute('stage');
    my $stage_text = "";
    my $stage_kids = $stage->getChildNodes();
    foreach my $kid (@{$stage_kids}) {
      $stage_text = $stage_text.$kid->toString() if $kid->getNodeType() != TEXT_NODE;
    }
    $blocks_stages->{$block_name}->{$stage_text} = get_stage_id($block_no, $stage_no);
    $blocks->{$block_name} = get_block_id($block_no) unless defined $blocks->{block_name};
  }

  #  print Dumper(\$blocks);
  #  print Dumper(\$blocks_stages);

  foreach my $block ($doc->getElementsByTagName('block')){
    my $block_name = $block->getAttribute('name');
    foreach my $line (split (/(\n)/, $block->toString())) {
      my $key = $line;
      $key =~ s/^\s*//; # remove any leading whitespace
      $key =~ s/\s*$//; # remove any trailing whitespace
      if ($key ne "") {
				my $block_id = $blocks->{$block_name};
				my $stage_id = $blocks_stages->{$block_name}->{$key};
				my $tags = [$block_id];
				push(@{$tags}, ($stage_id)) if defined $stage_id;
#				$self->Subwidget('text')->insert('end', $line."\n", $tags);
				if (defined $stage_id) {
					# print "reading $block_id $stage_id  block name  $block_name\n";
					$self->{mission}->{$block_id}->{$stage_id} = $line; }
      }
    }
  }
}

sub get_block_id {
  my ($no_block) = @_;
  return "block_".$no_block;
}

sub get_stage_id {
  my ($no_block, $no_stage) = @_;
  return "stage_".$no_block."_".$no_stage;
}

sub load_map {
  my ($self, $xml_map, $win_size) = @_;
	my $zinc = $self->{map_widget};
	$self->{win_size} = $win_size;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($xml_map);
	$self->read_palette($doc, 'map');
	$self->apply_palette();
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

  my $map_filename = Paparazzi::Environment::get_data($map_node->getAttribute('file'));
	print "map filename $map_filename \n";
  my $image = $zinc->Photo("bg_picture", -file => $map_filename);
  my $img_item = $zinc->add('icon', $self->{map_picture_group},
					       						-image => $image,
														-visible => 1,
														-tags => ['image', $image->width(), $image->height()]);
	$self->{image_data}->{size} = [$image->width(), $image->height()];
	
#  $self->{map_widget}->coords($self->{pan_group}, [0, -$image->height()]);
  $self->{map_widget}->treset($self->{pan_group});
#  $self->{map_widget}->coords($self->{pan_group}, [0 , $image->height()]);
	
	$self->load_map_options($doc);
	$self->create_max_dist_rect_mask();
}

sub default_palette {
	# Define default palette
  my ($self) = @_;
	$self->{palette}->{waypoint} = "black";
	$self->{palette}->{secondary_waypoint} = "gray25";
	$self->{palette}->{highlighted_waypoint} = "blue3";
	$self->{palette}->{desired_waypoint} = "red";
	$self->{palette}->{max_dist_from_home_circle} = "red3";
	$self->{palette}->{carrot_symbol} = "red";
	$self->{palette}->{carrot_label} = "dark";
	$self->{palette}->{aircraft_symbol} = "green";
	$self->{palette}->{aircraft_label} = "dark";
	$self->{palette}->{selected_aircraft} = "orange";
	$self->{palette}->{grid} = "navy";
	$self->{palette}->{back_screen} = "gray73";
	$self->{palette}->{track} = "blue";

	$self->{palette}->{default}->{waypoint} = "black";
	$self->{palette}->{default}->{secondary_waypoint} = "gray25";
	$self->{palette}->{default}->{highlighted_waypoint} = "blue3";
	$self->{palette}->{default}->{desired_waypoint} = "red";
	$self->{palette}->{default}->{max_dist_from_home_circle} = "red3";
	$self->{palette}->{default}->{carrot_symbol} = "red";
	$self->{palette}->{default}->{carrot_label} = "dark";
	$self->{palette}->{default}->{aircraft_symbol} = "green";
	$self->{palette}->{default}->{aircraft_label} = "dark";
	$self->{palette}->{default}->{selected_aircraft} = "orange";
	$self->{palette}->{default}->{grid} = "navy";
	$self->{palette}->{default}->{back_screen} = "gray73";
	$self->{palette}->{default}->{track} = "blue";

	$self->{palette}->{map}->{waypoint} = "yellow";

	$self->{palette}->{user}->{waypoint} = "green";
}

sub default_configuration {
	# Define default palette
  my ($self) = @_;
	$self->{configuration}->{grid} = "yes";
	$self->{configuration}->{map} = "yes";
	$self->{configuration}->{max_dist_from_home_mask} = "no";
	$self->{configuration}->{max_dist_from_home_circle} = "yes";
	$self->{configuration}->{grid_only_on_map} = "no";
	$self->{configuration}->{grid_step} = 100;
}

sub read_palette {
	# Read palette from a doc
  my ($self, $doc, $type) = @_;
  my $xml_palette = $doc->getElementsByTagName('palette')->[0];
	if (defined $xml_palette) {
		#	print "Reading palette... \n";
    my $palette_attributes = $xml_palette->getAttributes();
		my ($attrib_name, $attrib_value);
		my $atributes_nb = $palette_attributes->getLength();
		for (my $i=0; $i<$atributes_nb; $i++) {
			$attrib_name = $palette_attributes->item($i)->getNodeName();
			$attrib_value = $palette_attributes->item($i)->getNodeValue();
			#	print "attribute '$attrib_name' valueing '$attrib_value' \n";
			
			if ($attrib_name =~ /^.*waypoint.*$/) {
				$self->{palette}->{$type}->{$attrib_name} = $attrib_value; }
			else {
				$self->{palette}->{$attrib_name} = $attrib_value; }
    }
	}
}

sub apply_palette {
	#	Apply palette to previously created elements
	my ($self) = @_;
	my $zinc = $self->{map_widget};
	$zinc->configure(-backcolor => $self->{palette}->{back_screen});
	#	print "in apply_palette \n";
}

sub load_user_palette {
	# Load user palette (defined in conf/ground_segment.xml)
  my ($self, $xml_gui) = @_;
	my $zinc = $self->{map_widget};
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($xml_gui);
	my $map_element = $doc->getElementsByTagName('map')->[0];
	$self->read_palette($map_element, 'user');
	#	print "loading default user palette... \n"
}

sub load_configuration {
  my ($self, $xml_gui) = @_;
	my $zinc = $self->{map_widget};
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($xml_gui);
	my $map_element = $doc->getElementsByTagName('map')->[0];
	my $configuration = $map_element->getElementsByTagName('configuration')->[0];
	if (defined $configuration) {
		#	print "Reading palette... \n";
    my $configuration_attributes = $configuration->getAttributes();
		my ($attrib_name, $attrib_value);
		my $atributes_nb = $configuration_attributes->getLength();
		for (my $i=0; $i<$atributes_nb; $i++) {
			$attrib_name = $configuration_attributes->item($i)->getNodeName();
			$attrib_value = $configuration_attributes->item($i)->getNodeValue();
			#	print "attribute '$attrib_name' valueing '$attrib_value' \n";
			$self->{configuration}->{$attrib_name} = $attrib_value;
    }
	}
}

sub load_map_options {
	# Read and load map options
  my ($self, $doc) = @_;
	my $zinc = $self->{map_widget};
  my $xml_user_options = $doc->getElementsByTagName('user')->[0];
	my ($user_zoom, $user_center);
	$self->{is_centered_by_user} = 0;
	if (defined $xml_user_options) {
		$user_zoom = $xml_user_options->getAttribute('zoom');
#		$self->{map_options}->{user_zoom} = $user_zoom;
		$user_center = [$xml_user_options->getAttribute('x'), $xml_user_options->getAttribute('y')];
#		$self->{map_options}->{user_center} = $user_center;
	}
	if (defined $user_zoom and $user_zoom ne "") {
		$zinc->scale($self->{pan_group}, $user_zoom, $user_zoom); }
	if (defined $user_center and $user_center->[0] ne "" and $user_center->[1] ne "") {
		$self->scroll_to_map([$user_center->[0], $user_center->[1]], [$self->{win_size}->[0]/2, $self->{win_size}->[1]/2]);
		$self->{is_centered_by_user} = 1;
	}
}

sub create_max_dist_rect_mask {
	# Create the max_dist_from_home mask  (usefull only with OpenGL)
  my ($self) = @_;
	my $zinc = $self->{map_widget};
	my $img_size = $self->{image_data}->{size};
	# Define max_dist mask rectangle with a curve in order to retire the circle
	# (I think it is not possible with a real rectangle element)
	my $max_dist_rect_mask = $zinc->add('curve', $self->{max_dist_mask},
																			[[0, 0], [$img_size->[0], 0],
																				[$img_size->[0], $img_size->[1]], [0, $img_size->[1]]],
																			-closed => 1,
																			-visible => 1,
																			-filled => 1,
																			-fillcolor => "=axial 0 |red;30|red;30",
																			-linewidth => 0,
																			-tags =>'max_dist_rect_mask');
	# This second rectangle is just for cliping (Don't manage to use clone.
	# I think it is because it is a son and not a brother of cloned element)
	my $max_dist_rect_mask_clone = $zinc->add('curve', $self->{max_dist_mask},
																			[[0, 0], [$img_size->[0], 0],
																				[$img_size->[0], $img_size->[1]], [0, $img_size->[1]]],
																			-closed => 1,
																			-visible => 0,
																			-tags =>'max_dist_rect_mask_clone');
}

sub trace_grid {
# Trace a grid on the map
  my ($self) = @_;
	my $zinc = $self->{map_widget};
	my $max_dist_from_home = $self->{flight_plan}->{Id1}->{max_dist_from_home};
	my $grid_step = $self->{configuration}->{grid_step};
	my $grid_only_on_map = $self->{configuration}->{grid_only_on_map};
	my $img_size = $self->{image_data}->{size};
	my ($image_x, $image_y) = ($img_size->[0], $img_size->[1]);
	if (not defined $image_y) {$image_y = $image_x; }
	
	# Coordonnées de l'image en mètres
	my $map_size_map_1 = $self->mission_of_map([0, 0]);
	my $map_size_map_2 = $self->mission_of_map([$image_x, $image_y]);

	# Coordonnées du point HOME en mètres
	my $wp_home_tag = [$zinc->gettags($zinc->find('withtag', "HOME"))];
	my $wp_home_map = [$wp_home_tag->[1], $wp_home_tag->[2]];
	my ($wp_home_x, $wp_home_y) = ($wp_home_map->[0], $wp_home_map->[1]);
		
	my ($mis_point_1, $mis_point_2);
	my ($map_point_1, $map_point_2);
	if ($grid_only_on_map eq 'true' or $grid_only_on_map eq 'yes') {
		($mis_point_1, $mis_point_2) = ($map_size_map_1, $map_size_map_2);
		($map_point_1, $map_point_2) = ([0, 0], $img_size);
	}
	else {
		($mis_point_1, $mis_point_2) = ([-2*$max_dist_from_home, -2*$max_dist_from_home],
											 							[2*$max_dist_from_home, 2*$max_dist_from_home]);
		$mis_point_1 = [-2*$max_dist_from_home, 2*$max_dist_from_home];
		$mis_point_2 = [2*$max_dist_from_home, -2*$max_dist_from_home];
#		print "mis_point_1: @$mis_point_1, map_size_map_1: @$map_size_map_1 \n";
#		print "mis_point_2: @$mis_point_2, map_size_map_2: @$map_size_map_2 \n";
		($map_point_1, $map_point_2) = ([$self->map_of_mission($mis_point_1)], [$self->map_of_mission($mis_point_2)]);
#		print "dev_point_1: $map_point_1, dev_point_2: $map_point_2, img_size: @$img_size \n";
	}
	
	my ($i, $j);
	# Print vertical lines
#	for ($i = int(($map_size_map_1->[0] - $wp_home_x)/$grid_step); $i <= int(($map_size_map_2->[0] - $wp_home_x)/$grid_step); $i++) {
	for ($i = int(($mis_point_1->[0] - $wp_home_x)/$grid_step); $i <= int(($mis_point_2->[0] - $wp_home_x)/$grid_step); $i++) {
		$j = [$self->map_of_mission([$i*$grid_step + $wp_home_x, $wp_home_y])]->[0];
		#	print "$i $j \n";
		$zinc->add('curve', $self->{grid_group}, [[$j, $map_point_1->[1]], [$j, $map_point_2->[1]]],
								-visible => 1,
								-linewidth => 0.5,
								-linecolor => $self->{palette}->{grid} );
	}
	
	# Print horizontal lines
#	for ($i = int(($map_size_map_2->[1] - $wp_home_y)/$grid_step); $i <= int(($map_size_map_1->[1] - $wp_home_y)/$grid_step); $i++) {
	for ($i = int(($mis_point_2->[1] - $wp_home_y)/$grid_step); $i <= int(($mis_point_1->[1] - $wp_home_y)/$grid_step); $i++) {
		$j = [$self->map_of_mission([$wp_home_x, $i*$grid_step + $wp_home_y])]->[1];
		#	print "$i $j \n";
		$zinc->add('curve', $self->{grid_group}, [[$map_point_1->[0], $j], [$map_point_2->[0], $j]],
								-visible => 1,
								-linewidth => 0.5,
								-linecolor => $self->{palette}->{grid} );
	}
}

#sub set_wp_color {
## To put a color to a waypoint
#  my ($self, $wp_Id, $color) = @_;
#	$self->color_wp($wp_Id, $color, $color);
#}

sub set_wp_color {
# To put a color to a waypoint
  my ($self, $wp_Id, $color_type) = @_;
	my $color = $self->get_color($wp_Id, $color_type);
	$self->color_wp($wp_Id, $color, $color);
}

sub clear_wp_color {
# To clear the color of a waypoint and come back to first one
  my ($self, $wp_Id) = @_;
	if ($self->{map_widget}->hastag($wp_Id, 'desired')) {
#		$self->set_wp_color($wp_Id, $self->{palette}->{desired_waypoint}); }
		$self->set_wp_color($wp_Id, 'desired_waypoint'); }
	elsif ($self->{map_widget}->hastag($wp_Id, 'primary')) {
#		$self->set_wp_color($wp_Id, $self->{palette}->{waypoint}); }
		$self->set_wp_color($wp_Id, 'waypoint'); }
	else {
#		$self->set_wp_color($wp_Id, $self->{palette}->{secondary_waypoint); }
		$self->set_wp_color($wp_Id, 'secondary_waypoint'); }
}

sub color_wp {
# To put a color to a waypoint
  my ($self, $wp_Id, $label_color, $symbol_color) = @_;
	my $zinc = $self->{map_widget};
	$zinc->itemconfigure($wp_Id, 1,
												-color => $label_color);
	$zinc->itemconfigure($wp_Id, 
												-leadercolor => $symbol_color,
												-symbolcolor => $symbol_color );
}

sub highlight_desired_wp {
# To highlight desired waypoint
  my ($self, $wp_name) = @_;
	my $zinc = $self->{map_widget};
	my $wp_tag = $wp_name;
	$wp_tag =~ s/\./·/;				# In order not to have '.' or '*' in pathTag
	$wp_tag =~ s/\*/~/;
	my $current_wp = [$zinc->find('withtag', $wp_tag)]->[0];
	$self->set_wp_color($current_wp, 'desired_waypoint');
	$zinc->addtag('desired', 'withtag', $wp_tag);
	#	print "Highlighting waypoint $wp_name with tag $wp_tag \n";
	
	$self->{mission}->{last_highlighted_wp} = $wp_name;
}

sub unhighlight_desired_wp {
# To unhighlight desired waypoint
  my ($self) = @_;
	my $zinc = $self->{map_widget};
	if (defined $self->{mission}->{last_highlighted_wp}) {
		my $wp_tag = $self->{mission}->{last_highlighted_wp};
		$wp_tag =~ s/\./·/;			# In order not to have '.' or '*' in pathTag
		$wp_tag =~ s/\*/~/;
		my $current_wp = [$zinc->find('withtag', $wp_tag)]->[0];
		$zinc->dtag($current_wp, 'desired');
		$self->clear_wp_color($current_wp);
	}
}

sub examine_stage_line {
# Examine the stage line and call highlight_wp()
  my ($self, $cur_block_no, $cur_stage_no) = @_;
	my $cur_block_id = get_block_id($cur_block_no);
	my $cur_stage_id = get_stage_id($cur_block_no, $cur_stage_no);
	if (not defined $self->{mission}->{cur_stage_id} or not $self->{mission}->{cur_stage_id} eq $cur_stage_id) {
		$self->unhighlight_desired_wp();
		my $stage_line = $self->{mission}->{$cur_block_id}->{$cur_stage_id};
		if (defined $stage_line) {
			my $wp_name = $stage_line;
			# Parse the stage_line to get a wp_name defined with `wp="wp_name"`
			$wp_name =~ s/^.*\s[wW][pP]="([^"]*)".*$/$1/;
			$wp_name =~ s/^(\s*<.*>\s*)$//;
			if ($wp_name ne "") {
				#	print "Examining block $cur_block_no and stage $cur_stage_no \n";
				$self->highlight_desired_wp($wp_name); }
		}
	}
	$self->{mission}->{cur_stage_id} = $cur_stage_id;
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
#  print "result [@result]\n";
  return @result;
}

sub mission_of_map {
  my ($self, $p_map) = @_;
	my $geo_pos = [$self->geo_of_map($p_map)];
  my $result = [$geo_pos->[0] - $self->{NAV_UTM_EAST0},
		 $geo_pos->[1] - $self->{NAV_UTM_NORTH0}];
  return $result;
}

sub set_pos_geo {
  my ($self, $pos_utm) = @_;
  my $item = $self->{map_widget}->add('text', $self->{map_trajectory_group},
					   -position => [$self->map_of_geo($pos_utm)],
					   -color => $self->{palette}->{track},
					   -anchor => 'c',
					   -text => "·");
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
  my $zinc = $self->{map_widget};
  my $track_item = $self->{tracks}->{$name};
  if (not defined $track_item) {
    $track_item = $zinc->add( 'track', $self->{map_track_group}, 2,
			       									-position => $pos_xy,
			       									-labelformat => 'x60x18+0+0' );
    $zinc->itemconfigure($track_item, 0,
			 										-text => "$name" );
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
  my $zinc = $self->{map_widget};
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
    $zinc->translate($img_item, $pos_xy->[0] - $w/2, $pos_xy->[1] -$h/2);
    $zinc->rotate($img_item, $heading, $pos_xy->[0], $pos_xy->[1]);

  }
  else {
    $zinc->coords($track_item, $pos_xy);
  }
  return $track_item;
}

sub scroll_to_map {
  my ($self, $p1_map, $p2_dev) = @_;
  my $zinc = $self->{map_widget};

#  print "p1_map @$p1_map p2_dev @$p2_dev\n";
  my ($x1_dev, $y1_dev) = $zinc->transform($self->{zoom_group}, 'device', [$p1_map]);
#  print "$p1_map $x1_dev $y1_dev \n";
  my ($xt, $yt) = subst_c2d($p2_dev, [$x1_dev, $y1_dev]);
  $zinc->translate($self->{pan_group}, $xt, $yt);
}

sub map_of_dev {
  my ($self, $p_dev) = @_;
  my $zinc = $self->{map_widget};
  my ($x_m, $y_m) = $zinc->transform('device', $self->{zoom_group}, [$p_dev]);
#  print "in map_of_dev @$p_dev -> $x_m, $y_m \n";  
  return ([$x_m, $y_m]);
}

sub dev_of_map {
  my ($self, $p_map) = @_;
  my $zinc = $self->{map_widget};
  my ($x_dev, $y_dev) = $zinc->transform($self->{zoom_group}, 'device', [$p_map]);
#  print "in dev_of_map @$p_map -> $x_dev, $y_dev \n";  
  return ([$x_dev, $y_dev]);
}

sub dev_of_geo {
  my ($self, $p_geo) = @_;
	my $p_map = [$self->map_of_geo([$p_geo->[0], $p_geo->[1]])];
	my $result = $self->dev_of_map($p_map);
#  print "in dev_of_geo (geo -> map -> dev) @$p_geo -> @$p_map -> @$result \n";  
  return ($result);
}

sub dev_of_mission {
  my ($self, $p_mis) = @_;
	my $p_map = [$self->map_of_mission([$p_mis->[0], $p_mis->[1]])];
	my $result = $self->dev_of_map($p_map);
#  print "in dev_of_mission (mis -> map -> dev) @$p_mis -> @$p_map -> @$result \n";  
  return ($result);
}

sub set_view_on_plane {
# Could be improve with the tag of plane
  my ($self, $pos_utm) = @_;
	my $pos_dev = $self->dev_of_geo($pos_utm);
	my ($pos_x, $pos_y) = ($pos_dev->[0], $pos_dev->[1]);
	my $win_size = $self->get_window_size();
	my $border = 50;
	
	if ($pos_x < $border and $pos_x > -$win_size->[0] ) {
		$self->scroll($self, 1, 0);
		print "Scrolling right to see plane... \n"; }
	if ($pos_x > $win_size->[0]-$border and $pos_x < 2*$win_size->[0] ) {
		$self->scroll($self, -1, 0);
		print "Scrolling left to see plane... \n"; }
	if ($pos_y < $border and $pos_y > -$win_size->[1] ) {
		$self->scroll($self, 0, 1);
		print "Scrolling top to see plane... \n"; }
	if ($pos_y > $win_size->[1]-$border and $pos_y < 2*$win_size->[0] ) {
		$self->scroll($self, 0, -1);
		print "Scrolling bottom to see plane... \n"; }
#	print "in set_view_on_plane (x, y) ($pos_x, $pos_y) \n";
}

sub get_color {
	# Determine if the object is over the map and choose the appropriate color
  my ($self, $object, $color_type) = @_;
  my $zinc = $self->{map_widget};
	my $img_item = $zinc->find('withtype', 'icon', 'image');
	my $img_tags = [$zinc->gettags($img_item)];
	if (not defined $img_tags->[2]) { $img_tags->[2] = $img_tags->[1]; }
	my $map_size = [$img_tags->[1], $img_tags->[2]];

	my $position = [$zinc->itemcget($object, -position)];
	my $ok = 0;
	my $result;
	if ($zinc->itemcget($self->{map_picture_group}, -visible)
			and not ($position->[0] < 0 or $position->[0] > $map_size->[0]
			or $position->[1] < 0 or $position->[1] > $map_size->[1])) {
		if (defined $self->{palette}->{map}->{$color_type}) {
			$result = $self->{palette}->{map}->{$color_type};
			$ok = 1; }
	}
	elsif (defined $self->{palette}->{user}->{$color_type}) {
		$result = $self->{palette}->{user}->{$color_type};
		$ok = 1; }
	if ($ok == 0) {
		if (defined $self->{palette}->{default}->{$color_type}) {
			$result = $self->{palette}->{default}->{$color_type}; }
		else { $result = 'black'; }
	}
	#	print "In get_color. Return $return \n";
	return $result;
}

sub irpt {
	#	***********************
	#	***** In progress *****
	#	***********************
  my ($self, $object, $color_type) = @_;
  my $zinc = $self->{map_widget};
	foreach my $wp_item ($zinc->find('withtype', 'waypoint')) {
		$self->clear_wp_color($wp_item); }
}

sub create_circle {
  my ($self) = @_;
	# Create a circle with its center which will be used to trace the followed circle
  my $zinc = $self->{map_widget};
	$zinc->add('curve', $self->{map_circle_group},
							[[-2, 0], [2, 0]],
							-closed => 0,
							-linecolor => $self->{palette}->{desired_waypoint},
							-tags => 'reticule' );
	$zinc->add('curve', $self->{map_circle_group},
							[[0, -2], [0, 2]],
							-closed => 0,
							-linecolor => $self->{palette}->{desired_waypoint},
							-tags => 'reticule' );
	$zinc->add('arc', $self->{map_circle_group},
							[[-60, -60], [60, 60]],
							-linecolor => $self->{palette}->{desired_waypoint},
							-tags => 'circle', );
	$self->{circle}->{center_x} = 0;
	$self->{circle}->{center_y} = 0;	
}

sub create_segment {
	# Create a segment which will be used to trace the followed route
  my ($self) = @_;
  my $zinc = $self->{map_widget};
	$zinc->add('curve', $self->{map_segment_group},
							[[0, 0], [10, 10]],
							-closed => 0,
							-linecolor => $self->{palette}->{desired_waypoint},
							-tags => 'segment' );
}

sub set_circle {
  my ($self, $center_x, $center_y, $radius) = @_;
	# Set the current circle center and radius and show it
  my $zinc = $self->{map_widget};
	my $pos_map = [$self->map_of_mission([$center_x, $center_y])];
	my ($x_map, $y_map) = ($pos_map->[0], $pos_map->[1]);
	#	print "in set_circle with center (x, y) ($center_x, $center_y) in mission -> ($x_map, $y_map) in map \n";
	$zinc->itemconfigure($self->{map_circle_group}, -visible => 1);
	my ($x_trans, $y_trans) = ($x_map - $self->{circle}->{center_x}, $y_map - $self->{circle}->{center_y});
	$zinc->translate($self->{map_circle_group}, $x_trans, $y_trans);
	$self->{circle}->{center_x} = $x_map;
	$self->{circle}->{center_y} = $y_map;
	my $circle = $zinc->find('withtag', 'circle');
	my $zero_map = [$self->map_of_mission([0, 0])];
	my $radius_map_1 = [$self->map_of_mission([-$radius, -$radius])];	
	my $radius_map_2 = [$self->map_of_mission([$radius, $radius])];
	my $point_1 = [$radius_map_1->[0] - $zero_map->[0], $radius_map_1->[1] - $zero_map->[1]];
	my $point_2 = [$radius_map_2->[0] - $zero_map->[0], $radius_map_2->[1] - $zero_map->[1]];
	$zinc->coords($circle, [$point_1, $point_2]);
}

sub verify_circle_mode {
  my ($self, $cur_block_no, $cur_stage_no) = @_;
	# Verify the current stage to hide the circle if we are not anymore in a circle mode
  my $zinc = $self->{map_widget};
	my $cur_block_id = get_block_id($cur_block_no);
	my $cur_stage_id = get_stage_id($cur_block_no, $cur_stage_no);
	my $stage_line = $self->{mission}->{$cur_block_id}->{$cur_stage_id};
	if (defined $stage_line) {
		my $is_circle = $stage_line;
		# Find if we are in a xyz or a circle mode
		if (not $is_circle =~ /^.*<xyz.*$/ and not $is_circle =~ /^.*<circle.*$/) {
			$zinc->itemconfigure($self->{map_circle_group}, -visible => 0);
		}
	}
}

sub set_segment {
  my ($self, $segment_x_1, $segment_y_1, $segment_x_2, $segment_y_2) = @_;
	# Set the current route segment and show it
#	return;
  my $zinc = $self->{map_widget};
	my $pos_1_map = [$self->map_of_mission([$segment_x_1, $segment_y_1])];
	my $pos_2_map = [$self->map_of_mission([$segment_x_2, $segment_y_2])];
	#	print "in set_segment from (@$pos_1_map) to (@$pos_2_map) \n";
	$zinc->itemconfigure($self->{map_segment_group}, -visible => 1);
	my $segment = $zinc->find('withtag', 'segment');
	my $zero_map = [$self->map_of_mission([0, 0])];
#	my $point_1 = [$pos_1_map->[0] - $zero_map->[0], $pos_1_map->[1] - $zero_map->[1]];
#	my $point_2 = [$pos_2_map->[0] - $zero_map->[0], $pos_2_map->[1] - $zero_map->[1]];
	my $point_1 = [$pos_1_map->[0] - $zero_map->[0], $pos_1_map->[1] - $zero_map->[1]];
	my $point_2 = [$pos_2_map->[0] - $zero_map->[0], $pos_2_map->[1] - $zero_map->[1]];
#	$zinc->coords($segment, [$point_1, $point_2]);
	$zinc->coords($segment, [$pos_1_map, $pos_2_map]);
}

sub verify_segment_mode {
  my ($self, $cur_block_no, $cur_stage_no) = @_;
	# Verify the current stage to hide the circle if we are not anymore in a circle mode
  my $zinc = $self->{map_widget};
	my $cur_block_id = get_block_id($cur_block_no);
	my $cur_stage_id = get_stage_id($cur_block_no, $cur_stage_no);
	my $stage_line = $self->{mission}->{$cur_block_id}->{$cur_stage_id};
	if (defined $stage_line) {
		my $is_segment = $stage_line;
		# Find if we are in go mode
		if (not $is_segment =~ /^.*<go.*$/) {
			$zinc->itemconfigure($self->{map_segment_group}, -visible => 0);
		}
	}
}

#####
#
# Callbacks
#
#####

sub mouse_zoom {
  my ($_zinc, $self, $ratio) = @_;
  my $zinc = $self->{map_widget}; 
  my $ev = $zinc->XEvent();
	my $x = $ev->x;
	my $y = $ev->y;

#	if you want to zoom on mouse
#	my $pointed_on_map = $self->map_of_dev([$ev->x, $ev->y]);

#	if you want to zoom on center
	my $win_size = $self->get_window_size();
  my $pointed_on_map = $self->map_of_dev([$win_size->[0]/2, $win_size->[1]/2]);
  $zinc->scale($self->{pan_group}, $ratio, $ratio);

#	if you want to zoom on mouse
#	$self->scroll_to_map($pointed_on_map, [$ev->x, $ev->y]);

#	if you want to zoom on center
  $self->scroll_to_map($pointed_on_map, [$win_size->[0]/2, $win_size->[1]/2]);
}

sub inc_zoom {
  my ($tsointsoin, $self) = @_;
  my $zinc = $self->{map_widget}; 
	my $ratio = 2.;
#  my $ev = $zzz->XEvent();
	my $win_size = $self->get_window_size();
  my $pointed_on_map = $self->map_of_dev([$win_size->[0]/2, $win_size->[1]/2]);
  $zinc->scale($self->{pan_group}, $ratio, $ratio);
  $self->scroll_to_map($pointed_on_map, [$win_size->[0]/2, $win_size->[1]/2]);
}

sub dec_zoom {
  my ($tsointsoin, $self) = @_;
  my $zinc = $self->{map_widget}; 
	my $ratio = 0.5;
#  my $ev = $zzz->XEvent();
	my $win_size = $self->get_window_size();
  my $pointed_on_map = $self->map_of_dev([$win_size->[0]/2, $win_size->[1]/2]);
  $zinc->scale($self->{pan_group}, $ratio, $ratio);
  $self->scroll_to_map($pointed_on_map, [$win_size->[0]/2, $win_size->[1]/2]);
}

sub adjust_zoom {
  my ($binded, $self, $ratio) = @_;

  print ("$binded, $self, $ratio\n");

  my $zinc = $self->{map_widget}; # had to bind on frame for keyboard events ???
  $zinc->scale($self->{pan_group}, $ratio, $ratio);

  my $map_tgroup = $self->{zoom_group};
  my $p0 = $zinc->transform('device', $map_tgroup, [0, 0]);
  my $p1 = $zinc->transform('device', $map_tgroup, [100, 0]);

  print ("$p0 $p1\n");

#  my $v = subst_c2d([$p0], [$p1]);
#  my $mod = module_c2d([$v]);
#  my $r = $mod / SCALE_LEN;
#  print "SCALE_LEN -> $mod : $r \n"
}

sub clear_track {
  my ($zinc, $self) = @_;
  my $zzz = $self->{map_widget}; 
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
  my ($zinc) = @_;
  print "stop\n";
  $zinc->Tk::bind('<Motion>', '');
}

sub show_pos_cbk {
  my ($zinc, $self) = @_;
  my $ev = $zinc->XEvent();
  my $x = $ev->x;
  my $y = $ev->y;

  my ($x_m, $y_m) = $zinc->transform('device', $self->{zoom_group}, [$x, $y]);

  print "in show_pos_cbk $x $y -> $x_m, $y_m \n";
}


sub scroll {
# Scroll the map. 1 unit correspond to 1/4 of the window size.
  my ($useless, $self, $x_movement, $y_movement) = @_;
  my $zinc = $self->{map_widget};
#  my $p0 = $zinc->transform('device', $self->{zoom_group}, [0, 0]);
	my $win_size = $self->get_window_size();
	
	my $x_move = $x_movement * $win_size->[0] / 4;
	my $y_move = $y_movement * $win_size->[1] / 4;
	
  $zinc->translate($self->{pan_group}, $x_move, $y_move);
#  print "scroll the map \n";
}

sub toggle_map {
# Show or hide the map
  my ($useless, $self) = @_;
  my $zinc = $self->{map_widget};
#	my $img_item = $zinc->find('withtype', 'icon', 'image');
#	$self->{map_picture_group}
	$zinc->itemconfigure($self->{map_picture_group}, -visible => 1 - $zinc->itemcget($self->{map_picture_group}, -visible));
	$self->irpt();
	#	print "toogle the map \n";
}

sub toggle_grid {
# Show or hide the grid
  my ($useless, $self) = @_;
  my $zinc = $self->{map_widget};
	$zinc->itemconfigure($self->{grid_group}, -visible => 1 - $zinc->itemcget($self->{grid_group}, -visible));
  #	print "toggle the grid \n";
}

sub toggle_max_dist_circle {
# Scroll the map. 1 unit correspond to 1/4 of the window size.
  my ($useless, $self) = @_;
  my $zinc = $self->{map_widget};
	$zinc->itemconfigure($self->{grid_group}, -visible => 1 - $zinc->itemcget($self->{grid_group}, -visible));
  #	print "toggle the grid \n";
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

sub get_window_size {
	my ($self) = @_;
  my $zinc = $self->{map_widget};
	return ([$zinc->cget(-width), $zinc->cget(-height)]);
}




1;
