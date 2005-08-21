# $Id$
#
# Flightplan object
#
# Copyright (C) 2005 Louis Dugrain
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

package Paparazzi::Flightplan;

use Subject;
@ISA = ("Subject");
use strict;

require LWP::Simple;
use XML::DOM;
use Math::Trig;
require File::Basename;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(
                    -url	        => [S_NEEDINIT, S_PASSIVE,  S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
                    -nav_utm_east0	=> [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -nav_utm_north0	=> [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -max_dist_from_home	=> [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -waypoints		=> [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
                    -nb_waypoints	=> [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -mission		=> [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		    -compiled_xml       => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, ""],
		   );
}

# url
# nav_utm_east0
# nav_utm_north0
# max_dist_from_home
# waypoints
#    |-- name
#    |-- x
#    |-- y
#    |-- tag
#    |-- alt
#    |-- type
#    |-- number
# nb_waypoints
# mission
#    |-- block_ID
#		 |-- stage
#				|-- stage_ID
#							|-- text
#	|-- nb_exceptions
#	|-- exception
#					|-- n°
#							|-- cond
#							|-- deroute
#							|-- text
#	|-- description
#	|-- is_deroute
#	|-- [deroute_block]


sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  my $parser = XML::DOM::Parser->new();
  my $flight_plan_url = $self->get('-url');

  print "###### flight_plan_url $flight_plan_url\n";

#  my $flight_plan_xml ="";
  my $flight_plan_xml = LWP::Simple::get($flight_plan_url);
  $self->configure( -compiled_xml => $flight_plan_xml);

#  print "#######flight_plan_xml\n".$flight_plan_xml;

  my $doc = $parser->parse($flight_plan_xml);
#  print "in Flightplan : parsing $file $doc \n";
  $self->{doc} = $doc;
  $self->parse_flight_plan();
}

sub parse_flight_plan {
  my ($self) = @_;
  my $doc = $self->{doc};
  my $flight_plan = $doc->getElementsByTagName('flight_plan')->[0];

  my $max_dist_from_home = $flight_plan->getAttribute('max_dist_from_home');
  $self->{max_dist_from_home} = $max_dist_from_home;

  my $waypoints = $doc->getElementsByTagName('waypoints')->[0];
  $self->{nav_utm_east0} = $waypoints->getAttribute('utm_x0');
  $self->{nav_utm_north0} = $waypoints->getAttribute('utm_y0');
		
  $self->parse_waypoints($doc);
  $self->parse_mission($doc);
  $self->configure_spec();
}

sub configure_spec {
  my ($self) = @_;
  $self->configure(	-nav_utm_east0	 => $self->{nav_utm_east0},
			-nav_utm_north0	 => $self->{nav_utm_north0},
			-max_dist_from_home	=> $self->{max_dist_from_home},
			-waypoints	 => $self->{waypoints},
			-nb_waypoints	 => $self->{nb_waypoints},
			-mission	 => $self->{mission});
}

sub parse_waypoints {
  my ($self, $doc) = @_;
  my $wp_nb = 0;
  foreach my $wp ($doc->getElementsByTagName('waypoint')) {
    $wp_nb++;
    my ($wp_name, $wp_x_mission, $wp_y_mission, $wp_alt) = 
      ( $wp->getAttribute('name'),
	$wp->getAttribute('x'),
	$wp->getAttribute('y'),
	$wp->getAttribute('alt'));
    $self->{waypoints}->{$wp_name}->{x} = $wp_x_mission;
    $self->{waypoints}->{$wp_name}->{y} = $wp_y_mission;
    $self->{waypoints}->{$wp_name}->{alt} = $wp_alt;
    $self->{waypoints}->{$wp_name}->{number} = $wp_nb;
		
    #	We change the tag in order to have no '.' nor '*' because it is use by zinc for pathTag
    my $wp_tag = $wp_name;
    $wp_tag =~ s/\./·/;
    $wp_tag =~ s/\*/~/;
    $self->{waypoints}->{$wp_name}->{tag} = $wp_tag;
    if ($wp_tag =~ /^.*_\d*_\d*$/ or $wp_tag =~ /^.*__.*$/) {
      # This is a secondary waypoint (defined from another one)
      $self->{waypoints}->{$wp_name}->{type} = 'sedondary';
    } else {
      # This is a primary waypoint
      $self->{waypoints}->{$wp_name}->{type} = 'primary';
    }
  }
  $self->{nb_waypoints} = $wp_nb;
}

sub parse_mission {
  my ($self, $doc) = @_;
  my ($blocks, $blocks_stages);
  my ($nb_excep, $excep_cond, $excep_deroute);
  my (@deroute, $deroute_block);

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
  
#  use Data::Dumper;
#  print Dumper($blocks);
#  print Dumper(\$blocks_stages);

  foreach my $block ($doc->getElementsByTagName('block')) {
    my $block_name = $block->getAttribute('NAME').$block->getAttribute('name');
#    print "################### $block_name\n";
    my $block_description = $block->getAttribute('description');
    my $block_id = $blocks->{$block_name};

    $self->{mission}->{$block_id}->{description} = $block_description;
    foreach my $line (split (/(\n)/, $block->toString())) {
      my $key = $line;
      $key =~ s/^\s*//;		# remove any leading whitespace
      $key =~ s/\s*$//;		# remove any trailing whitespace
      if ($key ne "") {
	my $stage_id = $blocks_stages->{$block_name}->{$key};
	my $tags = [$block_id];
	push(@{$tags}, ($stage_id)) if defined $stage_id;
	#				$self->Subwidget('text')->insert('end', $line."\n", $tags);
	if (defined $stage_id) {
	  # print "reading $block_id $stage_id  block name  $block_name\n";
	  $self->{mission}->{$block_id}->{stage}->{$stage_id}->{text} = $line;
					
	  # Parse the stage_line to get a wp_name defined with `wp="wp_name"`
	  my $wp_name = $line;
	  $wp_name =~ s/^.*\s[wW][pP]="([^"]*)".*$/$1/;
	  $wp_name =~ s/^(\s*<.*>\s*)$//;
	  if ($wp_name ne "") {
	    $self->{mission}->{$block_id}->{stage}->{$stage_id}->{waypoint} = $wp_name;
	  }
					
	  #	I think we can do this better...
	  my $type = $line;
	  if ($type =~ /^.*<xyz.*$/) {
	    $self->{mission}->{$block_id}->{stage}->{$stage_id}->{type} = 'xyz';
	  }
	  if ($type =~ /^.*<circle.*$/) {
	    $self->{mission}->{$block_id}->{stage}->{$stage_id}->{type} = 'circle';
	  }
	  if ($type =~ /^.*<go.*$/) {
	    $self->{mission}->{$block_id}->{stage}->{$stage_id}->{type} = 'go';
	  }
	  if ($type =~ /^.*<attitude.*$/) {
	    $self->{mission}->{$block_id}->{stage}->{$stage_id}->{type} = 'go';
	  }

	}
      }
    }
    $nb_excep = 0;
    foreach my $except ($block->getElementsByTagName('exception')) {
      $nb_excep++;
      $excep_cond = $except->getAttribute('cond');
      $excep_deroute = $except->getAttribute('deroute');
			
      $self->{mission}->{$block_id}->{exception}->{$nb_excep}->{text} = $except->toString();
      $self->{mission}->{$block_id}->{exception}->{$nb_excep}->{cond} = $excep_cond;
      $self->{mission}->{$block_id}->{exception}->{$nb_excep}->{deroute} = $excep_deroute;
    }
    $self->{mission}->{$block_id}->{nb_exceptions} = $nb_excep;
		
    @deroute = $block->getElementsByTagName('deroute');
    if ($#deroute >= 1) {
      $self->{mission}->{$block_id}->{is_deroute} = 1;
      $deroute_block = $deroute[0];
      $self->{mission}->{$block_id}->{deroute_block} = $deroute_block;
    } else {
      $self->{mission}->{$block_id}->{is_deroute} = 0;
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

1;
