#
# $Id$
#
# PapaStrip::Strip (Aircraft flight informations)
#
# Copyright (C) 2005 Pierre-Selim Huard
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


package Paparazzi::Strip;

use Subject;
@ISA=("Subject");

use Tk;
use Tk::Zinc;

use XML::DOM;
use Data::Dumper;

use strict;
use warnings;

use Paparazzi::Traces;

# populate:
#   this sub is the subject constructor method
############################################################################## 
sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-aircraft =>  [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, undef],
		    -zinc => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, undef],
		    -parent_grp => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, undef],
		    -origin     => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -width      => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -height     => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -selected   => [S_NOINIT,   S_METHOD,  S_RDWR, S_OVRWRT, S_NOPRPG, 0],
		   );
}

# completeinit:
#   this sub complete the init of subject object
##############################################################################
sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  #$self->attach_to_aircraft();

  my $zinc = $self->get(-zinc);
#  my $ident = $self->get(-ident);
  my $flight_plan = "foo"; #$self->get(-flight_plan);
#  $self->{zinc} = $self->get(-zinc);
#  $self->{fp} = $flight_plan;
 
  $self->{modes} =
    {
#     ap_mode => 
#     { name => ["Manual", "Auto1", "Auto2", "Home"],
#       color => ["sienna", "blue", "brown", "red"]
#     },
#      gps_mode =>
#      { name => [ "No fix", "GPS dead reckoning only", "2D-fix",  "3D-fix", "GPS + dead reckoning combined"],
#        color => ["red", "red", "orange", "brown", "orange"]
#      },
#     rc_status => 
#     { name => ["Ok","Lost", "Really lost", "error"],
#       color => ["orange", "brown", "red", "red"]
#     },
#     contrast_status => 
#     { name => ["Default","Waiting", "Set", "error"],
#       color => ["orange", "brown", "red", "red"]
#     }
    };

  $self->{new_modes} = 
    {
     ap_mode => { 'MANUAL' => 'sienna', 'AUTO1' => 'blue', 'AUTO2' => 'brown', 'HOME' => 'red'},
     rc_status => {'OK' => 'brown', 'LOST' => 'orange', 'REALLY_LOST' => 'red'},
     rc_mode   => {'AUTO'=> 'orange', 'MANUAL' => 'brown', 'FAILSAFE' => 'red'},
     contrast_status => {'DEFAULT' => 'orange', 'WAITING' => 'brown', 'SET' => 'green'},
     gps_mode => {'NOFIX' => 'red', 'DRO' => 'red', '2D' => 'orange', '3D' => 'brown', 'GPSDRO' => 'red'},
    };

  $self->{frame} = undef;
  $self->{frame_clip} = undef;

  $self->{battery} = 12.5;
  $self->{topgroup} = undef;
  $self->{contentgroup} = undef;
  $self->{mission} = undef;

  $self->{options} = {
    normal_font => undef,
    small_font => undef,
    background_color => undef,
    border_color => undef,
    label_color => undef,
  };

  $self->parse_config();


  $self->{prefix} = "STRIP_".$self->get(-aircraft)->get('-ac_id')."_";
  $self->{zinc_bat} = undef;
  $self->{zinc_bat_value} = undef;
  $self->attach_to_aircraft();
  $self->draw();

}

sub parse_config {
  my ($self) = @_;
  my $parser = XML::DOM::Parser->new();
  print "Parsing gui.xml\n";
  my $doc = $parser->parsefile(Paparazzi::Environment::get_config("gui.xml"));
  my $strip = $doc->getElementsByTagName('strip')->[0];
  foreach my $attr ('selected_background_color', 'background_color', 'normal_font',
		    'small_font', 'border_color', 'label_color', 'value_color') {
    $self->{options}->{$attr} =  $strip->getAttribute($attr);
  }
}



# draw
#   draw elements of the strip
##############################################################################
sub draw {
  my $self = shift;
  my $zinc = $self->get(-zinc);
  my $ident = $self->get(-aircraft)->get('-ac_id');
  my ($x, $y) = @{$self->get('-origin')};
#  my $fp = $self->get(-flight_plan);
  
  my $width = 300;


  ## main group of the strip
  $self->{topgroup} = $zinc->add('group', scalar $self->get('-parent_grp'), -sensitive => 1,
  	-atomic => 0, -tags => ["strip_".$ident]);
  $self->{contentgroup} = $zinc->add('group', $self->{topgroup}, -composealpha=>0, -sensitive=>1);

  ## the strip
  $self->{frame} = $zinc->add('rectangle', $self->{topgroup}, [5,5,$width, 95], -fillcolor => $self->{options}->{background_color}, -filled=>1, -linecolor => $self->{options}->{border_color}, -sensitive => 1, -tags => ["strip_".$ident]);
  # cliping contour of the strip
  $self->{frame_clip} = $zinc->add('rectangle', $self->{contentgroup}, [5,5,$width, 95], -visible=>0);
  $zinc->itemconfigure($self->{contentgroup}, -clip=> $self->{frame_clip});
  
  ## bindings to highlight the strip when the mouse is over
  $zinc->bind($self->{frame}, '<Enter>', sub { $zinc->itemconfigure($self->{frame}, -linecolor => 'red'); });
  $zinc->bind($self->{frame}, '<Leave>', sub { $zinc->itemconfigure($self->{frame}, -linecolor => 'sienna'); });
  $zinc->bind($self->{frame}, '<1>', [\&onStripPressed, $self, $ident]);

  ## ident of the plane
  $self->{ident} = $zinc->add('text', $self->{contentgroup}, -text => uc($ident), -position=>[10,10], -font => $self->{options}->{normal_font}, -color => "midnightblue");
  
  my @label_attr = (['AP',  'ap_mode',          70,  10],
		    ['RC',  'rc_status',        70,  22],
		    ['GPS', 'gps_mode',         70,  34],
		    ['Cal', 'contrast_status',  70,  46],
		    ['Ctrst', 'contrast_value', 70,  58],
		    ['alt:',  'alt',            150, 10],
		    ['desired:','target_alt',   150, 22],
		    ['speed:',  'speed',        150, 46],
		    ['climb:',  'climb',        150, 58],
		   );
  foreach my $attr (@label_attr) {
    $self->add_label($attr->[0], $attr->[1], $attr->[2], $attr->[3]);
    $self->add_value_text($attr->[1]);
  }

  $zinc->add('text', $self->{contentgroup}, -text => $self->string_of_time(0), -position => [8, 82], -font => $self->{options}->{small_font}, -color => $self->{options}->{label_color}, -tags => [ $self->{prefix}."flight_time_value"] );
  ##
  # QUICK and DIRTY flight time an battery
  # 
  ## flight_time
  # FIXME: use tags instead of reference to zinc item

  $zinc->add('rectangle', $self->{contentgroup}, [10,25, 41,81], -filled=>1, -fillcolor=> '#d1d1d1');
  $self->{zinc_bat} = $zinc->add('rectangle', $self->{contentgroup}, [11,80-(($self->{battery}-6)/7)*55,40,80],  -filled=>1,-fillcolor=>'#8080ff', -linewidth=>0);

  $self->{zinc_bat_value} = $zinc->add('text', $self->{contentgroup}, -text => sprintf("%s",$self->{battery}), -position=>[12,40], -font => $self->{options}->{small_font});

  $zinc->translate($self->{topgroup}, $x, $y);
  $zinc->raise($self->{topgroup});
  $zinc->raise($self->{contentgroup});
#  $self->border_block(); # display blocks of flight plan
  return $self->{topgroup};
}

# add_label
#   adding a label
##############################################################################
sub add_label {
  my ($self, $label, $tag, $x, $y) = @_;
#  print "adding $tag\n";
  $self->get('-zinc')->add('text',$self->{contentgroup}, -text => $label, -position => [$x, $y], -font => $self->{options}->{normal_font}, -color => $self->{options}->{label_color}, -tags => [ $self->{prefix}.$tag ]);
}

# add_value_text
#   adding zinc item for value with the tag: tag_value
##############################################################################
sub add_value_text {
  my ($self, $tag) = @_;
  my $zinc = $self->get('-zinc');
  my $item = $zinc->find('withtag', $self->{prefix}.$tag);
  my $new_tag = $self->{prefix}.$tag."_value";
  my ($xo,$yo, $xc, $yc) = $zinc->bbox($item);
  $zinc->add('text', $self->{contentgroup}, -text =>  "N/A",
	     -position => [$xc+5, $yo], -font => $self->{options}->{normal_font}, 
	     -color => $self->{options}->{value_color}, -tags => [ $new_tag ]);
}

# parse_fp
#   Quick and dirty sub to parse a flight plan
# FIXME:
##############################################################################
sub parse_fp {
  my $self = shift;
  my ($fp) = @_;
  my $parser = XML::DOM::Parser->new();
  print "Parsing $fp\n";
  my $doc = $parser->parsefile($fp);
  my @result;
  my $flight_plan = $doc->getElementsByTagName('flight_plan')->[0];
  foreach my $block ($doc->getElementsByTagName('block')) {

    my @exceptions = $block->getElementsByTagName('exception');
    my $rc_events = {rc1=>"", rc2=>""};
    foreach my $event (@exceptions) {
      if($event->getAttribute('cond') eq "(RcEvent1())") {
	$rc_events->{rc1} = $event->getAttribute('deroute');
      }
      if($event->getAttribute('cond') eq "(RcEvent2())") {
	$rc_events->{rc2} = $event->getAttribute('deroute');
      }
    }
    push(@result, { name => $block->getAttribute('name'),
    	description => $block->getAttribute('description'),
	rc1 => $rc_events->{rc1},
	rc2 => $rc_events->{rc2}});
  }
  return @result;
}

# border_block
#   add the flight plans block to the strip
##############################################################################
sub border_block {
  my ($self) = @_;
  my $flight_plan_url = $self->get('-aircraft')->get('flight_plan');
  $flight_plan_url =~ /file:\/\/(.*)/;
  my $flight_plan = $1;
  print "in Strip border_block parsing $flight_plan\n";

  #my @blocks = $self->parse_fp($flight_plan);
  my @blocks;
  my @groups = ();
  my @x = ( 300, 350, 400, 450, 500, 550, 600, 650, 700, 750);

  my $zinc = $self->get('-zinc');

  $zinc->add('text', $self->{contentgroup}, -text => "Event1",
		     -position => [ 260, 50], -font => $self->{options}->{small_font},
		     -color => $self->{options}->{label_color}
		    );

  $zinc->add('text', $self->{contentgroup}, -text => "Event2",
		     -position => [ 260, 60], -font => $self->{options}->{small_font},
		     -color => $self->{options}->{label_color}
		    );
  
  my $i;
  for ($i=0; $i<10; $i++) {
    my $block_name = uc($blocks[$i]->{name});
    $zinc->add('curve', $self->{contentgroup}, [$x[$i], 10, $x[$i], 90]);
    push(@groups, $zinc->add('group', $self->{contentgroup}));
    my $clip = $zinc->add('rectangle', $groups[$i], [$x[$i], 10, $x[$i]+46, 90], -visible => 0);
    $zinc->itemconfigure($groups[$i], -clip => $clip);
    $zinc->add('text', $groups[$i], -text => $block_name,
	       -position => [ $x[$i]+3, 20], -font => $self->{options}->{normal_font},
	       -tags => [ $self->{prefix}."block_".$block_name, "block_label", $self->{prefix}."block_num_".$i ]
	      );
    $zinc->add('text', $groups[$i], -text => $blocks[$i]->{rc1},
	       -position => [ $x[$i]+3, 50], -font => $self->{options}->{small_font},
	       -color => $self->{options}->{value_color}
	      );
    $zinc->add('text', $groups[$i], -text => $blocks[$i]->{rc2},
	       -position => [ $x[$i]+3, 60], -font => $self->{options}->{small_font},
	       -color => $self->{options}->{value_color}
	      );

  }
}

# set and get methods
###############################################################################

sub set_item {
  my ($self, $item_name, $string, $color) = @_;
#  print "in Strip::set_item $item_name $string $color ($self->{prefix})\n";
  my $zinc = $self->get('-zinc');
  my $item = $zinc->find('withtag', $self->{prefix}.$item_name."_value");
  print "in Strip::set_item $item_name $string color $color\n";
  $zinc->itemconfigure($item, -text => $string, -color  => $color);
}

sub set_block {
  my ($self, $num) = @_;
  my $zinc = $self->get('-zinc');
  foreach my $b ($zinc->find('withtag', $self->{prefix}."block_label")) {
    $zinc->itemconfigure($b, -color => $self->{options}->{label_color});
  }
  my $item = $zinc->find('withtag', $self->{prefix}."block_num_".$num);
  $zinc->itemconfigure($item, -color => 'blue');
}


# FIXME: should be deprecated and we should use set_item or something like that
sub set_bat {
  my ($self, $bat) = @_;
#  print "in Strip::set_bat $bat ($self->{prefix})\n";
  my $zinc = $self->get('-zinc');
  $self->{battery} = $bat;
  my $batcolor = '#8080ff';
  $zinc->remove($self->{zinc_bat}) if (defined $self->{zinc_bat});
  $self->{zinc_bat} = $zinc->add('rectangle',
				 $self->{contentgroup},
				 [11,80-(($self->{battery}-6)/7)*55,40,80],
				 -filled=>1, -fillcolor=>$batcolor, -linewidth=>0) if (defined $self->{contentgroup});
  $zinc->itemconfigure($self->{zinc_bat_value},
		       -text => sprintf("%2.1f",$self->{battery}),);
  $zinc->raise($self->{zinc_bat_value});
}

sub string_of_time {
  my ($self, $t) = @_;
  my $hour = int($t/3600);
  my $min = int(($t-$hour*3600)/60);
  my $sec = $t-(3600*$hour)-($min*60);
  sprintf("%02d:%02d:%02d",$hour, $min, $sec);
}

# attach_to_aircraft
#   bla bla
##############################################################################
sub attach_to_aircraft {
  my ($self) = @_;
  my @options = ('airframe', 'flight_plan', 'ap_mode', 'rc_status', 'gps_mode', 'contrast_status', 'contrast_value',
		 'flight_time', 'alt', 'target_alt', 'speed', 'climb', '-engine_status');
  foreach my $option (@options) {
    $self->get('-aircraft')->attach($self, $option, [\&aircraft_config_changed]);
  }
}


sub aircraft_config_changed {
  my ($self, $aircraft, $event, $new_value) = @_;
#  print "in strip aircraft_config_changed $event $new_value\n";
  return unless defined $new_value;
  if ($event eq 'flight_plan') {
    #    $self->border_block() if (defined $new_value) ; # display blocks of flight plan
  }
  elsif ($event eq 'airframe') {
    $self->get('-zinc')->itemconfigure($self->{ident}, -text => scalar $new_value->get('-name')) if defined $new_value;
  }
  elsif ($event eq 'rc_status' or $event eq 'rc_mode' or $event eq 'contrast_status' or $event eq 'ap_mode' or $event eq 'gps_mode') {
    my $color = $self->{new_modes}->{$event}->{$new_value};
    $self->set_item($event, $new_value, $color);
  }
  elsif ($event eq 'flight_time') {
    $self->set_item("flight_time",$self->string_of_time($new_value), $self->{options}->{value_color});
  }
  elsif ($event eq '-engine_status') {
#    Paparazzi::Traces::trace( Paparazzi::Traces::TRACE_DEBUG, "in Strip::aircraft_config_changed\n".Dumper($new_value));
    $self->set_bat($new_value->{bat});
  }
  elsif ( $event eq 'speed' or $event eq 'climb' or $event eq 'alt' or $event eq 'target_alt' or $event eq 'contrast_value') {
    my $fmt = { speed => "%2.1fm/s",
		climb => "%+2.1fm/s",
		alt   => "%4.1fm",
		target_alt => "%4.1fm",
		contrast_value => "%03d",
	      };
    $self->set_item($event, sprintf($fmt->{$event}, $new_value), $self->{options}->{value_color});

  }
  # cur_block
  elsif ($event eq 'cur_block') {
    $self->set_block($new_value); # display current block of flight plan
  }

  else {
    print "in Strip::aircraft_config_changed : unknow event $event $new_value\n";
  }

}

sub selected {
  my ($self, $previous_val, $new_val) = @_;
  my $zinc = $self->get('-zinc');
  $zinc->itemconfigure($self->{frame},
		       -fillcolor => $new_val ? $self->{options}->{selected_background_color} :
		       $self->{options}->{background_color});
}

## fin de la classe
1;
