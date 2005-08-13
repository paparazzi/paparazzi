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


package PapaStrip::Strip;

BEGIN {
  our $paparazzi_home = $ENV{PAPARAZZI_HOME};
};
use lib ('/usr/share/perl5',
	 $paparazzi_home.'/sw/lib/perl', 
	 $paparazzi_home.'/sw/ground_segment/papastar');


use Subject;
@ISA=("Subject");

#use Ivy;
#use IvyMsgs;

use Tk;
use Tk::Zinc;

use XML::DOM;
use Data::Dumper;

use strict;
use warnings;

# populate:
#   this sub is the subject constructor method
############################################################################## 
sub populate {

  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, undef],
		    -mw => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -ident => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -pos_x => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -pos_y => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -flight_plan => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],		    

		   );
}

# completeinit:
#   this sub complete the init of subject object
##############################################################################
sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  my $mw = $self->get(-mw);
  my $zinc = $self->get(-zinc);
  my $ident = $self->get(-ident);
  my $pos_x = $self->get(-pos_x);
  my $pos_y = $self->get(-pos_y);
  my $flight_plan = $self->get(-flight_plan);
  $self->{'zinc'} = $self->get(-zinc);
  $self->{fp} = $flight_plan;
  
  $self->{modes} =
    { ap_mode => 
      { name => ["Manual", "Auto1", "Auto2", "Home"],
	color => ["sienna", "blue", "brown", "red"]
      },
      gps_mode =>
      { name => [ "No fix", "GPS dead reckoning only", "2D-fix",  "3D-fix", "GPS + dead reckoning combined"],
	color => ["red", "red", "orange", "brown", "orange"]
      },
      rc_status => 
      { name => ["Lost","Ok", "Really lost", "error"],
	color => ["orange", "brown", "red", "red"]
      }
    };

  $self->{'frame'} = undef;
  $self->{'frame_clip'} = undef;

  $self->{'battery'} = 12.5;
  $self->{'topgroup'} = undef;
  $self->{'contentgroup'} = undef;
  $self->{mission} = undef;

  $self->{options} = {
    normal_font => undef,
    small_font => undef,
    background_color => undef,
    border_color => undef,
    label_color => undef,
  };

  $self->parse_config();

  $self->{'sdv'} = 0;
  $self->{'zinc_flight_time'} = "";
  $self->{'zinc_bat'} = "";
  $self->{'zinc_bat_value'} = "";
}


sub parse_config {
  my ($self) = @_;
  my $parser = XML::DOM::Parser->new();
  print "Parsing gui.xml\n";
  my $doc = $parser->parsefile("/home/drac/paparazzi2/conf/gui.xml");
  my $strip = $doc->getElementsByTagName('strip')->[0];
  $self->{options}->{normal_font} =  $strip->getAttribute('normal_font');
  $self->{options}->{small_font} =  $strip->getAttribute('small_font');
  $self->{options}->{background_color} =  $strip->getAttribute('background_color');
  $self->{options}->{border_color} =  $strip->getAttribute('border_color');
  $self->{options}->{label_color} = $strip->getAttribute('label_color');
  $self->{options}->{value_color} = $strip->getAttribute('value_color');
}



# draw
#   draw elements of the strip
##############################################################################
sub draw {
  my $self = shift;
  my $zinc = $self->get(-zinc);
  my $ident = $self->get(-ident);
  my $x = $self->get(-pos_x);
  my $y = $self->get(-pos_y);
  my $fp = $self->get(-flight_plan);

  ## main group of the strip
  $self->{'topgroup'} = $zinc->add('group', 1, -sensitive => 1, 
  	-atomic => 0, -tags => ["strip_".$ident]);
  $self->{'contentgroup'} = $zinc->add('group', $self->{'topgroup'}, -composealpha=>0, -sensitive=>1);

  ## the strip
  $self->{'frame'} = $zinc->add('rectangle', $self->{'topgroup'}, [5,5,795, 95], -fillcolor => $self->{options}->{background_color}, -filled=>1, -linecolor => $self->{options}->{border_color}, -sensitive => 1, -tags => ["strip_".$ident]);
  # cliping contour of the strip
  $self->{'frame_clip'} = $zinc->add('rectangle', $self->{'contentgroup'}, [5,5,795, 95], -visible=>0);
  $zinc->itemconfigure($self->{'contentgroup'}, -clip=> $self->{'frame_clip'});
  
  ## bindings to highlight the strip when the mouse is over
  $zinc->bind($self->{frame}, '<Enter>', sub { $zinc->itemconfigure($self->{frame}, -linecolor => 'red'); });
  $zinc->bind($self->{frame}, '<Leave>', sub { $zinc->itemconfigure($self->{frame}, -linecolor => 'sienna'); });

  ## ident of the plane
  $zinc->add('text', $self->{'contentgroup'}, -text => uc($ident), -position=>[10,10], -font => $self->{options}->{normal_font}, -color => "midnightblue");
  
  ## AutoPilot label and value
  $self->add_label("AP", "ap_mode", 70, 10);
  $self->add_value_text("ap_mode");

  ## RC Status label and value
  $self->add_label("RC", "rc_status", 70, 22);
  $self->add_value_text("rc_status");

  ## GPS Fix label and value
  $self->add_label("GPS", "gps_mode", 70, 34);
  $self->add_value_text("gps_mode");

	
  ## alt label and value
  $self->add_label("alt:","alt", 150, 10);
  $self->add_value_text("alt");

  ## desired alt label and value
  $self->add_label("desired:","desired_alt", 150, 22);
  $self->add_value_text("desired_alt");

  ## speed label and value
  $self->add_label("speed:", "speed", 150, 46);
  $self->add_value_text("speed");

  ## climb label and value
  $self->add_label("climb:", "climb", 150, 58);
  $self->add_value_text("climb");

  ##
  # QUICK and DIRTY flight time an battery
  # 
  ## flight_time
  # FIXME: use tags instead of reference to zinc item
  $self->{'zinc_flight_time'} = $zinc->add('text', $self->{'contentgroup'}, -text => sprintf("%s",$self->string_of_time(0)), -position=>[8,82], -font => $self->{options}->{small_font});

  $zinc->add('rectangle', $self->{'contentgroup'}, [10,25, 41,81], -filled=>1, -fillcolor=> '#d1d1d1');
  $self->{'zinc_bat'} = $zinc->add('rectangle', $self->{'contentgroup'}, [11,80-(($self->{'battery'}-6)/7)*55,40,80],  -filled=>1,-fillcolor=>'#8080ff', -linewidth=>0);

  $self->{'zinc_bat_value'} = $zinc->add('text', $self->{'contentgroup'}, -text => sprintf("%s",$self->{'battery'}), -position=>[12,40], -font => $self->{options}->{small_font});

  $zinc->translate($self->{'topgroup'}, $x, $y);
  $zinc->raise($self->{'topgroup'});
  $zinc->raise($self->{'contentgroup'});
  $self->border_block(); # display blocks of flight plan
  return $self->{'topgroup'};
}

# add_label
#   adding a label
##############################################################################
sub add_label {
  my ($self, $label, $tag, $x, $y) = @_;
  print "adding $tag\n";
  $self->{zinc}->add('text',$self->{'contentgroup'}, -text => $label, -position => [$x, $y], -font => $self->{options}->{normal_font}, -color => $self->{options}->{label_color}, -tags => [ $tag ]);
}

# add_value_text
#   adding zinc item for value with the tag: tag_value
##############################################################################
sub add_value_text {
  my ($self, $tag) = @_;
  my $item = $self->{zinc}->find('withtag', $tag);
  my $new_tag = $tag."_value";
  my ($xo,$yo, $xc, $yc) = $self->{zinc}->bbox($item);
  $self->{zinc}->add('text', $self->{contentgroup}, -text =>  "N/A",
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
  my @blocks = $self->parse_fp($self->{fp});
  my @groups = ();
  my @x = ( 300, 350, 400, 450, 500, 550, 600, 650, 700, 750);
  $self->{zinc}->add('text', $self->{contentgroup}, -text => "Event1",
  -position => [ 260, 50], -font => $self->{options}->{small_font},
  -color => $self->{options}->{label_color}
);
$self->{zinc}->add('text', $self->{contentgroup}, -text => "Event2",
-position => [ 260, 60], -font => $self->{options}->{small_font},
-color => $self->{options}->{label_color}
  );

  my $i;
  for ($i=0; $i<10; $i++) {
    my $block_name = uc($blocks[$i]->{name});
    $self->{zinc}->add('curve', $self->{contentgroup}, [$x[$i], 10, $x[$i], 90]);
    push(@groups, $self->{zinc}->add('group', $self->{contentgroup}));
    my $clip = $self->{zinc}->add('rectangle', $groups[$i], [$x[$i], 10, $x[$i]+46, 90], -visible => 0);
    $self->{zinc}->itemconfigure($groups[$i], -clip => $clip);
    $self->{zinc}->add('text', $groups[$i], -text => $block_name,
    -position => [ $x[$i]+3, 20], -font => $self->{options}->{normal_font},
    -tags => [ "block_".$block_name, "block_label", "block_num_".$i ]
  );
  $self->{zinc}->add('text', $groups[$i], -text => $blocks[$i]->{rc1},
  -position => [ $x[$i]+3, 50], -font => $self->{options}->{small_font},
  -color => $self->{options}->{value_color}
);
$self->{zinc}->add('text', $groups[$i], -text => $blocks[$i]->{rc2},
-position => [ $x[$i]+3, 60], -font => $self->{options}->{small_font},
-color => $self->{options}->{value_color}
    );

  }
}

# set and get methods
###############################################################################

sub set_item {
  my ($self, $item_name, $string, $color) = @_;
  my $item = $self->{zinc}->find('withtag', $item_name."_value");
  $self->{zinc}->itemconfigure($item, -text => $string, -color  => $color);
}

sub set_block {
  my ($self, $num) = @_;
  print Dumper($self->{zinc}->find('withtag', "block_label"));
  foreach my $b ($self->{zinc}->find('withtag', "block_label")) {
    $self->{zinc}->itemconfigure($b, -color => $self->{options}->{label_color});
  }
  my $item = $self->{zinc}->find('withtag', "block_num_".$num);
  $self->{zinc}->itemconfigure($item, -color => 'blue');
}


# FIXME: should be deprecated and we should use set_item or something like that
sub setBat {
  my $self = shift;
  my ($bat) = @_;
  $self->{'battery'} = $bat;
  my $batcolor = '#8080ff';
  $self->{'zinc'}->remove($self->{'zinc_bat'});
  $self->{'zinc_bat'} = 
  $self->{'zinc'}->add('rectangle',
  $self->{'contentgroup'}, [11,80-(($self->{'battery'}-6)/7)*55,40,80],
  -filled=>1,-fillcolor=>$batcolor, -linewidth=>0);
  $self->{'zinc'}->itemconfigure($self->{'zinc_bat_value'},
  -text => sprintf("%2.1f",$self->{'battery'}),);
  $self->{'zinc'}->raise($self->{'zinc_bat_value'});
}

# FIXME: should use tags and set_item!!!
sub setSDV {
  my $self = shift;
  my ($sdv) = @_;
  $self->{'sdv'} = $sdv;
  $self->{'zinc'}->itemconfigure(
    $self->{'zinc_flight_time'},
    -text => sprintf("%s",$self->string_of_time($sdv)),
    -color => $self->{options}->{label_color}
  );
}

sub string_of_time {
  my ($self, $t) = @_;
  my $hour = int($t/3600);
  my $min = int(($t-$hour*3600)/60);
  my $sec = $t-(3600*$hour)-($min*60);
  sprintf("%02d:%02d:%02d",$hour, $min, $sec);
}

## fin de la classe
1;
