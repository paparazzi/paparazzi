#
# $Id$
#
# Paparazzi::Strip (Aircraft flight informations)
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

use Data::Dumper;

use strict;
use warnings;

use Paparazzi::Traces;
use Paparazzi::Utils;
use Paparazzi::GuiConfig;
use Paparazzi::SimpleLinearGauge;


my $modes_data = 
    {
     ap_mode => { 'MANUAL' => 'sienna', 'AUTO1' => 'blue', 'AUTO2' => 'brown', 'HOME' => 'red'},
     rc_status => {'OK' => 'brown', 'LOST' => 'orange', 'REALLY_LOST' => 'red'},
     rc_mode   => {'AUTO'=> 'orange', 'MANUAL' => 'brown', 'FAILSAFE' => 'red'},
     contrast_status => {'DEFAULT' => 'orange', 'WAITING' => 'brown', 'SET' => 'green'},
     gps_mode => {'NOFIX' => 'red', 'DRO' => 'red', '2D' => 'orange', '3D' => 'brown', 'GPSDRO' => 'red'},
    };

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-aircraft =>  [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, undef],
		    -zinc => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, undef],
		    -parent_grp => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, undef],
		    -origin     => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -width      => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -height     => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -selected   => [S_NOINIT,   S_METHOD,  S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();

  my $zinc = $self->get(-zinc);

  $self->{frame} = undef;
  $self->{frame_clip} = undef;

  $self->{battery} = 12.5;
  $self->{topgroup} = undef;
  $self->{contentgroup} = undef;
  $self->{mission} = undef;

  $self->{options} = {
    normal_font => Paparazzi::GuiConfig::get_resource('strip', 'normal_font'),
    small_font => Paparazzi::GuiConfig::get_resource('strip', 'small_font'),
    background_color => Paparazzi::GuiConfig::get_resource('strip', 'background_color'),
    selected_background_color => Paparazzi::GuiConfig::get_resource('strip', 'selected_background_color'),
    border_color => Paparazzi::GuiConfig::get_resource('strip', 'border_color'),
    label_color => Paparazzi::GuiConfig::get_resource('strip', 'label_color'),
    value_color => Paparazzi::GuiConfig::get_resource('strip', 'value_color'),
 };

  $self->{prefix} = "STRIP_".$self->get(-aircraft)->get('-ac_id')."_";
  $self->{zinc_bat} = undef;
  $self->{zinc_bat_value} = undef;
  $self->attach_to_aircraft();
  $self->draw();
}

# draw
#   draw elements of the strip
##############################################################################

use constant MARGIN => 3;
sub draw {
  my $self = shift;
  my $zinc = $self->get(-zinc);
  my $ident = $self->get(-aircraft)->get('-ac_id');
  my ($x, $y) = @{$self->get('-origin')};
  my $width = $self->get('-width');
  my $height = $self->get('-height');

  ## main group of the strip
  $self->{topgroup} = $zinc->add('group', scalar $self->get('-parent_grp'), -sensitive => 1,
  	-atomic => 0, -tags => ["strip_".$ident]);
  $self->{contentgroup} = $zinc->add('group', $self->{topgroup}, -composealpha=>0, -sensitive=>1);

  ## the strip
  my ($actual_width, $actual_height) = ($width - 2 * MARGIN, $height - 2 * MARGIN);
  my $strip_rect= [MARGIN, MARGIN, $actual_width , $actual_height];
  $self->{frame} = $zinc->add('rectangle', $self->{topgroup},
			      , $strip_rect,
			      -fillcolor => $self->{options}->{background_color},
			      -filled=>1, -linecolor => $self->{options}->{border_color},
			      -sensitive => 1,
			      -tags => ["strip_".$ident]
			     );
  # cliping contour of the strip
  $self->{frame_clip} = $zinc->add('rectangle', $self->{contentgroup}, 
				   $strip_rect, -visible=>0);
  $zinc->itemconfigure($self->{contentgroup}, -clip=> $self->{frame_clip});
  
  ## bindings to highlight the strip when the mouse is over
  $zinc->bind($self->{frame}, '<Enter>', sub { $zinc->itemconfigure($self->{frame}, -linecolor => 'red'); });
  $zinc->bind($self->{frame}, '<Leave>', sub { $zinc->itemconfigure($self->{frame}, -linecolor => 'sienna'); });
  $zinc->bind($self->{frame}, '<1>', [\&onStripPressed, $self, $ident]);

  my @col_x = ($actual_width * 0.03, $actual_width * 0.23, $actual_width * 0.53);
  my @row_y = ($actual_height * 0.03, $actual_height * 0.23, $actual_height * 0.36,
	       $actual_height * 0.48, $actual_height * 0.61, $actual_height * 0.74);

  ## ident of the plane
  $self->{ident} = $zinc->add('text', $self->{contentgroup}, -text => uc($ident),
			      -position=>[$col_x[0],$row_y[0]], -font => $self->{options}->{normal_font},
			      -color => "midnightblue");
  
  my @label_attr = (['AP',  'ap_mode',          $col_x[1], $row_y[0]],
		    ['RC',  'rc_status',        $col_x[1], $row_y[1]],
		    ['GPS', 'gps_mode',         $col_x[1], $row_y[2]],
		    ['Wind','dir',              $col_x[1], $row_y[3]],
		    ['     ', 'wspeed',         $col_x[1], $row_y[4]],
		    ['Mas', 'mean_aspeed',      $col_x[1], $row_y[5]],
		    ['alt:',  'alt',            $col_x[2], $row_y[0]],
		    ['desired:','target_alt',   $col_x[2], $row_y[1]],
		    ['throttle:',  'throttle',  $col_x[2], $row_y[2]],
		    ['speed:',  'speed',        $col_x[2], $row_y[3]],
		    ['climb:',  'climb',        $col_x[2], $row_y[4]],
		   );
  foreach my $attr (@label_attr) {
    $self->add_label($attr->[0], $attr->[1], $attr->[2], $attr->[3]);
    $self->add_value_text($attr->[1]);
  }

  $zinc->add('text', $self->{contentgroup}, -text => Utils::hhmmss_of_s(0), -position => [8, 82], -font => $self->{options}->{small_font}, -color => $self->{options}->{label_color}, -tags => [ $self->{prefix}."flight_time_value"] );
  ##
  # QUICK and DIRTY flight time an battery
  # 
  ## flight_time
  # FIXME: use tags instead of reference to zinc item

  $zinc->add('rectangle', $self->{contentgroup}, [10,25, 41,81], -filled=>1, -fillcolor=> '#d1d1d1');
  $self->{zinc_bat} = $zinc->add('rectangle', $self->{contentgroup}, [11,80-(($self->{battery}-6)/7)*55,40,80],  -filled=>1,-fillcolor=>'#8080ff', -linewidth=>0);

  $self->{zinc_bat_value} = $zinc->add('text', $self->{contentgroup}, -text => sprintf("%s",$self->{battery}), -position=>[12,40], -font => $self->{options}->{small_font});


#  my $bat_gauge = Paparazzi::SimpleLinearGauge->new(-zinc => $zinc, -parent_grp => $self->{contentgroup},
#						    -origin => [10,25], -width => 30, -height => 60 );
  

  $zinc->translate($self->{topgroup}, $x, $y);
  $zinc->raise($self->{topgroup});
  $zinc->raise($self->{contentgroup});

  

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


# set and get methods
###############################################################################

sub set_item {
  my ($self, $item_name, $string, $color) = @_;
#  print "in Strip::set_item $item_name $string $color ($self->{prefix})\n";
  my $zinc = $self->get('-zinc');
  my $item = $zinc->find('withtag', $self->{prefix}.$item_name."_value");
#  print "in Strip::set_item $item_name $string color $color\n";
  $zinc->itemconfigure($item, -text => $string, -color  => $color);
}

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

# attach_to_aircraft
#   bla bla
##############################################################################
sub attach_to_aircraft {
  my ($self) = @_;
  my @options = ('airframe', 'ap_mode', 'rc_status', 'gps_mode', 'wspeed', 'dir', 'mean_aspeed',
		 'flight_time', 'alt', 'target_alt', 'speed', 'climb', '-engine_status');
  foreach my $option (@options) {
    $self->get('-aircraft')->attach($self, $option, [\&aircraft_config_changed]);
  }
}

sub get_color {
  my ($self, $mode, $value) = @_;
  if (defined $modes_data->{$mode}->{$value}) {
    return $modes_data->{$mode}->{$value};
  }
  else {
    return 'black';
  }
}

sub aircraft_config_changed {
  my ($self, $aircraft, $event, $new_value) = @_;
#  print "in strip aircraft_config_changed $event $new_value\n";
  return unless defined $new_value;
  if ($event eq 'airframe') {
    my $ac_name = $new_value->get('-ac_name');
    $self->get('-zinc')->itemconfigure($self->{ident}, -text => $ac_name ) if defined $ac_name;
  }
  elsif ($event eq 'rc_status' or $event eq 'rc_mode' or $event eq 'ap_mode' or $event eq 'gps_mode') {
    $self->set_item($event, $new_value, $self->get_color($event, $new_value));
  }
  elsif ($event eq 'flight_time') {
    $self->set_item("flight_time",Utils::hhmmss_of_s($new_value), $self->{options}->{value_color});
  }
  elsif ($event eq '-engine_status') {
#    Paparazzi::Traces::trace( Paparazzi::Traces::TRACE_DEBUG, "in Strip::aircraft_config_changed\n".Dumper($new_value));
    $self->set_bat($new_value->{bat});
    $self->set_item('throttle', $new_value->{throttle}, 'black');
  }
  elsif ( $event eq 'speed' or $event eq 'climb' or $event eq 'alt' or $event eq 'target_alt' or $event eq 'wspeed' or $event eq 'dir' or $event eq 'mean_aspeed') {
    my $fmt = { speed => "%2.1fm/s",
		climb => "%+2.1fm/s",
		alt   => "%4.1fm",
		target_alt => "%4.1fm",
		wspeed => "%.1fm/s",
		dir    => "%03d°",
		mean_aspeed => "%.1fm/s",
	      };
    $self->set_item($event, sprintf($fmt->{$event}, $new_value), $self->{options}->{value_color});
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
