#
# $Id$
#
# Paparazzi::Alert (Aircrafts alert information)
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

package Paparazzi::Alert;
use Subject;
@ISA=("Subject");

use Tk;
use Tk::Zinc;

use XML::DOM;
use Data::Dumper;

use strict;
use warnings;

use Paparazzi::Traces;
use Paparazzi::Utils;
use Paparazzi::GuiConfig;
use Paparazzi::IvyProtocol;
use Paparazzi::Environment;
use Paparazzi::AircraftsManager;


my @paparazzi_lib;
BEGIN {
  @paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
    ($ENV{PAPARAZZI_SRC}."/sw/lib/perl", $ENV{PAPARAZZI_SRC}."/sw/ground_segment/cockpit"):();
}
use lib (@paparazzi_lib);


sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(
    -zinc	=> [S_NEEDINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, undef],
    -height	=> [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, 400],
    -width	=> [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, 400],
    -topgroup	=> [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, 1],
    -font       => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, "bleriot-radar-m14c"],
    -step       => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, 20],
    -color      => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, "=axial -90 |red |orange;6"],
    -colorack   => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_SUPER, "=axial -90 |red |orange;6"],
  );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  my $zinc = $self->get(-zinc);
  my $topgroup = $self->get(-topgroup);
  $self->{group} = $zinc->add('group', $topgroup);
  my $protocol_file = Paparazzi::Environment::get_config("messages.xml");
  my $options = 
    {
     ivy_bus  => "127.255.255.255:2010",
     render => 1,
     tracelevel => 1,
    };
  Paparazzi::Environment::parse_command_line($options);
  Paparazzi::IvyProtocol::init(
    -file	=> $protocol_file,
    -ivy_bus	=> $options->{ivy_bus},
    -app_name	=> "Alert",
    -loop_mode	=> 'TK',
  );
  print $protocol_file."\n";
  $self->read_protocol("alert", $protocol_file);
  $self->{aircrafts_manager} = 
    Paparazzi::AircraftsManager->new(
				     -listen_to_all => ['FLIGHT_PARAM', 'AP_STATUS', 'NAV_STATUS', 'CAM_STATUS', 'ENGINE_STATUS', 'FLY_BY_WIRE', 'INFRARED', 'INFLIGH_CALIB', 'SVSINFO', 'WIND']
				    );
}

# sub to read all msg from message.xml
sub read_protocol {
  my ($self, $classname, $filename) = @_;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  my $protocol = $doc->getElementsByTagName('protocol')->[0];
  foreach my $class ($protocol->getElementsByTagName('class')) {
    if ($class->getAttribute("name") eq $classname) {
      my $messages_by_name = {};
      foreach my $message ($class->getElementsByTagName('message')) {
	my $message_name = $message->getAttribute("name");
	print $message_name."\n";
	Paparazzi::IvyProtocol::bind_msg("ground", "alert", $message_name, { }, [\&on_alert, $self]);
      }
    }
  }
}

# caution:
# requires fields ac_id and level
sub on_alert {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
  my $ac_id = $fields->{ac_id};
  my $level = $fields->{level};
  #my $value = $fields->{value};
  $self->add($ac_id, $msg_name, $level);
}

# generate tag for an alert with aircraft id, alert_id and level
sub gen_tag {
  my ($self, $ac_id, $alert_id, $level) = @_;
  return "ALERT_".$alert_id."_AC_".$ac_id."_LEVEL_".$level;
}

# add an alert if there is no alert of the same kind still alive
sub add {
  my ($self, $ac_id, $alert_id, $level) = @_;
  my $tag = $self->gen_tag($ac_id, $alert_id, $level);
  if(!defined($self->{tags}->{$tag})) {
    $self->{tags}->{$tag} = 0;
  }
  #print Dumper($self->{tags}->{$tag});
  if($self->{tags}->{$tag}==0) {
    $self->add_alert($ac_id, $alert_id, $level);
  }
}

# add the graphic alert
sub add_alert {
  my ($self, $ac_id, $alert_id, $level) = @_;
  my $zinc = $self->get(-zinc);
  my $time = $self->get_localtime();
  my $width = $self->get(-width);
  my $tag = $self->gen_tag($ac_id, $alert_id, $level);

  my $font = $self->get(-font);
  my $step = $self->get(-step);
  my $color = $self->get(-color);
  foreach my $item ($zinc->find('withtag', "alert")) {
    $zinc->translate($item, 0, $step);
  }

  $zinc->add(
	     'rectangle',
	     $self->{group},
	     [1, 1, $width, $step],
	     -filled => 1,
	     -fillcolor => $color,
	     -linecolor => "black",
	     -tags => [ "alert", $tag ]
  );
  $zinc->add(
	     'text',
	     $self->{group},
	     -position => [5, 2],
	     -text => $time." - ".$ac_id." ".$alert_id." ".$level,
	     -tags => ["alert", $tag ],
	     -font => $font
	    );
  foreach my $item ($zinc->find('withtag', $tag)) {
    $zinc->bind($item, '<1>', sub { $self->acknowledge($tag); });
  }
  $self->{tags}->{$tag} = 1;
  print "adding tag ".$tag."\n";
}


# acknowledge a king of alert (identifying the alert with a tag)
sub acknowledge {
  my ($self, $alert_tag) = @_;
  my $zinc = $self->get(-zinc);
  foreach my $item ($zinc->find('withtag', $alert_tag)) {
    my $type = $zinc->type($item);
    if( $type eq 'rectangle') {
      my $color = $self->get(-colorack);
      $zinc->itemconfigure($item, -fillcolor => $color);
      $self->{tags}->{$alert_tag} = 0;
    }
  }
}
sub get_localtime {
  my ($self) = @_;
  my ($sec, $min, $h, @nothing) = localtime time;
  sprintf("%02d:%02d:%02d", $h, $min, $sec);
}
1;
