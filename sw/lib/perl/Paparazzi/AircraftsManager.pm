# $Id$
#
# AircraftsManager object
# 
# Copyright (C) 2005 Antoine Drouin
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

package Paparazzi::AircraftsManager;

use Subject;
@ISA = ("Subject");
use strict;

use Text::CSV;

use Paparazzi::IvyProtocol;
use Paparazzi::Aircraft;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec( 
		    -listen_to_all => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0],
		    -aircrafts => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		    -pubevts   => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_APPEND, S_NOPRPG,[]],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->configure('-pubevts' => 'NEW_AIRCRAFT');
  $self->configure('-pubevts' => 'DIE_AIRCRAFT');
}

sub start {
  my ($self) = @_;
  Paparazzi::IvyProtocol::send_request("ground", "ground", "AIRCRAFTS", {}, [\&on_aircrafts, $self]);
  Paparazzi::IvyProtocol::bind_msg("ground", "ground", "NEW_AIRCRAFT", 
				   {}, [\&on_aircraft_new_die, $self]);
}

sub on_aircraft_new_die {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
  $self->add_aircraft($fields->{ac_id});
}

sub on_aircrafts {
  print "AircraftsManager : in on_aircrafts\n";
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
#  use Data::Dumper;
#  print "fields ".Dumper($fields)."\n";
  my $csv = Text::CSV->new();
  $csv->parse($fields->{ac_list});
  my @ac_list = $csv->fields();
  foreach my $ac_id (@ac_list) {
    $self->add_aircraft($ac_id) unless $ac_id eq "";
  }
}


sub add_aircraft {
  my ($self, $ac_id) = @_;
  Paparazzi::IvyProtocol::send_request("ground", "ground", "CONFIG",
				       {ac_id => $ac_id}, [\&on_config, $self]) unless $ac_id eq "";
  my $aircraft = Paparazzi::Aircraft->new(-ac_id => $ac_id);
  $self->get('-aircrafts')->{$ac_id} = $aircraft;
  $self->listen_to_ac($ac_id) if ($self->get('-listen_to_all'));

  print "int AircraftsManager : notifying new ac $ac_id\n";
  $self->notify('NEW_AIRCRAFT', $ac_id);
}


sub on_config {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
  print "AircraftsManager : in on_config\n"; # if (COCKPIT_DEBUG);
#  use Data::Dumper;
#  print "fields ".Dumper($fields)."\n";
  my $ac_id = $fields->{ac_id};
  my $ac = $self->get('-aircrafts')->{$ac_id};
  delete $fields->{ac_id};
  $ac->configure(%{$fields});
}

sub on_ac_msg {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
#  print "in on_ac_msg $msg_name\n";# if (COCKPIT_DEBUG);
  my $ac_id = $fields->{ac_id};
  my $aircraft = $self->get('-aircrafts')->{$ac_id};
  delete $fields->{ac_id};
#  print Dumper($fields);
  $aircraft->configure(%{$fields});
}

sub listen_to_ac {
  my ($self, $ac_id)  = @_;
  my @ac_events = ( ['FLIGHT_PARAM',  \&on_ac_msg],
		    ['NAV_STATUS',    \&on_ac_msg],
		    ['AP_STATUS',     \&on_ac_msg],
		    ['ENGINE_STATUS', \&on_ac_msg],
		    #		    ['SATS', \&ivyOnSats],
		  );
  foreach my $event (@ac_events) {
    Paparazzi::IvyProtocol::bind_msg("ground", "ground", $event->[0], 
				     {aircraft_id => $ac_id}, [$event->[1], $self]);
  }
}

sub listen_to_all {

}

sub get_aircraft_by_id {
  my ($self, $id) = @_;
  return $self->get('-aircrafts')->{$id};
}

1;
