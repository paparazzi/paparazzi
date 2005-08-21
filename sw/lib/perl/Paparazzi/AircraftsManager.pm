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

use Paparazzi::IvyProtocol;
use Paparazzi::Aircraft;
use Paparazzi::Flightplan;
use Paparazzi::Airframe;



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
#  print "AircraftsManager::on_aircrafts\n";
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
#  use Data::Dumper;
#  print "in AircraftsManager::on_aircrafts : dumping fields\n ".Dumper($fields);
  my $ac_list = $fields->{ac_list};
  foreach my $ac_id (@{$ac_list}) {
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

#  print "int AircraftsManager : notifying new ac $ac_id\n";
  $self->notify('NEW_AIRCRAFT', $ac_id);
}


sub on_config {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
#  print "AircraftsManager::on_config\n";
#  use Data::Dumper;
#  print "fields ".Dumper($fields)."\n";
  my $ac_id = $fields->{ac_id};
  my $ac = $self->get('-aircrafts')->{$ac_id};
  delete $fields->{ac_id};

  my $fp_url = $fields->{flight_plan};
  if (defined $fp_url) {
    #    print "in AircraftsManager : on_config creating new flight plan\n";
    my $fp = Paparazzi::Flightplan->new(-url => $fp_url);
#    use Data::Dumper; 
#    print "##### waypoints\n".Dumper($fp->get('-waypoints'));
#    print "##### mission\n".Dumper($fp->get('-mission'));
    $fields->{flight_plan} = $fp;
  }
  my $airframe_url = $fields->{airframe};
  if (defined $airframe_url) {
    my $af = Paparazzi::Airframe->new(-url => $airframe_url);
    $fields->{airframe} = $af;
  }

  $ac->configure(%{$fields});
}

sub on_ac_msg {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
  my $ac_id = $fields->{ac_id};
  my $aircraft = $self->get('-aircrafts')->{$ac_id};
#  print "AircraftsManager::on_ac_msg : $msg_name\n".Dumper($fields);
  if (defined ($aircraft)) {
    delete $fields->{ac_id};
    if ($msg_name eq "SVSINFO" or $msg_name eq "ENGINE_STATUS") {
      $aircraft->configure('-'.(lc $msg_name) => $fields);
    }
    else {
      $aircraft->configure(%{$fields});
    }
  }
  else {
    print STDERR "in AircraftsManager::on_ac_msg : unknow aircraft $ac_id in message $msg_class:$msg_name\n";
  }
}

sub listen_to_ac {
  my ($self, $ac_id)  = @_;
  my @ac_msgs = ( 'FLIGHT_PARAM', 'AP_STATUS', 'NAV_STATUS', 'CAM_STATUS', 'ENGINE_STATUS',
		  'FLY_BY_WIRE', 'INFRARED', 'INFLIGH_CALIB', 'SVSINFO');
  foreach my $msg_name (@ac_msgs) {
    Paparazzi::IvyProtocol::bind_msg("ground", "ground", $msg_name, {aircraft_id => $ac_id}, 
				     [\&on_ac_msg, $self]);
  }
}

sub listen_to_all {

}

sub get_aircraft_by_id {
  my ($self, $id) = @_;
  return $self->get('-aircrafts')->{$id};
}

1;
