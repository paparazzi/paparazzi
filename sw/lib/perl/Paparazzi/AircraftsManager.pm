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
		    -listen_to_all => [S_NOINIT, S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, []],
		    -listen_to_selected => [S_NOINIT, S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, []],
		    -selected_aircrafts => [S_NOINIT, S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, []],
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
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
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
  my $all_ac_msg = $self->get('-listen_to_all');
  foreach my $msg_name (@{$all_ac_msg}) {
    Paparazzi::IvyProtocol::bind_msg("ground", "ground", $msg_name, {aircraft_id => $ac_id},
				     [\&on_ac_msg, $self]);
  }
  $self->notify('NEW_AIRCRAFT', $ac_id);
}

sub on_config {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
  my $ac_id = $fields->{ac_id};
  my $ac = $self->get('-aircrafts')->{$ac_id};
  delete $fields->{ac_id};
  my $fp_url = $fields->{flight_plan};
  if (defined $fp_url) {
    my $fp = Paparazzi::Flightplan->new(-url => $fp_url);
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

sub get_aircraft_by_id {
  my ($self, $id) = @_;
  return $self->get('-aircrafts')->{$id};
}

1;
