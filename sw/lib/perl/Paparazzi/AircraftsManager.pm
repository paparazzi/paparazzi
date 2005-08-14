package Paparazzi::AircraftsManager;

use Subject;
@ISA = ("Subject");
use strict;

use Paparazzi::IvyProtocol;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec( 
		    -listen_to_all => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0],
		    -aircrafts => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
}

sub start {
  my ($self) = @_;
  Paparazzi::IvyProtocol::send_request("ground", "ground", "AIRCRAFTS", {}, [\&on_aircrafts, $self]);
}

sub on_aircrafts {
  print "AircraftsManager : in on_aircrafts\n";
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
#  use Data::Dumper;
#  print "fields ".Dumper($fields)."\n";
  my $csv = Text::CSV->new();
  $csv->parse($fields->{ac_list});
  my @ac_list = $csv->fields();
  my $aircrafts = $self->get('-aircrafts');
  foreach my $ac_id (@ac_list) {
    Paparazzi::IvyProtocol::send_request("ground", "ground", "CONFIG",
					 {ac_id => $ac_id}, [\&on_config, $self]) unless $ac_id eq "";
    my $aircraft = Paparazzi::Aircraft->new(-ac_id => $ac_id);
    $aircrafts->{$ac_id} = $aircraft;
    $self->listen_to_ac($ac_id) if ($self->get('-listen_to_all'));
  }
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
