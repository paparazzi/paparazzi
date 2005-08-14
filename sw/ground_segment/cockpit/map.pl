#!/usr/bin/perl -w

package Map;

my @paparazzi_lib;
BEGIN {
  @paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
    ($ENV{PAPARAZZI_SRC}."/sw/lib/perl", $ENV{PAPARAZZI_SRC}."/sw/ground_segment/cockpit"):();
}
use lib (@paparazzi_lib);

use vars qw (@ISA) ;
use Subject;
@ISA = ("Subject");

use strict;
use Paparazzi::Environment;

use constant APP_NAME => "Map";

use Tk;
use Data::Dumper;

use strict;

my $options = {
	       ivy_bus  => "127.255.255.255:2010",
	       map_file => "defaultUTM.xml",
	      };

use Paparazzi::Environment;
use Paparazzi::IvyProtocol;
use Paparazzi::AircraftsManager;
use Paparazzi::MapView;

Paparazzi::Environment::parse_command_line($options);



sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  my $protocol_file = Paparazzi::Environment::get_config_file("messages.xml");
  print "protocol_file $protocol_file\n";
  Paparazzi::IvyProtocol::init(-file      => $protocol_file,
			       -ivy_bus   => $options->{ivy_bus},
			       -app_name  => APP_NAME,
			       -loop_mode => 'TK',
			      );
  $self->build_gui();
  $self->{aircrafts_manager} = Paparazzi::AircraftsManager->new(-listen_to_all => 1);
  $self->{mw}->after(500, [\&on_foo, $self]);
}

sub on_foo {
  my ($self) = @_;
  print "in ivy_on_foo\n"; # if (COCKPIT_DEBUG);
  $self->{aircrafts_manager}->start();
}


sub build_gui {
  my ($self) = @_;
  my $mw = MainWindow->new();
  $mw->title("Paparazzi map : $options->{map_file}");
  my $win_size = [770, 600];
  my $win_pos = [310, 140];
  $mw->geometry(sprintf("%dx%d+%d+%d", $win_size->[0], $win_size->[1], $win_pos->[0], $win_pos->[1]));
  $self->{mw} = $mw;
  $self->{map_view} = $mw;
  my $map_view = Paparazzi::MapView->new(-mw => $mw);
  

#  my $flight_plan_name = $options->{conf_dir}."/".$options->{mission_file};
#  my $flight_plan = `/home/$paparazzi_home/sw/tools/gen_flight_plan.out -dump $ flight_plan_name`;
#  $mv->load_flight_plan($flight_plan, $win_size);
#  $mv->create_circle();
#  $mv->create_segment();
}

Paparazzi::Environment::parse_command_line($options) || pod2usage(-verbose => 0);
print Dumper($options);
my $map = Map->new();
MainLoop();

