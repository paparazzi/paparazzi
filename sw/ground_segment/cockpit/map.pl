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

use constant MAP_DEBUG => 0;
use constant APP_ID => "Paparazzi Map";
use constant MESSAGE_WHEN_READY => APP_ID.': READY';

use Paparazzi::IvyProtocol;
use Paparazzi::MapView;
use Paparazzi::Utils;

use Getopt::Long;
use Tk;
use Ivy;
use Text::CSV;

my $paparazzi_home = Paparazzi::Environment::paparazzi_home();

my $options = {
	       paparazzi_home => $paparazzi_home,
	       ivy_bus  => "127.255.255.255:2010",
	       data_dir => $paparazzi_home."/data",
	       map_file => "maps/defaultUTM.xml",
	       conf_dir => $paparazzi_home."/conf",
	       render => "1"
	      };

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->{aircrafts} = [];
  $self->start_ivy();
  $self->build_gui();
}

sub start_ivy {
  my ($self) = @_;

  Ivy->init (-ivyBus        => $options->{ivy_bus},
	     -appName       => APP_ID,
	     -loopMode      => 'TK',
	     -messWhenReady => MESSAGE_WHEN_READY,
	    );
  $self->{ivy} = Ivy->new();
  $self->{ivy}->start();
  Paparazzi::IvyProtocol::read_protocol($paparazzi_home."/conf/messages.xml", "aircraft_info");
  Paparazzi::IvyProtocol::read_protocol($paparazzi_home."/conf/messages.xml", "ground");
  Paparazzi::IvyProtocol::bind_message("ground", "AIRCRAFTS", {}, $self->{ivy}, [$self, \&ivyOnAircrafts]);
}

sub build_gui {
  my ($self) = @_;
  my $mw = MainWindow->new();
  $mw->title("Paparazzi map : $options->{map_file}");
  $mw->geometry("600x600");
  $self->{map_view} = $mw->MapView(-render => $options->{render});
  $self->{map_view}->pack(-fill => 'both', -expand => "1");
  
  $self->{map_view}->load_map($options->{data_dir}."/".$options->{map_file});
#  my $flight_plan = `$paparazzi_home/sw/tools/gen_flight_plan.out -dump $ flight_plan_name`;
#  $mv->load_flight_plan($flight_plan);
}

sub ivyOnAircrafts {
  print "in ivyOnAircrafts\n";
  my ($self, @args) = @_;
  my $fields_by_name = Paparazzi::IvyProtocol::get_values_by_name("ground", "AIRCRAFTS", \@args);
#  print Dumper($fields_by_name);
  my $ac_list = $fields_by_name->{ac_list};
  my $csv = Text::CSV->new();
  $csv->parse($ac_list);
  my @new_ac = $csv->fields();
  my @added_ac = Utils::diff_array(\@new_ac, $self->{aircrafts});
  my @removed_ac = Utils::diff_array($self->{aircrafts}, \@new_ac);
  foreach my $new_ac (@added_ac) {
    print "added_ac $new_ac\n";
    Paparazzi::IvyProtocol::bind_message("aircraft_info", "FLIGHT_PARAM", {id => $new_ac}, $self->{ivy}, [$self, \&ivyOnFlightParam]);
    my $track_item = $self->{map_view}->set_track_geo($new_ac, [0, 0]);

  }
  foreach my $ac2 (@removed_ac) {
    print "removed_ac $ac2\n"; 
  }
  $self->{aircrafts} = \@new_ac;
}

sub ivyOnFlightParam {
#  print "in ivyOnFlightParam\n";
  my ($self, @args) = @_;
  my $fbn = Paparazzi::IvyProtocol::get_values_by_name("aircraft_info", "FLIGHT_PARAM", \@args);
  my $ac = $fbn->{id};
#  print Dumper($fbn);
  $self->{map_view}->set_track_geo($ac, [$fbn->{east}, $fbn->{north}]);
}

use Data::Dumper;

GetOptions ("b=s" => \$options->{ivy_bus},
	    "t=s" => \$options->{paparazzi_home},
	    "d=s" => \$options->{data_dir},
	    "m=s" => \$options->{map_file},
	    "c=s" => \$options->{conf_dir},
	    "r=s" => \$options->{render},
	   );
print Dumper($options);
my $map = Map->new();
MainLoop();

1;

