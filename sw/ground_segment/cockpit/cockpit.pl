#!/usr/bin/perl -w
package Cockpit;

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

use constant COCKPIT_DEBUG => 0;
use constant APP_NAME => "Cockpit";
use constant MESSAGE_WHEN_READY => APP_NAME.': READY';

use Paparazzi::IvyProtocol;
use Paparazzi::AircraftsManager;
use Paparazzi::Aircraft;
use Paparazzi::PFD;
use Paparazzi::ND;
use Paparazzi::MissionD;
use Paparazzi::StripPanel;
use Paparazzi::Geometry;

use Tk;
#use Tk::PNG;
use Tk::Zinc;
use Ivy;
use Data::Dumper;
use Pod::Usage;

my $options = 
  {
   ivy_bus  => "127.255.255.255:2005",
   render => 1,
  };

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->{selected_ac} = undef;
  $self->{wind_dir} = 0.;
  $self->{wind_speed} = 0.;
  $self->build_gui();
  my $protocol_file = Paparazzi::Environment::get_config("messages.xml");
  Paparazzi::IvyProtocol::init(-file      => $protocol_file,
			       -ivy_bus   => $options->{ivy_bus},
			       -app_name  => APP_NAME,
			       -loop_mode => 'TK',
			    );
  $self->{aircrafts_manager} = Paparazzi::AircraftsManager->new(-listen_to_all => 1);
  $self->{aircrafts_manager}->attach($self, 'NEW_AIRCRAFT', [\&on_new_aircraft]);
  $self->{mw}->after(500, [\&on_foo, $self]);
}

sub on_new_aircraft {
  my ($self, $ac_manager, $event, $ac_id) = @_;
  print "in Cockpit : on_new_aircraft\n";
  my $aircraft = $self->{aircrafts_manager}->get_aircraft_by_id($ac_id);
  $self->{strip_panel}->add_strip($aircraft);
  $self->{md}->add_aircraft($aircraft);
}



sub build_gui {
  my ($self) = @_;
  $self->{mw} = MainWindow->new();
#  $self->{mw}->geometry("1280x1024");
  my $top_frame =  $self->{mw}->Frame()->pack(-side => 'top', -fill => 'both');
  my $bot_frame =  $self->{mw}->Frame()->pack(-side => 'bottom', -fill => 'both', -expand => 1);
  my ($stp_p, $stp_w, $stp_h) = ([0, 0],                   315, 300);
  my ($pfd_p, $pfd_w, $pfd_h) = ([$stp_w, 0]             , 300, $stp_h);
  my ($nd_p,  $nd_w,  $nd_h) =  ([$pfd_p->[0]+ $pfd_w, 0], 300, 300);
  my $zinc = $top_frame->Zinc(-width => $stp_w + $pfd_w + $nd_w ,
			      -height => $nd_h,
			      -backcolor => 'black',
			      -borderwidth => 3, -relief => 'sunken',
			      -render => $options->{render},
			      -lightangle => 130,);
  $zinc->pack(-side => 'left', -anchor => "nw");
  $self->{strip_panel} = Paparazzi::StripPanel->new( -zinc => $zinc,
						     -origin => $stp_p,
						     -width  => $stp_w,
						     -height => $stp_h
						   );
  $self->{strip_panel}->attach($self, 'SELECTED', ['on_aircraft_selection', ()]);

  $self->{pfd} = Paparazzi::PFD->new( -zinc => $zinc,
				      -origin => $pfd_p,
				      -width  => $pfd_w,
				      -height => $pfd_h,
				    );
#  $self->{pfd}->attach($self, 'SHOW_PAGE', ['onShowPage']);
   $self->{nd} = Paparazzi::ND->new( -zinc => $zinc,
 				    -origin => $nd_p,
 				    -width  => $nd_w,
 				    -height => $nd_h,
 				  );
#   $self->{nd}->attach($self, 'WIND_COMMAND', ['onWindCommand']);
  my $md = $bot_frame->MissionD(-bg => '#c1daff');
  $md->pack(-side => 'bottom', -anchor => "n", -fill => 'both', -expand => 1);
  $self->{md} = $md;

}

sub on_foo {
  my ($self) = @_;
  print "in ivy_on_foo\n"; # if (COCKPIT_DEBUG);
  $self->{aircrafts_manager}->start();
  Paparazzi::IvyProtocol::bind_msg("ground", "ground", "SELECTED", {}, [\&ivy_on_selected, $self]);
}

sub ivy_on_selected {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
  print "in ivy_on_selected\n"; # if (COCKPIT_DEBUG);
  $self->select_ac($fields->{aicraft_id});
}

sub on_aircraft_selection {
  #  print ("onAircratftSelection @_\n");
  my ($self, $_sp, $what, $new_selected_ac ) = @_;
  print "on_aircraft_selection @_\n";
  $self->select_ac($new_selected_ac);

}

sub select_ac {
  my ($self, $ac_id) = @_;
  $self->{selected_ac} = $ac_id;
  $self->{aircrafts_manager}->listen_to_ac($ac_id);
  my $aircraft = $self->{aircrafts_manager}->get_aircraft_by_id($ac_id);
  $self->{pfd}->configure('-selected_ac', $aircraft);
  $self->{md}->set_selected_ac($aircraft);
}

sub onShowPage {
  my ($self, $component, $signal, $page) = @_;
  print "cockpit::onShowPage $page\n";
  print "$self->{nd}\n";
  $self->{nd}->configure('-page' => $page);
}

# sub onWindCommand {
#   my ($self, $component, $signal, $cmd) = @_;
#   print "cockpit::onWindCommand $cmd\n";
#   $self->{ivy}->sendMsgs("WIND_COMMAND $cmd");
#   if ($cmd eq "start") {
#     $self->{timer_id} = $self->{mw}->repeat(5000, [\&onTimer, $self]);
#     $self->{ivy}->sendMsgs("WIND_COMMAND clear");
#   }
#   elsif ($cmd eq "stop") {
#     $self->{mw}->afterCancel($self->{timer_id})
#   }
# }





# sub onWindRes {
#   my ($self, @args) = @_;
# #  my $fields_by_name = Paparazzi::IvyProtocol::get_values_by_name("ground", "WIND_RES", \@args);
#   print  Dumper ($fields_by_name) if (COCKPIT_DEBUG);
# }

# sub onConfigRes {
#   print "in onConfigRes\n" if (COCKPIT_DEBUG);
#   my ($self, @args) = @_;
# #  my $fields_by_name = Paparazzi::IvyProtocol::get_values_by_name("aircraft_info", "CONFIG", \@args);
#   my $fp = $fields_by_name->{flight_plan};
#   #  print Dumper($fields_by_name);
#   my $paparazzi_src =  Paparazzi::Environment::paparazzi_src();
#   my $gfp_bin = ((defined $paparazzi_src) ? $paparazzi_src."/sw/tools" : "/usr/share/paparazzi/bin") ."/gen_flight_plan.out";
#   my $flight_plan_xml = `$gfp_bin -dump $fp`;
#   $md->load_flight_plan($flight_plan_xml);
#   $md->set_block_and_stage(0,0);
# }






# sub onTimer {
#   my ( $self) = @_;
# #  print("in onTimer $self\n");
#   $self->{ivy}->sendMsgs("WIND_REQ toto", {-id => "toto"});
#  #   Paparazzi::IvyProtocol::request_message("ground", "CONFIG", {id => 'ground'}, $self->{ivy}, [$self, \&ivyOnWind]);
# }

# sub ivyOnWind {
# #  print "in ivyOnWind\n"; # if (COCKPIT_DEBUG);
#   my ($self, @args) = @_;
# #  my $fields_by_name = Paparazzi::IvyProtocol::get_values_by_name("ground", "RES_WIND", \@args);
#   $self->{wind_dir} = $args[2];
#   $self->{wind_speed} = $args[3];

#   my $h = { dir => $args[2],
# 	    speed => $args[3],
# 	    mean_aspeed => $args[4], 
# 	    stddev => $args[5],
# 	  };
  
# #  print Dumper($h);# if (COCKPIT_DEBUG); 
#   $self->{nd}->configure('-wind' => $h);
# }

Paparazzi::Environment::parse_command_line($options) || pod2usage(-verbose => 0);
print Dumper($options);
my $cockpit = Cockpit->new();
MainLoop();

__END__

=head1 NAME

cockpit

=head1 SYNOPSIS

cockpit [options]

Options:
    -ivybus          the ivy bus (eg 127.2552.55255:2010)
    -render          toggle opengl usage

=head1 OPTIONS

=over 8

=item B<-help>

Print a brief help message and exits.

=item B<-man>

Prints the manual page and exits.

=back

=head1 DESCRIPTION

B<This program> will display an aircraft cockpit.

=cut
