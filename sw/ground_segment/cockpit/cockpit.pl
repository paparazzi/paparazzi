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
use Text::CSV;
use Data::Dumper;
use Pod::Usage;

my $options = 
  {
   ivy_bus  => "127.255.255.255:2005",
   render => 1,
  };

my $md;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->{aircrafts} = {};
  $self->{selected_ac} = undef;
  $self->{wind_dir} = 0.;
  $self->{wind_speed} = 0.;
  $self->build_gui();
  my $protocol_file = Paparazzi::Environment::get_config_file("messages.xml");
  Paparazzi::IvyProtocol::init(-file      => $protocol_file,
			       -ivy_bus   => $options->{ivy_bus},
			       -app_name  => APP_NAME,
			       -loop_mode => 'TK',
			    );
  $self->{aircrafts_manager} = Paparazzi::AircraftsManager->new(-listen_to_all => 1);
  $self->{mw}->after(500, [\&on_foo, $self]);
}

sub build_gui {
  my ($self) = @_;
  $self->{mw} = MainWindow->new();
  my $top_frame =  $self->{mw}->Frame()->pack(-side => 'top', -fill => 'both');
  my $bot_frame =  $self->{mw}->Frame()->pack(-side => 'bottom', -fill => 'both');
  my ($stp_p, $stp_w, $stp_h) = ([0, 0],                   800, 300);
  my ($pfd_p, $pfd_w, $pfd_h) = ([$stp_w, 0]             , 300, $stp_h);
  my ($nd_p,  $nd_w,  $nd_h) =  ([$pfd_p->[0]+ $pfd_w, 0], 600, 600);
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
#  $self->{strip_panel}->attach($self, '-selected_ac', ['onAircratftSelection', ()]);

  $self->{pfd} = Paparazzi::PFD->new( -zinc => $zinc,
				      -origin => $pfd_p,
				      -width  => $pfd_w,
				      -height => $pfd_h,
				    );
#  $self->{pfd}->attach($self, 'SHOW_PAGE', ['onShowPage']);
#   $self->{nd} = Paparazzi::ND->new( -zinc => $zinc,
# 				    -origin => $nd_p,
# 				    -width  => $nd_w,
# 				    -height => $nd_h,
# 				  );
#   $self->{nd}->attach($self, 'WIND_COMMAND', ['onWindCommand']);
#  $md = $bot_frame->MissionD(-bg => '#c1daff');
#  $md->pack(-side => 'bottom', -anchor => "n", -fill => 'both');


}



#    $self->{strip_panel}->add_strip($aircraft);


sub on_foo {
  my ($self) = @_;
  print "in ivy_on_foo\n"; # if (COCKPIT_DEBUG);
  $self->{aircrafts_manager}->start();
  Paparazzi::IvyProtocol::bind_msg("ground", "ground", "SELECTED", {}, [\&ivy_on_selected, $self]);
}



sub ivy_on_selected {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
  print "in ivy_on_selected\n"; # if (COCKPIT_DEBUG);
 
  my $sel_ac_id = $fields->{aicraft_id};
  $self->{selected_ac} = $sel_ac_id;
  $self->{aircrafts_manager}->listen_to_ac($sel_ac_id);
  print "configuring pfd\n";
  my $aircraft = $self->{aircrafts_manager}->get_aircraft_by_id($sel_ac_id);
  my $pfd = $self->{pfd};
  $pfd->configure('-selected_ac', $aircraft);
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

sub onAircratftSelection {
#  print ("onAircratftSelection @_\n");
  my ($self, $_sp, $what, $new_selected_ac ) = @_;
  my $ivy = $self->{ivy};
#  Paparazzi::IvyProtocol::sendMsg($ivy, "ground", "SELECTED",{ id => $new_selected_ac});
  return if ($new_selected_ac eq "NONE");
  my @ac_events = ( ['FLIGHT_PARAM',  \&ivyOnFlightParam],
		   ['NAV_STATUS',    \&ivyOnNavStatus],
		   ['AP_STATUS',     \&ivyOnApStatus],
		   ['ENGINE_STATUS', \&ivyOnEngineStatus],
		   ['SATS', \&ivyOnSats],
		  );
  foreach my $event (@ac_events) {
    # removes existing binding
#    Paparazzi::IvyProtocol::bind_message("aircraft_info", $event->[0], {id => $self->{selected_ac}}, $ivy, undef) unless !defined $self->{selected_ac};
    # add new one
#    Paparazzi::IvyProtocol::bind_message("aircraft_info", $event->[0], {id => $new_selected_ac}, $ivy, [$self, $event->[1]]);
  }
  $self->{selected_ac} = $new_selected_ac;
}






# sub ivyOnNavStatus {
#   my ($self, @args) = @_;
# #  my $fields_by_name = Paparazzi::IvyProtocol::get_values_by_name("aircraft_info", "NAV_STATUS", \@args);
#   print Dumper($fields_by_name) if (COCKPIT_DEBUG);
#   $md->set_block_and_stage($fields_by_name->{cur_block}, $fields_by_name->{cur_stage});
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




# sub ivyOnApStatus {
#   print "in ivyOnApStatus\n" if (COCKPIT_DEBUG);
#   my ($self, @args) = @_;
# #  my $fbn = Paparazzi::IvyProtocol::get_values_by_name("aircraft_info", "AP_STATUS", \@args);
# #  print $self->{selected_ac}." ".Dumper($fbn);# if (COCKPIT_DEBUG); 
#   $self->{pfd}->configure( -ap_mode  => $fbn->{mode},
# #			   -h_mode   => $fbn->{h_mode},
# #			   -v_mode   => $fbn->{v_mode},
# #			   -target_vz  =>  $fbn->{target_climb}
# 			   -target_alt =>  $fbn->{target_alt},
# 			   -target_heading =>  $fbn->{target_heading},
# 			 ); 
#   $self->{nd}->configure( -ap_status, $fbn);
# }

# sub ivyOnEngineStatus {
#   print "in ivyOnEngineStatus\n" if (COCKPIT_DEBUG);
#   my ($self, @args) = @_;
# #  my $fbn = Paparazzi::IvyProtocol::get_values_by_name("aircraft_info", "ENGINE_STATUS", \@args);
#   $self->{nd}->configure( -engine_status, $fbn);
# }

# sub ivyOnSats {
#   print "in ivyOnSats\n" if (COCKPIT_DEBUG);
#   my ($self, @args) = @_;
# #  my $fbn = Paparazzi::IvyProtocol::get_values_by_name("aircraft_info", "SATS", \@args);
#   $self->{nd}->configure( -sats, $fbn);
# }

# sub ivyOnIR {
#   my ($self, @args) = @_;
#   my $h = { 
# 	   ir => $args[2],
# 	   rad => $args[3],
# 	   rad_of_ir => $args[4],
# 	   ir_roll_ntrl => $args[5],
# 	   ir_pitch_ntrl => $args[6]
# 	  };
#   $self->{nd}->configure('-lls' => $h->{rad_of_ir});
#   $self->{nd}->put_lls($h->{rad_of_ir});
# }



# sub ivyStatusCbk {
#   printf("in ivyStatusCbk\n")  if (COCKPIT_DEBUG);
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
