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

use constant APP_NAME => "Cockpit";
use constant MESSAGE_WHEN_READY => APP_NAME.': READY';

use Paparazzi::Traces;
use Paparazzi::GuiConfig;
use Paparazzi::IvyProtocol;
use Paparazzi::AircraftsManager;
use Paparazzi::Aircraft;
use Paparazzi::PFD;
use Paparazzi::ND;
use Paparazzi::MissionD;
use Paparazzi::StripPanel;

use Tk;
#use Tk::PNG;
use Tk::Zinc;
use Ivy;
use Data::Dumper;
use Pod::Usage;

my $options = 
  {
   ivy_bus  => "127.255.255.255:2010",
   render => 1,
   vertical => 0,
   tracelevel => 1,
   width => 250,
   height => 750,
  };

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  Paparazzi::Traces::init($options->{tracelevel});
  my $gui_file = Paparazzi::Environment::get_config("gui.xml");
  Paparazzi::GuiConfig->init($gui_file);
  $self->{selected_ac} = undef;
  $self->build_gui();
  my $protocol_file = Paparazzi::Environment::get_config("messages.xml");
  Paparazzi::IvyProtocol::init(-file      => $protocol_file,
			       -ivy_bus   => $options->{ivy_bus},
			       -app_name  => APP_NAME,
			       -loop_mode => 'TK',
			    );
  $self->{aircrafts_manager} = 
    Paparazzi::AircraftsManager->new(-listen_to_all => 
				     ['FLIGHT_PARAM', 'AP_STATUS', 'NAV_STATUS', 'CAM_STATUS', 'ENGINE_STATUS',
				      'FLY_BY_WIRE', 'INFRARED', 'INFLIGH_CALIB', 'SVSINFO', 'WIND']);
  $self->{aircrafts_manager}->attach($self, 'NEW_AIRCRAFT', [\&on_new_aircraft]);
  $self->{mw}->after(500, [\&on_foo, $self]);
}

sub on_new_aircraft {
  my ($self, $ac_manager, $event, $ac_id) = @_;
  trace(TRACE_DEBUG, "cockpit::on_new_aircraft $ac_id");
  my $aircraft = $self->{aircrafts_manager}->get_aircraft_by_id($ac_id);
  $self->{strip_panel}->add_strip($aircraft);
  $self->{md}->add_aircraft($aircraft);
}

sub build_gui {
  my ($self) = @_;
  $self->{mw} = MainWindow->new();
  my $size = $options->{width};
  my $top_frame =  $self->{mw}->Frame()->pack(-side => 'top', -fill => 'both');
  my ($stp_p, $stp_w, $stp_h) = ([0, 0],                   $size, $size);
  my ($pfd_p, $pfd_w, $pfd_h) = ([$stp_w, 0]             , $size, $size);
  my ($nd_p,  $nd_w,  $nd_h) =  ([$pfd_p->[0]+ $pfd_w, 0], $size, $size);
  my ($z_w, $z_h) = ($stp_w + $pfd_w + $nd_w, $size);
  
  if ($options->{vertical}) {
    ($pfd_p, $pfd_w, $pfd_h) = ([0, $stp_h] , $size, $size);
    ($nd_p,  $nd_w,  $nd_h) =  ([0, $pfd_p->[1]+ $pfd_h], $size, $size);
    ($z_w, $z_h) = ($size, $stp_h + $pfd_h + $nd_h)
  }
  
  my $zinc = $top_frame->Zinc(-width => $z_w ,
			      -height => $z_h,
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
  $self->{pfd}->attach($self, 'SHOW_PAGE', ['onShowPage']);
  $self->{nd} = Paparazzi::ND->new( -zinc => $zinc,
				    -origin => $nd_p,
				    -width  => $nd_w,
				    -height => $nd_h,
 				  );
  my $sw = MainWindow->new(-title => "Flight Plan", -class => "Flight Plan");
 #  my $bot_frame =  $self->{mw}->Frame()->pack(-side => 'bottom', -fill => 'both', -expand => 1);
#  my $md = $bot_frame->MissionD(-bg => '#c1daff');
  my $md = $sw->MissionD(-bg => '#c1daff');
  $md->pack(-side => 'bottom', -anchor => "n", -fill => 'both', -expand => 1);
  $self->{md} = $md;
  $sw->configure(-width => 600, -height => 300);

}

sub on_foo {
  my ($self) = @_;
  $self->{aircrafts_manager}->start();
  Paparazzi::IvyProtocol::bind_msg("ground", "ground", "SELECTED", {}, [\&ivy_on_selected, $self]);
}

sub ivy_on_selected {
  my ($sender_name, $msg_class, $msg_name, $fields, $self) = @_;
  my $ac_id = $fields->{aircraft_id};
  trace(TRACE_DEBUG, "cockpit::ivy_on_selected : selecting aircraft $ac_id");
  if (defined $self->{aircrafts_manager}->get_aircraft_by_id($ac_id)) {
    $self->select_ac($ac_id);
  }
  else {
    trace(TRACE_ERROR, "cockpit::ivy_on_selected : received select order for unknown aircraft $ac_id");
  }
}

sub on_aircraft_selection {
  my ($self, $_sp, $what, $new_selected_ac ) = @_;
  $self->select_ac($new_selected_ac);
  Paparazzi::IvyProtocol::send_msg('ground', 'SELECTED', { aircraft_id => $new_selected_ac });
}

sub select_ac {
  my ($self, $ac_id) = @_;
  $self->{selected_ac} = $ac_id;
  $self->{aircrafts_manager}->configure('-selected_aircrafts' => [$ac_id]);
  my $aircraft = $self->{aircrafts_manager}->get_aircraft_by_id($ac_id);
  $self->{pfd}->configure('-selected_ac', $aircraft);
  $self->{nd}->configure('-selected_ac', $aircraft);
  $self->{md}->set_selected_ac($aircraft);
}

sub onShowPage {
  my ($self, $component, $signal, $page) = @_;
  trace(TRACE_DEBUG, "cockpit::onShowPage $page");
  $self->{nd}->configure('-page' => $page);
}


Paparazzi::Environment::parse_command_line($options) || pod2usage(-verbose => 0);
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
