package Paparazzi::Airframe;

use Subject;
@ISA = ("Subject");
use strict;

require XML::DOM;
require LWP::Simple;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(
                    -url     => [S_NEEDINIT, S_PASSIVE,  S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -ac_name => [S_NOINIT,   S_PASSIVE,  S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  my $airframe_url = $self->get('-url');
#  print "in Airframe::completeinit url $airframe_url\n";
  my $airframe_xml = LWP::Simple::get($airframe_url);
  $self->parse_airframe($airframe_xml) if defined $airframe_xml;
}

sub parse_airframe {
  my ($self, $airframe_xml) = @_;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parse($airframe_xml);
  my $airframe = $doc->getElementsByTagName('airframe')->[0];
  $self->configure( -ac_name => $airframe->getAttribute('name'));
}

1;
