package Paparazzi::Airframe;

use Subject;
@ISA = ("Subject");
use strict;

use XML::DOM;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(
                    -url	        => [S_NEEDINIT, S_PASSIVE,  S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -name	        => [S_NOINIT,   S_PASSIVE,  S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  my $airframe_url = $self->get('-url');
  $airframe_url =~ /file:\/\/(.*)/;
  my $filename = $1;
  $self->parse_airframe($filename);
}

sub parse_airframe {
  my ($self, $filename) = @_;

  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  my $airframe = $doc->getElementsByTagName('airframe')->[0];
  $self->configure( -name => $airframe->getAttribute('name'));
}

1;
