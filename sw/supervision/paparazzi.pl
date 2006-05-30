#!/usr/bin/perl -w
package Paparazzi;

my $paparazzi_lib;
BEGIN {
  $paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
    $ENV{PAPARAZZI_SRC}."/sw/lib/perl" : "/usr/lib/paparazzi/";
}
use lib ($paparazzi_lib);
#use lib ($ENV{PAPARAZZI_SRC}."/sw/supervision");

use Paparazzi::CpGui;
@ISA = qw(Paparazzi::CpGui);

use Paparazzi::Environment;

use strict;

use Tk;
use Subject;

use Data::Dumper;
use Getopt::Long;

sub populate {
  my ($self, $args) = @_;
  my $paparazzi_src = Paparazzi::Environment::paparazzi_src();
  my $paparazzi_home = Paparazzi::Environment::paparazzi_home();
  Paparazzi::Environment::check_paparazzi_home();
  $args->{-config_file} = $paparazzi_home."/conf/control_panel.xml";
  $args->{-variables} = {paparazzi_home => $paparazzi_home};
  $args->{-bin_base_dir} = defined $paparazzi_src ? $paparazzi_src : "/usr/share/paparazzi";
  $args->{-logo_file} = $paparazzi_home."/data/pictures/penguin_logo.gif";
  $self->SUPER::populate($args);
  $self->configspec(-variables => [S_SUPER,    S_SUPER,    S_SUPER,  S_SUPER,  S_SUPER, {}]);
}

sub completeinit {
  my ($self) = @_;
  $self->SUPER::completeinit();
  $self->parse_args();
}

sub parse_args {
  my ($self) = @_;
  my $options = {
		 ivy_bus => undef,
		 map => undef,
		 render => undef,
		};
  GetOptions("b=s" => \$options->{ivy_bus},
	     "m=s" => \$options->{map},
	     "r=s" => \$options->{render},
	    );
  my $variables = $self->get('-variables');
  foreach my $var (keys %{$options}) {
    $variables->{$var} = $options->{$var} if defined $options->{$var};
  }
#  print "in paparazzi::parse_args variables after\n".Dumper($variables);
}

sub catchSigTerm() {
  my ($paparazzi) = @_;
  printf("in catchSigTerm\n");
  $paparazzi->terminate_all();
}

my $paparazzi = Paparazzi->new();
$SIG{TERM} = sub {$paparazzi->catchSigTerm()};
Tk::MainLoop();
$paparazzi->catchSigTerm();

1;









