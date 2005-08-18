package Paparazzi::SatSigView;
use Subject;
@ISA = ("Subject");
use strict;

use constant MAX_CH => 16;

use Tk; 
use Tk::Zinc;
use Math::Trig;
use Data::Dumper;

use Paparazzi::Utils;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -parent_grp  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -svid =>     [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, undef],
		    -cno  =>     [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, undef],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->build_gui();
}

sub svid {
  my ($self, $old_val, $new_val) = @_;
  return unless defined $new_val;
  $self->get('-zinc')->itemconfigure($self->{-id_lab},  -text => sprintf("%d", $new_val));
}

sub cno {
  my ($self, $old_val, $new_val) = @_;
  return unless defined $new_val;
  $self->get('-zinc')->itemconfigure($self->{-sig_lab}, -text => sprintf("%.1f db", $new_val));
}

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  my $origin = $self->get('-origin');
  my $parent_grp =  $self->get('-parent_grp');
  $self->{main_group} = $zinc->add('group', $parent_grp, -visible => 1);
  $zinc->coords($self->{main_group}, $origin);
  $zinc->add('rectangle',  $self->{main_group}, [0, 0, $width, $height],
	     -visible => 1, 
#	     -fillcolor => 'red', -filled => 1
	     -linecolor => 'white',
	    );
  $self->{-id_lab} = $zinc->add('text', $self->{main_group},
				-position => [2, 1],
				-color => 'white',
				-anchor => 'nw',
			       );
  $self->{-sig_lab} = $zinc->add('text', $self->{main_group},
				 -position => [$width/3, 1],
				 -color => 'white',
				 -anchor => 'nw',
				);
}


1;
