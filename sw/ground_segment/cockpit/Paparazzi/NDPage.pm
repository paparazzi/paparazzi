package Paparazzi::NDPage;

use Subject;
@ISA = ("Subject");
use strict;


use Tk;
use Tk::Zinc;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -parent_grp  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -title   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -visible => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, "1"],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->build_gui();
}

sub visible {
  my ($self, $old_val, $new_val) = @_;
  print "in EngineView::visible $new_val\n";
  return unless defined $new_val and defined $self->{main_group};
  my $zinc = $self->get('-zinc');
  $zinc->itemconfigure ($self->{main_group},
			-visible => $new_val,
		       );
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
	     -visible => 1, -linecolor => "green");
  my $vmargin = $self->get('-height')*0.1;
  $self->{vmargin} = $vmargin;
  my $hmargin = 10;
  $zinc->add('text', $self->{main_group},
	     -position => [$hmargin, $vmargin/2],
	     -anchor => 'w',
	     -text => scalar $self->get('-title'),
	     -color => 'white',
	    );
}

1;
