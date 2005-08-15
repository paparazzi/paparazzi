package Paparazzi::StripPanel;
use Subject;
@ISA = ("Subject");

use strict;

use Math::Trig;
use Tk; 
use Tk::Zinc;

use Paparazzi::Strip;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -selected_ac  => [S_NOINIT, S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 'NONE'],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->build_gui();
}

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  my $origin = $self->get('-origin');

  $self->{sp_main_group} = $zinc->add('group', 1, -visible => 1);
  $zinc->coords($self->{sp_main_group}, $origin);

  my $board = $zinc->add('rectangle', $self->{sp_main_group} ,
			 [0, 0, $width-5, $height-7],
			 -linewidth => 0,
			 -filled => 1,
			 -fillcolor => 'blue',
			);
  my $texture =  $zinc->Photo('background_texture.gif',
			      -file => Tk->findINC("demos/zinc_data/background_texture.gif"));
  $zinc->itemconfigure($board, -tile => $texture);
  $self->{strips} = {};

}

sub add_strip {
  my ($self, $aircraft) = @_;
  # add strip only once
  return if (defined $self->{strips}->{$aircraft->get('-ac_id')});
  my $zinc = $self->get('-zinc');
  use constant NB_STRIP => 6;
  my $step = $self->get('-height') / NB_STRIP;
  my $nb_strips = keys %{$self->{strips}};
  my ($p, $w, $h) = ([5, 10 + $step * $nb_strips], 120, 45);
  my $strip = Paparazzi::Strip->new( -zinc => $zinc, -parent_grp => $self->{sp_main_group},
				     -origin => $p, -width  => $w, -height => $h,
				     -aircraft => $aircraft);
  $self->{strips}->{$aircraft->get('-ac_id')} = $strip;
#  $zinc->bind($self->{strips}->{$name}->{-paper},'<ButtonPress-1>',[\&OnStripPressed,$self, $name]);
}

sub OnStripPressed {
#  print ("OnStripPressed @_\n");
  my ($zinc, $self, $name) = @_;
  $self->configure( -selected_ac => $name);
}

sub selected_ac {
  my ($self, $previous, $new) = @_;
  $self->{strips}->{$previous}->configure( -selected => 0) if defined $previous and defined $self->{strips}->{$previous};
  $self->{strips}->{$new}->configure( -selected => 1) if defined $new and $new ne "NONE" and defined $self->{strips}->{$new};
}

1;
