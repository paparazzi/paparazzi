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
		    -selected => [S_NOINIT,  S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef],
		    -pubevts   => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_APPEND, S_NOPRPG,[]],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->configure('-pubevts' => 'SELECTED');
  $self->build_gui();
}

use constant H_MARGIN => 5;
use constant V_MARGIN => 7;

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  my $origin = $self->get('-origin');

  $self->{sp_main_group} = $zinc->add('group', 1, -visible => 1);
  $zinc->coords($self->{sp_main_group}, $origin);

  my $board = $zinc->add('rectangle', $self->{sp_main_group} ,
			 [0, 0, $width - H_MARGIN , $height - V_MARGIN],
			 -linewidth => 0,
			 -filled => 1,
			 -fillcolor => 'blue',
			);
  my $texture =  $zinc->Photo('background_texture.gif',
			      -file => Tk->findINC("demos/zinc_data/background_texture.gif"));
  $zinc->itemconfigure($board, -tile => $texture);
  $self->{strips} = {};

}

use constant NB_STRIP => 3;

sub add_strip {
  my ($self, $aircraft) = @_;
  # add strip only once
  return if (defined $self->{strips}->{$aircraft->get('-ac_id')});
  my $zinc = $self->get('-zinc');
  my $strip_vspacing = ($self->get('-height') - 2 * V_MARGIN) / NB_STRIP;
  my $strip_width = $self->get('-width') - H_MARGIN;
  my $strip_height = $strip_vspacing;
  my $nb_strips = keys %{$self->{strips}};
  my ($p, $w, $h) = ([H_MARGIN, V_MARGIN + $strip_vspacing * $nb_strips], $strip_width, $strip_height);
  my $strip = Paparazzi::Strip->new( -zinc => $zinc, -parent_grp => $self->{sp_main_group},
				     -origin => $p, -width  => $w, -height => $h,
				     -aircraft => $aircraft);
  my $ac_id = $aircraft->get('-ac_id');
  $zinc->bind($strip->{'frame'},'<ButtonPress-1>',[\&OnStripPressed,$self, $ac_id]);
  $self->{strips}->{$ac_id} = $strip;
  if ($nb_strips == 0) {
    OnStripPressed($zinc, $self, $ac_id);
  }
}

sub OnStripPressed {
#  print ("OnStripPressed @_\n");
  my ($zinc, $self, $ac_id) = @_;
  $self->{strips}->{$self->get('-selected')}->configure('-selected' => 0) if defined $self->get('-selected');
  $self->configure('-selected' => $ac_id);
  $self->{strips}->{$ac_id}->configure('-selected' => 1);
  $self->notify('SELECTED', $ac_id);
}


1;
