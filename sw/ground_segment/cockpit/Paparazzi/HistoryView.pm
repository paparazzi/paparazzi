package Paparazzi::HistoryView;

use strict;
use Carp;
use vars qw(@ISA);

use Subject;
@ISA = qw(Subject);

use POSIX;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(
		    -zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -parent_grp  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -nb_bars => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -initial_range => [S_NEEDINIT,  S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, 1.],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->{x_scrolling_val} = 0;
  $self->{bar_width} = POSIX::floor($self->get('-width') / $self->get('-nb_bars'));
  $self->{bars} = [];
  $self->{nb_visible_bars} = 0;
  $self->{scale} = $self->get('-height') / $self->get('-initial_range');
  $self->build_gui();
}

sub put_value {
  my ($self, $val) = @_;
  return unless defined $val and defined $self->{moving_group};
  if ($self->{nb_visible_bars} >= $self->get('-nb_bars')) {
    my $rect = shift @{$self->{bars}};
    $self->get('-zinc')->remove($rect);
#    print "removing $rect\n";
  }
  else {
    $self->{nb_visible_bars}++;
  }
  my $height = $self->get('-height');
  my $h = $val * $self->{scale};
  my $rect = [$self->{x_scrolling_val}, $height-$h, 
	      $self->{x_scrolling_val}+$self->{bar_width}, $height];
  my $bar = $self->get('-zinc')->add('rectangle',  $self->{moving_group}, $rect,
				     -filled => 1,
				     -fillcolor => 'gray60',
				     -visible => 1);
  push @{$self->{bars}}, $bar;
  $self->{x_scrolling_val}+=$self->{bar_width};
  if ($self->{x_scrolling_val} > $self->get('-width')) {
    $self->get('-zinc')->translate($self->{moving_group}, -$self->{bar_width}, 0);
  }
}

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $origin = $self->get('-origin');
  my $parent_grp =  $self->get('-parent_grp');
  my $height = $self->get('-height');
  my $width = $self->get('-width');
  $self->{main_group} = $zinc->add('group', $parent_grp, -visible => 1);
  $zinc->coords($self->{main_group}, $origin);

  $self->{clipping_group} = $zinc->add('group',$self->{main_group}, -visible => 1);
  $self->{itemclip} = $zinc->add('rectangle',  $self->{clipping_group}, [0, 0, $width, $height],
				 -visible => 0);
  $zinc->itemconfigure($self->{clipping_group}, -clip => $self->{itemclip});
  $self->{moving_group} = $zinc->add('group',$self->{clipping_group}, -visible => 1);
  $zinc->add('rectangle',  $self->{main_group}, [0, 0, $width, $height],
	     -visible => 1, -linecolor => "green");
}
