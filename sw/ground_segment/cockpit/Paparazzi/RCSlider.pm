package Paparazzi::RCSlider;
use Subject;
@ISA = ("Subject");

use Tk; 
use Tk::Zinc;
use Math::Trig;

use strict;

use constant CURSOR_WIDTH => 2;
use constant VERTICAL_CONTROL   => 0;
use constant HORIZONTAL_CONTROL => 1;

sub populate {
  my ($self, $args) = @_;
 
  $self->SUPER::populate($args);
  $self->configspec(-zinc      => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width     => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -len       => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -direction => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -name      => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -value     => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->build_gui();
}

sub value() {
  my ($self, $previous_value, $new_value) = @_;
  my $zinc = $self->get('-zinc');
  my $len = $self->get('-len');
  my $new_c = $new_value * $len/2;
  my $cursor_item = $self->{'cursor_item'};
  $zinc->treset($cursor_item);
  if ($self->get('-direction') == VERTICAL_CONTROL) {
    $zinc->translate($cursor_item, 0, $new_c);
  }
  else {
    $zinc->translate($cursor_item, $new_c, 0);
 }
}

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $main_group = $zinc->add('group', 1, -visible => 1);
  $self->{main_group} = $main_group;
  my @origin = $self->get('-origin');
  $zinc->coords($main_group, \@origin);
  my $w = $self->get('-width');
  my $l = $self->get('-len');
  my $d = $self->get('-direction');
  my $name = $self->get('-name');
  my $rectangle_coor = ($d == VERTICAL_CONTROL) ?
    [-$w/2, 0, $w/2, -$l] : [0, -$w/2, $l, $w/2];
  my $cursor_coor = ($d == VERTICAL_CONTROL) ?
    [-$w/2, -$l/2 - CURSOR_WIDTH, $w/2, -$l/2+CURSOR_WIDTH] :
      [$l/2 - CURSOR_WIDTH, -$w/2, $l/2+CURSOR_WIDTH, $w/2];

  $zinc->add('text', $main_group,
	     -position =>[0, 0],
	     -color => 'white',
	     -anchor => ($d == VERTICAL_CONTROL) ? 'n':'e',
	     -text => $name
	    );
  $zinc->add('rectangle', $main_group ,
	      $rectangle_coor,
	      -linewidth => 1,
	      -linecolor => 'black',
	      -filled => 1,
	      -fillcolor => 'white',
	     );
  my $cursor_item = $zinc->add('rectangle', $main_group ,
			       $cursor_coor,
			       -linewidth => 1,
			       -linecolor => 'black',
			       -filled => 1,
			       -fillcolor => 'yellow',
			      );
  $self->{'cursor_item'} = $cursor_item;
  $zinc->bind($cursor_item, '<ButtonPress-1>' => [\&press, $self, \&motion]);
  $zinc->bind($cursor_item, '<ButtonRelease-1>' => [\&release, $self]);
}

my ($x_orig, $y_orig);

sub press {
    my ($zinc, $self, $action) = @_;
    my $ev = $zinc->XEvent();
    my @origin=$zinc->coords($self->{main_group});
    $x_orig = $ev->x - $origin[0];
    $y_orig = $ev->y - $origin[1];
    $zinc->Tk::bind('<Motion>', [$action, $self]);
  }

sub motion {
  my ($zinc, $self) = @_;
  my $ev = $zinc->XEvent();
  my @origin=$zinc->coords($self->{main_group});
  my $x = $ev->x - $origin[0];
  my $y = $ev->y - $origin[1];
  
  if ($self->get('-direction') == VERTICAL_CONTROL) {
    if ($y > -$self->get('-len') and $y < 0) {
      $zinc->translate($self->{cursor_item}, 0, $y-$y_orig);
      $y_orig = $y;
    }
  }
  else {
    if ($x < $self->get('-len') and $x > 0) {
      $zinc->translate($self->{cursor_item}, $x-$x_orig, 0);
      $x_orig = $x;
    }
  }
}

sub release {
  my ($zinc, $self) = @_;
  $zinc->Tk::bind('<Motion>', '');
  my $ev = $zinc->XEvent();
  my @origin=$zinc->coords($self->{main_group});
  my $x = $ev->x - $origin[0];
  my $y = $ev->y - $origin[1];
  my $len = $self->get('-len');
  my $value =  Utils::trim($self->get('-direction') == VERTICAL_CONTROL ?
			   -2 * $y/$len - 1. : 2 * $x/$len - 1, -1., 1.);
  print "slider release ( $value )\n";

}

1;
