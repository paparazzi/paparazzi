package Paparazzi::RCStick;
use Subject;
@ISA = ("Subject");

use Paparazzi::RCSlider;
use Paparazzi::Utils;

use Tk; 
use Tk::Zinc;
use Math::Trig;

use strict;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc      => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -radius    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -name      => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -v_name    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -h_name    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -v_value   => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
		    -h_value   => [S_NOINIT,   S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->build_gui();
  $self->connectoptions('-v_value', S_TO, [$self->{v_slider}, '-value']);
  $self->connectoptions('-h_value', S_TO, [$self->{h_slider}, '-value']);
}

sub h_value() {
  my ($self, $previous_value, $new_value) = @_;
#  print "in h_value $new_value\n";
}

sub v_value() {
  my ($self, $previous_value, $new_value) = @_;
}

use constant WIDTH => 14;

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $radius =  $self->get('-radius');
  my @origin = $self->get('-origin');
  my @v_origin = ($origin[0], $origin[1] + $radius);
  my @h_origin = ($origin[0] - $radius, $origin[1]);
  my $v_name = $self->get('-v_name');
  my $h_name = $self->get('-h_name');

  $self->{v_slider} =  Paparazzi::RCSlider->new( -zinc => $zinc, -origin => \@v_origin,
						 -width => WIDTH, -len => 2 * $radius,
						 -direction => Paparazzi::RCSlider::VERTICAL_CONTROL,
						 -name => $v_name
					       );
  $self->{h_slider} =  Paparazzi::RCSlider->new( -zinc => $zinc, -origin => \@h_origin,
						 -width => WIDTH, -len => 2 * $radius,
						 -direction => Paparazzi::RCSlider::HORIZONTAL_CONTROL,
						 -name => $h_name
					       );

  my $main_group = $zinc->add('group', 1, -visible => 1);
  $zinc->coords($main_group, \@origin);
  $self->{main_group} = $main_group;


  my $cursor_coor = [ - WIDTH/2, - WIDTH/2, WIDTH/2, WIDTH/2 ];

  my $cursor_item = $zinc->add('arc', $main_group,
			       $cursor_coor,
			       -filled => 1,
			       -fillcolor => "yellow",
			       -linewidth => 1,
			       -linecolor => "black",
			       -startangle => 0, -extent => 360,
			       -pieslice => 1, -closed => 1,
			      );
  $self->{'cursor_item'} = $cursor_item;
  $zinc->bind($cursor_item, '<ButtonPress-1>' => [\&press, $self, \&motion]);
  $zinc->bind($cursor_item, '<ButtonRelease-1>' =>[\&release, $self]);
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
  my $radius = $self->get('-radius');

  if ($y > -$radius and $y < $radius) {
    $zinc->translate($self->{cursor_item}, 0, $y-$y_orig);
    $y_orig = $y;
  }
  if ($x > -$radius and $x < $radius) {
    $zinc->translate($self->{cursor_item}, $x-$x_orig, 0);
    $x_orig = $x;
  }
}

sub release {
  my ($zinc, $self) = @_;
  $zinc->Tk::bind('<Motion>', '');
  my $ev = $zinc->XEvent();
  my @origin=$zinc->coords($self->{main_group});
  my $x = $ev->x - $origin[0];
  my $y = $ev->y - $origin[1];
  my $radius = $self->get('-radius');
  my ($v_value, $h_value) = (Utils::trim($y/$radius, -1., 1.), Utils::trim($x/$radius, -1, 1));
  print "stick release ( $v_value, $h_value ) \n";
  $self->configure('-v_value' => $v_value, '-h_value' => $h_value);
}

1;
