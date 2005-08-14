package Paparazzi::Aircraft;

use Subject;
@ISA = ("Subject");
use strict;


sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(
                    -ac_id           => [S_NEEDINIT, S_PASSIVE,  S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
                    -callsign     => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, ""],
                    -ssr          => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, "UNKNOWN"],
                    -sector       => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, "UNKNOWN"],
                    -layers       => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, "UNKNOWN"],
                    -pos_x        => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -pos_y        => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -v_x          => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -v_y          => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -afl          => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
                    -rate         => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 0.],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
}

1;


