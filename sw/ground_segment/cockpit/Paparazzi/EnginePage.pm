package Paparazzi::EnginePage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;

use Tk;
use Tk::Zinc;
use Math::Trig;
use Data::Dumper;

use Paparazzi::Utils;
use Paparazzi::RotaryGauge;

use constant TITLE => "Engine";

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $args->{-title} = TITLE;
  $self->configspec(
		    -engine_status => [S_NOINIT,  S_METHOD, S_RDWR, S_OVRWRT, S_NOPRPG, {}],
		   );
}

sub engine_status {
  my ($self, $old_val, $new_val) = @_;
  my $zinc = $self->get('-zinc');
  foreach my $field (keys %{$new_val}) {
    $self->{'gauge_'.$field}->configure( -value =>  $new_val->{$field}) if defined     $self->{'gauge_'.$field};
  }
}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  use constant GAUGES_PER_ROW => 3;
  my @gauges = (  { name => 'throttle', format => "%.1f %%",  extends => [0  ,  101, 10, 20]},
		 { name => 'rpm',      format => "%.0f rpm", extends => [0  ,  16,  1, 2]},
		 { name => 'temp',     format => "%.1f °C",  extends => [-20, 80, 10, 20]},
		 { name => 'bat',      format => "%.1f V",   extends => [6., 14., 0.5, 1.]},
		 { name => 'amp',      format => "%.1f A",   extends => [0, 16, 1, 2] },
		 { name => 'energy',   format => "%.1f Wh",  extends => [0, 50, 5, 10] },
	       );
  my $margin = 5;
  my $vmargin = 50;
  my $gauge_spacing = ($width - 2 * $margin) / GAUGES_PER_ROW;
  my $gauge_radius = $gauge_spacing / 2. * 0.8;
  my $gauge_row = 0;
  my $gauge_col = 0;
  foreach my $gauge (@gauges) {
    my $extends = $gauge->{extends};
    my $pos = [ $margin + ($gauge_col+0.1) * $gauge_spacing,  $vmargin + $gauge_row * ($gauge_spacing + 30)];
    $self->{'gauge_'.$gauge->{name}} = Paparazzi::RotaryGauge->new(-zinc => $zinc,
								   -radius => $gauge_radius,
								   -origin => $pos,
								   -parent_grp => $self->{main_group},
								   -min_val  => $extends->[0],
								   -max_val => $extends->[1],
								   -tick_spacing => $extends->[2],
								   -legend_spacing => $extends->[3],
								   -min_val_angle => -90.,
								   -dead_sector => 30.,
								   -text => $gauge->{name},
								   -format => $gauge->{format}
								  );
    $gauge_col++;
    unless ($gauge_col lt GAUGES_PER_ROW) {
      $gauge_row++;
      $gauge_col = 0;
    }
  }
}

1;
