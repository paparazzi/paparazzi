package Paparazzi::NavPage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;
use Data::Dumper;

use constant TITLE => "Autopilot";

sub populate {
  my ($self, $args) = @_;
  $args->{-title} = TITLE;
  $self->SUPER::populate($args);
  $self->configspec(
		    -ap_status => [S_NOINIT, S_METHOD,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		   );
}

sub ap_status {
  my ($self, $old_val, $new_val) = @_;
  return unless defined $new_val;
#  print "in APPage ap_status\n".Dumper($new_val);
  my $zinc = $self->get('-zinc');
  foreach my $field (keys %{$new_val}) {
     $zinc->itemconfigure ($self->{'text_'.$field},
			   -text => sprintf("%s : %.1f", $field, $new_val->{$field})) if defined $self->{'text_'.$field};
   }
}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my $dy = $self->get('-height')/10;
  my $y=10;
  my $x=10;
  foreach my $field ('mode', 'h_mode', 'v_mode', 'target_climb', 'target_alt', 'target_heading') {
    $self->{'text_'.$field} = $zinc->add('text', $self->{main_group},
					 -position => [$x, $y+$self->{vmargin}],
					 -color => 'white',
					 -anchor => 'w',
					 -text => $field);
    $y+=$dy;
  }
}

