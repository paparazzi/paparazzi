package Paparazzi::SettingsPage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;
use Data::Dumper;

use constant TITLE => "Settings";

my @fields = ('if_mode', 'ap_mode');

sub populate {
  my ($self, $args) = @_;
  $args->{-title} = TITLE;
  $self->SUPER::populate($args);
  $self->configspec(
		   );
}



sub set_aircraft {
  my ($self, $prev_ac, $new_ac) = @_;
  foreach my $field  (@fields) {
    $prev_ac->detach($self, $field, [\&update_field]) if ($prev_ac);
    $new_ac->attach($self, $field, [\&update_field]) if ($new_ac);
  }
  my $fligh_plan = $new_ac->get('flight_plan');
  #  use Data::Dumper;
  #  print "#####".Dumper(scalar $fligh_plan->get('-rc_control'));
  my $zinc = $self->get('-zinc');
  my $rc_controls = $fligh_plan->get('-rc_control');
  foreach my $mode (keys %{$rc_controls}) {
    foreach my $dir (keys %{$rc_controls->{$mode}}) {
      foreach my $slider (keys %{$rc_controls->{$mode}->{$dir}}) {
	my $label = $self->{'label_'.$mode."_".$dir."_".$slider};
	$zinc->itemconfigure($label, -text => $rc_controls->{$mode}->{$dir}->{$slider}->[0]);
      }
    }
  }
}

sub update_field {
  my ($self, $aircraft, $field, $new_value) = @_;
  if ($field eq 'ap_mode' or $field eq 'if_mode') {
    $self->get('-zinc')->itemconfigure($self->{'text_'.$field}, -text => $new_value);
  }
#  else {
  my $zinc = $self->get('-zinc');
  my $ap_mode = $aircraft->get('ap_mode');
  my $if_mode = $aircraft->get('if_mode');
  foreach my $slider ('gain_1', 'gain_2') {
    my $label = $self->{'label_'.$ap_mode."_".$if_mode."_".$slider};
    $zinc->itemconfigure($label, -color => 'green');
    print "$ap_mode $if_mode $slider $label\n";
  }
 #    $field =~ /if_value(\S)/;
 #   my $slider = "GAIN_".$1;
 #   my $label = $self->{'label_'.$ap_mode."_".$if_mode."_".$slider};
 #   print "$ap_mode $if_mode $slider $label\n";

#  }
}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my $dy = $self->get('-height')/10;
  my $y=10;
  my $x=10;
  foreach my $field (@fields) {
    $zinc->add('text', $self->{main_group},
	       -position => [$x, $y+$self->{vmargin}],
	       -color => 'white',
	       -anchor => 'w',
	       -text => $field);
    $self->{'text_'.$field} = $zinc->add('text', $self->{main_group},
					 -position => [$x + 100, $y+$self->{vmargin}],
					 -color => 'white',
					 -anchor => 'w',
					 -text => 'NA');
    $y+=$dy;
  }

  my @ap_modes = ('AUTO1', 'AUTO2');
  my @if_modes = ('UP', 'DOWN');
  my @sliders = ('gain_1', 'gain_2');
  foreach my $ap_mode (@ap_modes) {
    $x = 10;
    $zinc->add('text', $self->{main_group},
	       -position => [$x, $y+$self->{vmargin}],
	       -color => 'white',
	       -anchor => 'w',
	       -text => $ap_mode);
    foreach my $slider (@sliders) {
      $x += 100;
      $zinc->add('text', $self->{main_group},
		 -position => [$x, $y+$self->{vmargin}],
		 -color => 'white',
		 -anchor => 'w',
		 -text => $slider);
    }
    foreach my $if_mode (@if_modes) {
      $x = 10;
      $y += 35;
      $zinc->add('text', $self->{main_group},
		 -position => [$x, $y+$self->{vmargin}],
		 -color => 'white',
		 -anchor => 'w',
		 -text => $if_mode);
      foreach my $slider (@sliders) {
	$x += 100;
	$self->{'label_'.$ap_mode."_".$if_mode."_".$slider} =
	  $zinc->add('text', $self->{main_group},
		     -position => [$x, $y+$self->{vmargin}],
		     -color => 'white',
		     -anchor => 'w',
		     -text => "N/A",
		     -font => $self->{small_font},
		    );
      }
    }
    $y += 35;
  }
}

