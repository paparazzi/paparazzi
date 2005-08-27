package Paparazzi::SettingsPage;
use Paparazzi::NDPage;
@ISA = ("Paparazzi::NDPage");
use strict;
use Subject;
use Data::Dumper;

use constant TITLE => "Settings";

my @fields = ('if_mode', 'if_value1', 'if_value2', 'ap_mode');
my @ap_modes = ('AUTO1', 'AUTO2');
my @if_modes = ('UP', 'DOWN');
my @sliders = ('gain_1', 'gain_2');

sub populate {
  my ($self, $args) = @_;
  $args->{-title} = TITLE;
  $self->SUPER::populate($args);
  $self->configspec(
		    ap_mode       => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 'MANUAL'],
		    if_mode       => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, 'OFF'],
		   );
}

sub set_aircraft {
  my ($self, $prev_ac, $new_ac) = @_;
  foreach my $field  (@fields) {
    $prev_ac->detach($self, $field, [\&update_field]) if ($prev_ac);
    $new_ac->attach($self, $field, [\&update_field]) if ($new_ac);
  }
  my $fligh_plan = $new_ac->get('flight_plan');
  my $rc_controls = $fligh_plan->get('-rc_control');
  my $zinc = $self->get('-zinc');
  foreach my $mode (@ap_modes) {
    foreach my $dir (@if_modes) {
      foreach my $slider (@sliders) {
	my $label = $self->{'label_'.$mode."_".$dir."_".$slider};
	my $text = defined $rc_controls->{$mode}->{$dir}->{$slider} ?
	  $rc_controls->{$mode}->{$dir}->{$slider}->[0] : 'N/A';
	$zinc->itemconfigure($label, -text => $text);
      }
    }
  }
}

sub update_field {
  my ($self, $aircraft, $field, $new_value) = @_;
  my $zinc = $self->get('-zinc');
  my $ap_mode = $aircraft->get('ap_mode');
  my $if_mode = $aircraft->get('if_mode');
  if ($field eq 'ap_mode' or $field eq 'if_mode') {
    $zinc->itemconfigure($self->{'text_'.$field}, -text => $new_value);
    my $old_ap_mode = $self->get('ap_mode');
    my $old_if_mode = $self->get('if_mode');
    foreach my $slider (@sliders) {
      my $label = $self->{'label_'.$old_ap_mode."_".$old_if_mode."_".$slider};
      $zinc->itemconfigure($label, -color => 'white') if defined $label;
      $label = $self->{'value_'.$old_ap_mode."_".$old_if_mode."_".$slider};
      $zinc->itemconfigure($label, -color => 'white') if defined $label;
      $label = $self->{'label_'.$ap_mode."_".$if_mode."_".$slider};
      $zinc->itemconfigure($label, -color => 'green') if defined $label;
      $label = $self->{'value_'.$ap_mode."_".$if_mode."_".$slider};
      $zinc->itemconfigure($label, -color => 'green') if defined $label;
      $self->configure( 'ap_mode' => $ap_mode, 'if_mode' => $if_mode);
      #      print "$ap_mode $if_mode $slider $label\n";
    }
  }
  else {
    $field =~ /if_value(\S)/;
    my $slider = "gain_".$1;
    my $label = $self->{'value_'.$ap_mode."_".$if_mode."_".$slider};
    $zinc->itemconfigure($label, -text => $new_value) if defined $label;
  }
}

sub build_gui {
  my ($self) = @_;
  $self->SUPER::build_gui();
  my $zinc = $self->get('-zinc');
  my $dy = $self->get('-height')/10;
  my $y=10;
  my $x=10;
  my $dx = 70;
  foreach my $field ('ap_mode', 'if_mode') {
    $zinc->add('text', $self->{main_group},
	       -position => [$x, $y+$self->{vmargin}],
	       -color => 'white',
	       -anchor => 'w',
	       -text => $field,
	       -font => $self->{normal_font},
	      );
    $x += $dx;
    $self->{'text_'.$field} = $zinc->add('text', $self->{main_group},
					 -position => [$x, $y+$self->{vmargin}],
					 -color => 'white',
					 -anchor => 'w',
					 -text => 'NA');
    $x += $dx;
  }

  $x = 10;
  $y+=$dy;

  $dx = 100;
  $dy = 35;
  foreach my $ap_mode (@ap_modes) {
    $x = 10;
    $zinc->add('text', $self->{main_group},
	       -position => [$x, $y+$self->{vmargin}],
	       -color => 'white',
	       -anchor => 'w',
	       -text => $ap_mode);
    foreach my $slider (@sliders) {
      $x += $dx;
      $zinc->add('text', $self->{main_group},
		 -position => [$x, $y+$self->{vmargin}],
		 -color => 'white',
		 -anchor => 'w',
		 -text => $slider);
    }
    foreach my $if_mode (@if_modes) {
      $x = 10;
      $y += $dy;
      $zinc->add('text', $self->{main_group},
		 -position => [$x, $y+$self->{vmargin}],
		 -color => 'white',
		 -anchor => 'w',
		 -text => $if_mode);
      foreach my $slider (@sliders) {
	$x += $dx;
	$self->{'label_'.$ap_mode."_".$if_mode."_".$slider} =
	  $zinc->add('text', $self->{main_group},
		     -position => [$x, $y+$self->{vmargin} - 7],
		     -color => 'white',
		     -anchor => 'w',
		     -text => "N/A",
		     -font => $self->{small_font},
		    );
	$self->{'value_'.$ap_mode."_".$if_mode."_".$slider} =
	  $zinc->add('text', $self->{main_group},
		     -position => [$x, $y+$self->{vmargin} + 7],
		     -color => 'white',
		     -anchor => 'w',
		     -text => "N/A",
		     -font => $self->{small_font},
		    );
	
      }
    }
    $y += $dy;
  }
}

