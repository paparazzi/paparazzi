package Paparazzi::RCTransmitter;

use strict;

use Tk;
use Tk::Zinc;
use XML::DOM;

use base "Tk::Frame";
use strict;

use Paparazzi::RCSlider;
use Paparazzi::RCStick;

Construct Tk::Widget 'Paparazzi::RCTransmitter';

use constant TYPE_SLIDER  => 0;
use constant TYPE_STICK   => 1;
use constant TYPE_SWITCH  => 2;

use constant VERTICAL_CONTROL   => 0;
use constant HORIZONTAL_CONTROL => 1;

sub ClassInit {
  my ($class, $mw) = @_;
  $class->SUPER::ClassInit($mw);
}

sub Populate {
  my ($self, $args) = @_;
  $self->SUPER::Populate($args);
  $self->ConfigSpecs( -filename => ['PASSIVE', undef, undef, undef],
		      -width    => ['PASSIVE', undef, undef, undef],
		      -height   => ['PASSIVE', undef, undef, undef]);
  $self->{zinc} = $self->Zinc(
#			      -width => $args->{-width}, -height => $args->{-height},
			      -backcolor => 'black',
			      -borderwidth => 3,
			      -relief => 'sunken',
			      -render => '1');
  $self->{main_group} = $self->{zinc}->add('group', 1, -visible => 1);
  
  my $filename = $args->{-filename};
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  foreach my $radio ($doc->getElementsByTagName('radio')) {
    my $name = $radio->getAttribute('name');
    print "name $name\n";
    my $img_filename = $doc->getElementsByTagName('photo')->[0]->getAttribute('filename');
    print "name $img_filename\n";
    my $image = $self->{zinc}->Photo("bg_picture", -file => $img_filename);
    $args->{-width} = $image->width();
    $args->{-height} = $image->height;
    $self->{zinc}->configure(-width => $image->width(), -height => $image->height());

   my $img_item = $self->{zinc}->add('icon', $self->{main_group}, -image => $image);
    foreach my $control ($doc->getElementsByTagName('control')) {
      if ($control->getAttribute('type') eq 'stick') {
	Paparazzi::RCStick->new( -zinc => $self->{zinc}, 
				 -origin => [ $control->getAttribute('x'), $control->getAttribute('y')],
				 -radius => $control->getAttribute('size'),
				 -name => $control->getAttribute('name'),
				 -v_name => $control->getAttribute('v_axe'),
				 -h_name => $control->getAttribute('h_axe')
			       );
      }
      elsif ($control->getAttribute('type') eq 'slider') {
	Paparazzi::RCSlider->new( -zinc => $self->{zinc}, 
				  -origin => [$control->getAttribute('x'), $control->getAttribute('y')],
				  -width => 14, -len => $control->getAttribute('size'), 
				  -direction => $control->getAttribute('direction') eq "horizontal"?
				  HORIZONTAL_CONTROL : VERTICAL_CONTROL,
				  -name => $control->getAttribute('name')
				);
      }
    }
  }
  $self->{zinc}->pack(-fill => 'both', -expand => "1");
}




1;
