package Paparazzi::Strip;
use Subject;
@ISA = ("Subject");

use Data::Dumper;

use strict;

use Math::Trig;
use Tk; 
use Tk::Zinc;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -parent_grp  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -name  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -selected => [S_NOINIT, S_METHOD, S_RDWR, S_OVRWRT, S_NOPRPG, 0],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();
  $self->build_gui();
}

my $style = {
	     -linewidth => 3,
	     -linecolor => '#aaccff',
	     -fillcolor => 'back',
	     -relief => 'roundraised'
	    };

my $gradset = {
	       'idnt' => '=axial 90 |#ffffff 0|#ffeedd 30|#e9d1ca 90|#e9a89a',
	       'back' => '=axial 0 |#c1daff|#8aaaff',
	       'shad' => '=path -40 -40 |#000000;50 0|#000000;50 92|#000000;0 100',
	       'btn_outside' => '=axial 0 |#ffeedd|#8a9acc', 
	       'btn_inside' => '=axial 180 |#ffeedd|#8a9acc',
	       'ch1' => '=axial 0 |#8aaaff|#5B76ED',
	      };

my @stripGradiants;

sub init_gradiants {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  unless (@stripGradiants) {
    my %gradiants = %{$gradset};
    my ($name, $gradiant);
    while (($name, $gradiant) = each(%gradiants)) {
      # création des gradients nommés
      $zinc->gname($gradiant, $name) unless $zinc->gname($gradiant);
      # the previous test is usefull only
      # when this script is executed many time in the same process
      # (it is typically the case in zinc-demos)
      push(@stripGradiants, $name);
    }  
  }
}

sub build_gui {
  my ($self) = @_;
  my $zinc = $self->get('-zinc');
  my $width = $self->get('-width');
  my $height = $self->get('-height');
  my $origin = $self->get('-origin');
  my $parent_grp = $self->get('-parent_grp');

  $self->init_gradiants();

  $self->{s_main_group} = $zinc->add('group', $parent_grp, -visible => 1);
  $zinc->coords($self->{s_main_group}, $origin);

  my $ombre = $zinc->add('rectangle', $self->{s_main_group} ,
			 [5, 5, $width+5, $height+5],
			 -filled => 1,
			 -linewidth => 0,
			 -fillcolor => 'shad',
			 -priority => 10,
			);

  $self->{-paper} = $zinc->add('rectangle', $self->{s_main_group} ,
			       [0, 0, $width, $height],
			       -filled => 1,
			       -linewidth => $style->{'-linewidth'},
			       -linecolor => $style->{'-linecolor'},
			       -fillcolor => $style->{'-fillcolor'},
			       -relief => $style->{'-relief'},
			       -priority => 100
			      );

#  $zinc->bind ($self->{-paper},'<ButtonPress-1>',[\&OnButton1PressPaper,$self]);
#  $zinc->bind ($self->{-paper},'<ButtonRelease-1>',[\&OnButton1ReleasePaper,$self]);
  
#  my $texture =  $zinc->Photo('background_texture.gif',
#			   -file => Tk->findINC("demos/zinc_data/background_texture.gif"));
 # $zinc->itemconfigure($foo, -tile => $texture);
  my $text = $self->get('-name');

  $zinc->add('text', $self->{s_main_group},
	     -position => [10, 10],
	     -color => 'white',
#	     -font => $v_tick_font,
	     -anchor => 'w',
	     -text => $text,
	     -priority => 110);
  
}

sub selected {
  my ($self, $previous, $new) = @_;
#  print ("in selected $self->get('-name') $previous, $new\n");
  $self->get('-zinc')->itemconfigure ($self->{-paper},
				      -fillcolor => $new != 0 ? 'ch1':  $style->{'-fillcolor'},
				     );

}


sub OnButton1PressPaper {
  my ($zinc, $self) = @_;
  $zinc = $self->get('-zinc');

  $zinc->itemconfigure ($self->{-paper},
			-fillcolor => 'ch1',
		       );
  
}

sub OnButton1ReleasePaper {
  my ($zinc, $self) = @_;
  $zinc = $self->get('-zinc');

  $zinc->itemconfigure ($self->{-paper},
			-fillcolor => $style->{'-fillcolor'}
		       );


}

1;
