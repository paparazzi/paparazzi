# The zone above the artificial horizon which gives informations on ap mode, GPS etc..
#=============================================================================
package Paparazzi::PFD_Panel;
use Subject;
@ISA = ("Subject");

use Tk;
use Tk::Zinc;
use Data::Dumper;

use strict;
sub populate {
  my ($self, $args) = @_;

  $self->SUPER::populate($args);
  $self->configspec(-zinc    => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -parent_grp  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -origin  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -width   => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -height  => [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
		    -pages  =>  [S_NEEDINIT, S_PASSIVE, S_RDONLY, S_OVRWRT, S_NOPRPG, []],
#		    -pubevts => [S_NEEDINIT, S_PASSIVE, S_RDWR, S_APPEND, S_NOPRPG,[]]
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit();

  $self->{pages} = ['Infrared', 'Gps', 'Autopilot', 'Settings', 'Engine'];
  $self->build_gui();
  $self->configure('-pubevts' => 'CLICKED');
}


sub build_gui {
  my ($self) = @_;

  my $zinc = $self->get('-zinc');
  my $parent_grp = $self->get('-parent_grp');
  $self->{main_group} = $zinc->add('group', $parent_grp, -visible => 1);
  my @origin = $self->get('-origin');
  $zinc->coords($self->{main_group}, \@origin);

  my $nb_pages = scalar @{$self->{pages}};
  my $dx =  $self->get('-width') / $nb_pages;

  foreach my $i (0..$nb_pages-1){
    my $page = $self->{pages}->[$i];
    my $button = DigiKit::Button->new(-widget => $zinc,
				      -style => ['Aqualike',
						 -width => $dx,
						 -height => 35,
						 -color => 'green',
						 -text => $page,
						 -trunc => 'both',
						],
				      -position =>  [$i * $dx, 0],
				      -parentgroup => $self->{main_group},
				      -releasecommand => sub { $self->notify('CLICKED', $page)},
				     );
  }

}

sub set_mode {
  my ($self, $name, $previous_val, $new_val) = @_;
#   my $mode = $self->{modes_by_name}->{$name};
#   if (defined $mode) {
#     if (!defined $previous_val || $previous_val != $new_val) {
#       my $zinc = $self->get('-zinc');
#       $zinc->itemconfigure( $mode->{tabular}, 1,
# 			   -text => $mode->{str}[$new_val],
# 			   -color =>$mode->{color}[$new_val],
# 			  );
#     }
#   }
}

sub gps_mode() {
  my ($self, $previous_mode, $new_mode) = @_;
#  $self->set_mode("gps", $previous_mode, $new_mode); 

}

sub ap_mode() {
  my ($self, $previous_mode, $new_mode) = @_;
 # $self->set_mode("ap", $previous_mode, $new_mode);
}

sub rc_mode {
  my ($self, $previous_mode, $new_mode) = @_;
 # $self->set_mode("rc", $previous_mode, $new_mode);
}

sub lls_mode {
  my ($self, $previous_mode, $new_mode) = @_;
 # $self->set_mode("lls", $previous_mode, $new_mode);
}

sub if_mode {
  my ($self, $previous_mode, $new_mode) = @_;
 # $self->set_mode("if", $previous_mode, $new_mode); 
}

sub lls_value {
  my ($self, $previous_val, $new_val) = @_;
#   my $mode = $self->{modes_by_name}->{lls};
#   if (defined $mode) {
#     if (!defined $previous_val || $previous_val != $new_val) {
#       my $zinc = $self->get('-zinc');
#       my $str_val = sprintf ("%.4f", $new_val);
#       $zinc->itemconfigure( $mode->{tabular}, 2,
# 			    -text => $str_val,
# 			    -color => "green",
# 			  );
#     }
#   }
}



sub ctrst_mode {
  my ($self, $previous_mode, $new_mode) = @_;
#  $self->set_mode("ctrst", $previous_mode, $new_mode);
}

sub ctrst_value {
  my ($self, $previous_val, $new_val) = @_;
#   my $mode = $self->{modes_by_name}->{ctrst};
#   if (defined $mode) {
#     if (!defined $previous_val || $previous_val != $new_val) {
#       my $zinc = $self->get('-zinc');
#       my $str_val = sprintf ("%.4f", $new_val);
#       $zinc->itemconfigure( $mode->{tabular}, 2,
# 			    -text => $str_val,
# 			    -color => "green",
# 			  );
#     }
#   }
}


sub setLabelContent {
  my ($self, $item, $labelformat) = @_;

  my @fieldsSpec = split (/ / , $labelformat);
  shift @fieldsSpec;

  my $i=0;
  foreach my $fieldSpec (@fieldsSpec) {
    my ($posSpec) = $fieldSpec =~ /^.\d+.\d+(.*)/ ;
    print "$fieldSpec\t$i\t$posSpec\n";
    $self->{zinc}->itemconfigure ($item,$i,
				  -text => "$i: $posSpec",
				  -border => "contour",
				  -color => 'green',
				  -backcolor => 'white',
				  -filled => 1
				 );
    $i++;
  }
}

1;
