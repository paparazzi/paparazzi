package Paparazzi::CpGui;

use Subject;
use Paparazzi::CpSessionMgr;
@ISA = qw(Paparazzi::CpSessionMgr);

use strict;

use Tk;
use Tk::MainWindow;
use Tk::NoteBook;
use Tk::HList;
use Tk::ItemStyle;
use Data::Dumper;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-logo_file => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, undef],
		    -variables => [S_SUPER,    S_SUPER,    S_SUPER,  S_SUPER,  S_SUPER, undef],
		   );
}
sub completeinit {
  my ($self) = @_;
  $self->SUPER::completeinit();
  $self->build_gui();
}

sub onProgramSelected {
  my ($self, $pgm_name) = @_;
  $self->toggle_program("NONE", $pgm_name, []);
}

sub onSessionSelected {
  my ($self, $session_name) = @_;
  $self->start_session($session_name);
}

use constant LIST_WIDTH => 80;
use constant LIST_HEIGHT => 20;

sub build_gui {
  my ($self) = @_;
  my $mw = MainWindow->new();
  $mw->title ($self->{cp_name});
  # menu bar
  my $menubar =
    $mw->Frame( -relief => 'ridge')->pack(-side => 'top', -fill => 'x');
  my $session_menu = $menubar->Menubutton(-text => 'Sessions')->pack(-side => 'left');;
  my $sessions = $self->get('-sessions');
  foreach my $session_name (keys %{$sessions}) {
    $session_menu->command( -label  => $session_name,
			    -command => [\&onSessionSelected, $self, $session_name] );
  }
  my $program_menu = $menubar->Menubutton(-text => 'Programs')->pack(-side => 'left');;
  my $programs = $self->get(-programs);
  foreach my $pgm_name (keys %{$programs}) {
    $program_menu->command( -label  => $pgm_name,
			    -command => [\&onProgramSelected, $self, $pgm_name]
			  );
  }
  # session frame
  my $session_frame = $mw->Frame( -relief => 'groove')->pack(-side => 'bottom', -fill => 'both', -expand => "yes",);
  my $notebook = $session_frame->NoteBook( -ipadx => 6, -ipady => 6);
  $notebook->pack(-expand => "yes",
		  -fill => "both",
		  -padx => 5, -pady => 5,
		  -side => "top");
  $self->build_logo_page($notebook);
  $self->build_list_page($notebook, "hosts", "Hosts", ["name", "ip", "status"], \&build_hosts_page);
  $self->build_list_page($notebook, "variables", "Variables", ["name", "value"], \&build_variables_page);
#  $self->build_list_page($notebook, "programs", "Programs", ["name", "command", "args"], \&build_programs_page);
  $self->build_programs_page($notebook);
  $self->build_list_page($notebook, "sessions", "Sessions", ["name", "command", "args"], \&build_sessions_page);
#  $self->build_programs_page($notebook);
# my $programs_page = $notebook->add("programs", -label => "Programs", -underline => 0);
#  my $sessions_page = $notebook->add("sessions", -label => "Sessions", -underline => 0);

  $self->{session_frame} = $session_frame;
}


sub build_logo_page {
  my ($self, $notebook) = @_;
  my $logo_filename = $self->get('-logo_file');
  return unless defined $logo_filename;
  my $logo_page = $notebook->add("logo", -label => "Logo", -underline => 0);
  my $image = $logo_page->Photo('logogif',
				-format => 'GIF',
				-file => $logo_filename);
  my $labelImage = $logo_page->Label('-image' => 'logogif')->pack();
  return $logo_page;
}


sub build_hosts_page {
  my ($self, $hlist, $e, $section_h, $item) = @_;
  $hlist->itemCreate ($e, 0,
		      -itemtype => 'text',
		      -text => $item
		     );
  my $ip = $section_h->{$item};
  $hlist->itemCreate ($e, 1,
		      -itemtype => 'text',
		      -text => $ip?$ip:"unknown"
		     );
 $hlist->itemCreate ($e, 2,
		      -itemtype => 'text',
		      -text => "unknown"
		     );
}

sub build_variables_page {
  my ($self, $hlist, $e, $section_h, $item) = @_;
  $hlist->itemCreate ($e, 0, -itemtype => 'text',
		      -text => $item
		     );
  $hlist->itemCreate ($e, 1, -itemtype => 'text',
		      -text => $section_h->{$item},
		     );
}

#sub build_programs_page {
#  my ($self, $hlist, $e, $section_h, $item) = @_;
#  $hlist->itemCreate ($e, 0, -itemtype => 'text',
#		      -text => $item
#		     );
#  $hlist->itemCreate ($e, 1, -itemtype => 'text',
#		      -text => $section_h->{$item}->{command},
#		     );

#  $hlist->itemCreate ($e, 2, -itemtype => 'text',
#		      -text => $section_h->{$item}->{args},
#		     );
#}

sub build_sessions_page {
  my ($self, $hlist, $e, $section_h, $item) = @_;
  $hlist->itemCreate ($e, 0, -itemtype => 'text',
		      -text => $item
		     );
  $hlist->itemCreate ($e, 1, -itemtype => 'text',
		      -text => $section_h->{$item}->{command},
		     );

  $hlist->itemCreate ($e, 2, -itemtype => 'text',
		      -text => $section_h->{$item}->{args},
		     );
}


sub build_programs_page {
  my ($self, $notebook) = @_;
  my $page = $notebook->add("programs", -label => "Programs", -underline => 0);
  my @header = ("name", "command", "args");
  my $hlist = $page->Scrolled ('HList',
#			       -selectmode => 'extended',
			       -header => 1,
#			       -columns => $#header + 1,
			       -width => LIST_WIDTH,
			       -height => LIST_HEIGHT,
			       -itemtype => 'imagetext',
			       -indent => 35,
			       -separator => '/',
			      )->grid(-sticky => 'nsew');
#   for my $i (0 .. $#header) {
#    $hlist->header('create', $i, -text => $header[$i]);
#  }
  my $section_h = $self->get('-programs');
  foreach my $program (keys %{$section_h}) {
#    print Dumper($section_h->{$program})."\n";
    $hlist->add($program,  -text => $program );

    $hlist->add($program."/command",  -text => "command : ".$section_h->{$program}->{command});
    $hlist->add($program."/args",  -text => "args :");
    my $args = $section_h->{$program}->{args};
    foreach my $argh (@{$args}) {
      $hlist->add($program."/args/".$argh->{flag},  -text => $argh->{flag}."\t". $argh->{type}."\t". $argh->{value});
    }
  }
  return $page
}


sub build_list_page {
  my ($self, $notebook, $section, $label, $header, $row_fun) = @_;
  my $page = $notebook->add($section, -label => $label, -underline => 0);
  my @header = @{$header};
  my $hlist = $page->Scrolled ('HList',
			       -header => 1,
			       -columns => $#header + 1,
			       -width => LIST_WIDTH,
			       -height => LIST_HEIGHT,
			      )->grid(-sticky => 'nsew');
  for my $i (0 .. $#header) {
#    print("header $header[$i]\n");
    $hlist->header('create', $i, -text => $header[$i]);
  }
  my $section_h = $self->get('-'.$section);
  print "CpGui variables ".Dumper($section_h) if ($section eq "variables");
  foreach my $item (keys %{$section_h}) {
    my $e = $hlist->addchild("");
    &$row_fun($self, $hlist, $e, $section_h, $item);
 #   print("$hlist, $e, $section_h, $item\n");
  }
  return $page
}

1;










