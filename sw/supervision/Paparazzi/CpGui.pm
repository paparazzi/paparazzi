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
		    -variables => [S_SUPER,    S_SUPER,    S_SUPER,  S_SUPER,  S_SUPER, {}],
		   );
}
sub completeinit {
  my ($self) = @_;
  $self->SUPER::completeinit();
  $self->build_gui();
}

sub onProgramSelected {
  my ($self, $pgm_name) = @_;
  $self->toggle_program($pgm_name);
}

sub onSessionSelected {
  my ($self, $session_name) = @_;
  $self->{'session_command_'.$session_name}->configure(-state => 'disabled');
  $self->start_session($session_name);
  $self->add_session_page($session_name);
}

sub onCloseSession {
  my ($self, $session_name) = @_;
  print "on close session\n";
  $self->kill_session($session_name);
  $self->{'session_command_'.$session_name}->configure(-state => 'active');
  $self->remove_session_page($session_name);
}

use constant LIST_WIDTH => 80;
use constant LIST_HEIGHT => 20;


sub onCompile {
  my ($self) = @_;
  my $paparazzi_src = Paparazzi::Environment::paparazzi_src();
  print `cd $paparazzi_src; make`;

}

sub build_gui {
  my ($self) = @_;
  my $mw = MainWindow->new();
  $self->{mw} = $mw;
  $mw->title ($self->{cp_name});
  # menu bar
  my $menubar =
    $mw->Frame( -relief => 'ridge')->pack(-side => 'top', -fill => 'x');
  my $session_menu = $menubar->Menubutton(-text => 'Sessions')->pack(-side => 'left');;
  my $sessions = $self->get('-sessions');
  foreach my $session_name (sort keys %{$sessions}) {
    $self->{'session_command_'.$session_name} =
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
  $self->{notebook} = $notebook;
  $notebook->pack(-expand => "yes",
		  -fill => "both",
		  -padx => 5, -pady => 5,
		  -side => "top");
  $self->build_logo_page($notebook);
#  $self->build_compile_page($notebook);
#  $self->build_list_page($notebook, "hosts", "Hosts", ["name", "ip", "status"], \&build_hosts_page);
#  $self->build_list_page($notebook, "variables", "Variables", ["name", "value"], \&build_variables_page);
#  $self->build_list_page($notebook, "programs", "Programs", ["name", "command", "args"], \&build_programs_page);
#  $self->build_programs_page($notebook);
#  $self->build_list_page($notebook, "sessions", "Sessions", ["name", "command", "args"], \&build_sessions_page);
#  $self->build_programs_page($notebook);
# my $programs_page = $notebook->add("programs", -label => "Programs", -underline => 0);
#  my $sessions_page = $notebook->add("sessions", -label => "Sessions", -underline => 0);

  $self->{session_frame} = $session_frame;

  check_paparazzi_home($mw);

}

sub check_paparazzi_home {
  my ($mw) = @_;
  if (Paparazzi::Environment::paparazzi_home() eq "/usr/share/paparazzi") {
    my $button = $mw->messageBox(
				 -title => 'Welcome to Paparazzi',
				 -type =>  'YesNo',
				 -message => "Paparazzi needs access to a writable directory to store your data and settings. Do you want Paparazzi to use a temporary system directory ?");
    
    my $pph = "";

    if ($button eq "Yes") {
      $pph = `mktemp -d /tmp/paparazzi.XXXXXXXX`;
      print "using dir $pph as paparazzi_home\n"
    }
    else {
      $pph = $mw->chooseDirectory(-title => 'choose a directory',
				  -initialdir => '~/paparazzi'
				      );
#      print "choosed $file_name\n";
      if ($pph eq "") {
	die("need a writable dir. Exiting");
      }
    }
    my $pps = "/usr/share/paparazzi";
    `cd $pps && make -f conf/Makefile.install DESTDIR=$pph install_skel\n`;
  }
}



sub build_logo_page {
  my ($self, $notebook) = @_;
  my $logo_filename = $self->get('-logo_file');
  return unless defined $logo_filename;
  my $logo_page = $notebook->add("logo", -label => "Logo", -underline => 0);
  my $image = $logo_page->Photo('logogif', -format => 'GIF',
				-file => $logo_filename);
  my $labelImage = $logo_page->Label('-image' => 'logogif')->pack();
  return $logo_page;
}

sub build_compile_page {
  my ($self, $notebook) = @_;
  my $compile_page = $notebook->add("compile", -label => "Compile", -underline => 0);
  my $paparazzi_src = Paparazzi::Environment::paparazzi_src();
  my(@pl) = qw/-side top -expand yes -padx 10 -pady 10 -fill both/;
  my $ground_frame = $compile_page->Frame(-label => 'Ground', -borderwidth => 2, - relief =>'groove')->pack(@pl);
#  my $airborne_frame = $compile_page->Frame(-label => "Air", -borderwidth => 2, - relief =>'groove')->pack(@pl);
#  my $ground_frame = $compile_page->Frame(-borderwidth => 2, - relief =>'groove')->pack(@pl);
  my $airborne_frame = $compile_page->Frame(-borderwidth => 2, - relief =>'groove')->pack(@pl);
  @pl = qw/-side top -expand yes -pady 2 -anchor w/;
  my $mode_txt = 'Mode : '. (defined $paparazzi_src ? "Source tree" : "System install");
  my $mode_label = $ground_frame->Label(-text => $mode_txt)->pack(@pl);
  my $paparazzi_src_txt = 'location : '. (defined $paparazzi_src ? $paparazzi_src : "/usr/share/paparazzi");
  my $paparazzi_src_label = $ground_frame->Label(-text => $paparazzi_src_txt)->pack(@pl);
  my $make_button = $ground_frame->Button(
					  -text    => "Compile",
					  -width   => 10,
					  -command => [\&onCompile, $self],
					 );
  $make_button->pack(qw/-side top -expand yes -pady 2/);

  my @header = ("name", "airframe", "radio", "flight plan");
  my $hlist = $airborne_frame->Scrolled ('HList',
				   #			       -selectmode => 'extended',
				   -header => 1,
				   #			       -columns => $#header + 1,
				   -width => LIST_WIDTH,
#				   -height => LIST_HEIGHT,
				   -itemtype => 'imagetext',
				   -indent => 35,
				   -separator => '/',
				  )->grid(-sticky => 'nsew');

  Paparazzi::Environment::read_config();

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
#  print "CpGui variables ".Dumper($section_h) if ($section eq "variables");
  foreach my $item (keys %{$section_h}) {
    my $e = $hlist->addchild("");
    &$row_fun($self, $hlist, $e, $section_h, $item);
 #   print("$hlist, $e, $section_h, $item\n");
  }
  return $page
}

sub add_session_page {
  my ($self, $session_name) = @_;
  my ($mw, $notebook) = ($self->{mw}, $self->{notebook});
  my $page_id = "session_".$session_name;
  my $session_page = $notebook->add($page_id, -label => "$session_name",
				    -underline => 0);
  my $sessions = $self->get('-sessions');
  my $session = $sessions->{$session_name};
  my $hlist = $session_page->Scrolled ('HList',
				       -scrollbars => 'o',
				       -header => 1,
				       -columns => 3,
				       -width => LIST_WIDTH,
				       -height => LIST_HEIGHT,
				       -command => [\&on_session_pgm_clicked, $self, $session_name],
				      )->grid(-columnspan => 2);
  $hlist->header('create', 0, -text => 'name');
  $hlist->header('create', 1, -text => 'status');
  $hlist->header('create', 2, -text => 'args');
  foreach my $i (0..@{$session->{pgms}}-1) {
    my $pgm = $session->{pgms}->[$i];
    my $pgm_name = $pgm->{name};
    print "i $i name : $pgm_name\n";
    $hlist->add($i);
    $hlist->itemCreate($i, 0,   -text => $pgm_name );
    $hlist->itemCreate($i, 1,   -text => 'on' );
    $hlist->itemCreate($i, 2,   -text => 'blah' );
  }
  $self->{$session_name.'hlist'} = $hlist;
#  $session_page->Button( -text => "killall")->grid(-column => 0, -row => 1);
  $session_page->Button( -text => "close",
			 -command => [\&onCloseSession, $self, $session_name])->grid(-column => 1, -row => 1);
  $notebook->raise($page_id);
}

sub remove_session_page {
  my ($self, $session_name) = @_;
  my $notebook = $self->{notebook};
  my $page_id = "session_".$session_name;
  $notebook->delete($page_id);
}

sub on_session_pgm_clicked {
  print "in CpGui::on_session_pgm_clicked @_\n";
  my ($self, $session_name, $pgm_idx) = @_;
  $self->toggle_program_in_session($session_name, $pgm_idx);
  my $text = $self->get_session_program_status($session_name, $pgm_idx) ? 'on' : 'off';
  $self->{$session_name.'hlist'}->itemConfigure($pgm_idx, 1, -text => $text);
}

1;










