package Paparazzi::CpSessionMgr;

use Data::Dumper;
use XML::DOM;
use Subject;

use Paparazzi::CpPgmMgr;
@ISA = qw(Paparazzi::CpPgmMgr);

use strict;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-config_file =>  [S_NEEDINIT, S_PASSIVE,  S_RDONLY, S_OVRWRT, S_NOPRPG, undef],
                    -bin_base_dir => [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, "/usr/bin"],
		    -log_dir =>      [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, "/var/tmp"],
		    -variables =>    [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		    -hosts =>        [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		    -programs =>     [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		    -sessions =>     [S_NOINIT,   S_PASSIVE,  S_RDWR,   S_OVRWRT, S_NOPRPG, {}],
		   );
}

sub completeinit {
  my ($self) = @_;
  $self->SUPER::completeinit();
  my $cfg_file = $self->get('-config_file');
#  my $variables = $self->get('-variables');
#  print "initial variables\n".Dumper($variables);
  $self->read_cfg($cfg_file);
#  $variables = $self->get('-variables');
#  print "configured variables\n".Dumper($variables);
}

sub prepare_args {
  my ($self, $args) = @_;
  my (@options, @rargs);
  my $variables = $self->get('-variables');
#  print "CpSessionMgr : variables ".Dumper($variables);
  foreach my $opt (@{$args}) {
    my $type = $opt->{type};
    my $flag = $opt->{flag};
    my $value = $type eq 'var' ? $variables->{$opt->{value}}: $opt->{value};
    if ($flag) {
      if ($flag =~ /\.*=/) { push @options, $flag.$value}
      else {push @options, $flag, $value}
    }
    else { push @rargs, $value}
  }
  return (@options, @rargs);
}

sub find_binary {
  my ($self, $program) = @_;
  my $command;
  if ($program->{command} =~ /^\/.*/) {
    $command = $program->{command};
  }
  else {
    $command = $self->get('-bin_base_dir')."/".$program->{command};
  }
  return $command;
}

sub toggle_program {
  my ($self, $pgm_name) = @_;
  print "processing program out session context\n";
  my $program = $self->get('-programs')->{$pgm_name};
  if (defined $program->{pid}) {
    $self->SUPER::stop_program($program->{pid});
    $program->{pid} = undef;
  }
  else {
    my (@options, @args) = $self->prepare_args($program->{args});
    my $command = $self->find_binary($program);
    print "starting $program->{name} [$command @options, @args]\n";
    $program->{pid} = $self->SUPER::start_program($command, @options[0..$#options], @args[0..$#args]);
  }
}

sub toggle_program_in_session {
  my ($self, $session_name, $pgm_idx) = @_;
  my $session = $self->get('-sessions')->{$session_name};
  my $session_program = $session->{pgms}->[$pgm_idx];
#  print "processing program in session context\n".Dumper($session_program);
  if (defined $session_program->{pid}) {
    print "killing program $session_program->{name} => $session_program->{pid}\n";
    $self->SUPER::stop_program($session_program->{pid});
    $session_program->{pid} = undef;
  }
  else {
    print "launching program\n";
    my $program_name = $session_program->{name};
    my $program = $self->get('-programs')->{$program_name};
    my (@pgm_options, @pgm_args) = $self->prepare_args($program->{args});
    my (@session_options, @session_args) = $self->prepare_args($session_program->{args});
    push @pgm_options, @session_options;
    push @pgm_args, @session_args;
    my $command = $self->find_binary($program);
    $session_program->{pid} = $self->SUPER::start_program($command, @pgm_options[0..$#pgm_options], @pgm_args[0..$#pgm_args]);
    print "launched $program_name => pid $session_program->{pid}\n";
  }
}

sub start_session {
  my ($self, $session_name) = @_;
#  print "starting session $session_name\n";
  my $sessions = $self->get('-sessions');
  my $session = $sessions->{$session_name};
  my @progs = @{$session->{pgms}};
#  print "progs ".Dumper(@progs);
  foreach my $pgm_idx (0..@progs-1) {
    $self->toggle_program_in_session($session_name, $pgm_idx);
  }
}

sub xml_parse_args {
  my ($args) = @_;
  my @args_a;
  foreach my $arg (@{$args}){
    my $var = $arg->getAttribute('variable');
    my $args_h = {
		  flag => $arg->getAttribute('flag'),
		  type => $var eq '' ? 'const' : 'var',
		  value => $var eq '' ? $arg->getAttribute('constant'): $var,
		 };
    push @args_a, $args_h;
  }
#  print "@args_a\n";
  return \@args_a;
}

sub xml_parse_section {
  my ($self, $section) = @_;
  my $section_name = $section->getAttribute('name');
  my ($items_name) = ($section_name =~ /(.*)s$/);
#  print "section $section_name items_name $items_name\n";
  my $items = $section->getElementsByTagName($items_name);
  my $h_name = '-'.$section_name;
#  print "h_name $h_name\n";
  my $tmp = $self->get($h_name);
  foreach my $item (@{$items}){
    if ($section_name eq "hosts") {
      $tmp->{$item->getAttribute('name')} = $item->getAttribute('ip');
    }
    elsif ($section_name eq 'variables') {
      $tmp->{$item->getAttribute('name')} = $item->getAttribute('value');
    } 
    elsif ($section_name eq 'programs') {
      my $pgm_name = $item->getAttribute('name');
      my $args = $item->getElementsByTagName("arg");
      my $args_h = xml_parse_args($args);
      $tmp->{$pgm_name} =
	{name => $pgm_name,
	 command => $item->getAttribute('command'),
	 args => $args_h,
	};
    }
    elsif ($section_name eq 'sessions') {
      my $session_name = $item->getAttribute('name');
      my $xsessions_pgms = $item->getElementsByTagName("program");
      my @sessions_pgms;
      foreach my $session_pgm (@{$xsessions_pgms}){
	my $pgm_name = $session_pgm->getAttribute('name');
	my $session_args = $session_pgm->getElementsByTagName("arg");
	my $args_h = xml_parse_args($session_args);
	push @sessions_pgms, {
			      name => $pgm_name,
			      args => $args_h
			     };
      }
      $tmp->{$session_name} = {
			       name => $session_name,
			       pgms => \@sessions_pgms
			      };
    }
  }
  $self->configure($h_name => $tmp);
}

sub read_cfg {
  my ($self, $filename) = @_;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  my $cp = $doc->getElementsByTagName("control_panel")->[0];
  $self->{cp_name} = $cp->getAttribute('name');
  my $sections = $cp->getElementsByTagName("section");
  foreach my $section (@{$sections}) {
    $self->xml_parse_section($section);
  }
}


1;
