package Paparazzi::CpPgmMgr;

use Subject;
@ISA = ("Subject");

use strict;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-children => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, {}],
		   );
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
}

sub start_program() {
  my ($self, $pgm, @options, @args, $keep_stdin) = @_;
  my %children = %{$self->get('-children')};
  
#  print("in ChildrenSpawner::start_programm args [$pgm @args]\n");
  my $pid = undef;
  my $sleep_count = 0;
  my $fh;
  do {
    $pid = fork();
    $SIG{PIPE} = sub { die "whoops, $pgm pipe broke" };
    unless (defined $pid) {
      warn "cannot fork: $!";
      die "bailing out" if $sleep_count++ > 6;
      sleep 1;
    }
  } until defined $pid;

  if (! $pid) {                # child
    $SIG{TERM} = 'IGNORE';
    exec ($pgm, @options, @args);# or die "couldnt exec foo: $pgm @args";
    # NOTREACHED
    exit(1);
  }
                               # parent
  $children{$pid} = {cmd => $pgm, args => \@args};#, ktw => $fh};
  $self->configure('-children', \%children);
  foreach my $key (keys %children) {
#    print("in ChildrenSpawner::start_programm child: [$key $children{$key}]\n");
  }
  return $pid;
}

sub stop_program() {
  my ($self, $pid) = @_;
#  print "in_stop_program $pid\n";

  my %children = %{$self->get('-children')};
  my $pgm = $children{$pid};
 
  if (defined $pgm) {
#    printf STDOUT "Killing Process %d [%s %s]\n", $pid, $pgm->{cmd}, $pgm->{args};
    kill 9, $pid;
    $children{$pid} = undef;
    $self->configure('-children', \%children);
  }
}


sub terminate_all() {
  my ($self) = @_;
#  print("in ChildrenSpawner::terminate_all\n");
  my %pgms = %{$self->get('-children')};
  foreach my $pid (keys %pgms) {
#    print "killing $pid  ($pgms{$pid})\n";
    $self->stop_program($pid);
  }
}

1;
