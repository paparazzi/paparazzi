#!/usr/bin/perl -w
use Getopt::Long;
use Tk;

package Ploter;

use Tk::LabEntry;
use Tk::FileSelect;
use Tk::DialogBox;
use XML::Parser;
use Expect;

my $paparazzi_home;
BEGIN {
  $paparazzi_home = "/home/drouin/work/savannah/paparazzi2";
  $paparazzi_home = $ENV{PAPARAZZI_HOME} if defined $ENV{PAPARAZZI_HOME};
}
use lib ($paparazzi_home.'/sw/lib/perl');

#use ChildrenSpawner;
@ISA = qw(Subject);
use strict;
use warnings;
use diagnostics;

use Subject;


my $log_date;
my $log_duration;
my $log_filename;

my $time_range="[]";
my $tr_entry;

sub populate {
  my ($self, $args) = @_;
  $self->SUPER::populate($args);
  $self->configspec(-log => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-log_start_date => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-protocol => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-listbox => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-gnuplots => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  #  print("in Ploter::populate\n");
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->configure('-protocol' => undef);
  $self->build_gui();
  #  print("in Ploter::completeinit\n");
}

#
# XML
#
sub parse_messages_xml() {
  my $filename = $paparazzi_home."/conf/messages.xml";
  my $p = new XML::Parser (Style => 'Tree') ;
  my @msg_xml_tree = $p->parsefile ($filename) ;
  print STDOUT "successfully parsed $filename\n";
  return  \@msg_xml_tree;
}

#
# Menus
#
sub build_menu_msg() {
  my ($self, $xml_top_node, $menubar) = @_;
  my $plot_menu = $menubar->cascade(-label => "~Data");
  while ( defined ( my $element = shift @{ $xml_top_node } )) {
    my $child = shift @{ $xml_top_node };
    if ( ref $child )  {
      my %attr = %{ shift @{ $child } };
      if ($element eq "protocol") {
	print "found protocol\n";
	$self->build_msg_menu($child, $plot_menu);
      }
    }
  }
}

sub build_msg_menu() {
  my ($self, $msg_node, $data_menu) = @_;
#  sort @{$msg_node};
  while ( defined ( my $element = shift @{ $msg_node } )) {
    my $child = shift @{ $msg_node };
    if ( ref $child )  {
      my $attr = shift @{ $child };
      if ($element eq "message") {
	my $id = $attr->{id};
#	print "found message $id\n";
	my $msg_menu = $data_menu->cascade(-label => $id);
	$self->build_field_commands($child, $id, $msg_menu);
      }
    }
  } 
}

sub build_field_commands() {
  my ($self, $fields_node, $msg_name, $msg_menu) = @_;
  my $no_field = 0;
  while ( defined ( my $element = shift @{ $fields_node } )) {
    my $child = shift @{ $fields_node };
    if ( ref $child )  {
      my $attr = shift @{ $child } ;
      if ($element eq "field") {
	my $field_name = $attr->{id};
#	print "found field $field_name\n";
	my $no_field1 = $no_field;
	my $file_menu = $msg_menu->command(-label => $field_name,
					   -command => sub { on_plot($self, $msg_name, $field_name, $no_field1 )});
	$no_field++;
      }
    }
  }
}

sub build_gui() {
  my ($self) = @_;
  my $width = 450;
  my $height = 300;
  my $mw = MainWindow->new;
  $mw->geometry(sprintf("%dx%d", $width, $height));
  $mw->title("Paparazzi (gnu)plotter");

  my $mb = $mw->Menu();
  my $log_menu = $mb->command(-label => "~Log", 
			      -command => sub { on_load($self, $mw)});
  $self->build_menu_msg(@{$self->parse_messages_xml()}, $mb);
  $mw->configure(-menu => $mb);

  my $padx = 10;
  
  my $filename_label = $self->add_label("filename :", \$log_filename, 0, $padx, $mw);
  my $date_label =     $self->add_label("date :", \$log_date, 1, $padx, $mw);
  my $duration_label = $self->add_label("duration :", \$log_duration, 2, $padx, $mw);

  my $time_range_label = $mw->Label( -text => "time range")->pack(-side=>'left');
  $time_range_label->grid (-column=>0, -row=>3, -ipadx=>$padx);
  $tr_entry = $mw->Entry(-width => 25);
  $tr_entry->grid (-column=>1, -row=>3, -ipadx=>$padx);
  $tr_entry->insert(0, $time_range);

  my $button = $mw->Button (-text               => "update",
			    -command            => sub { update_time_range($self)},
			   );
  $button->grid (-column=>2, -row=>3, -ipadx=>$padx);

  my $listbox = $mw->Listbox();
  $listbox->grid (-column=>0, -columnspan => 3, -row=>4, -ipadx=>$padx);
  $listbox->bind('<Double-1>', sub {$self->on_list_clicked($listbox, $mw)});
  $self->configure('-listbox' => $listbox);
}

sub add_label() {
  my ($self, $text, $text_variable, $row, $padx, $mw) = @_;
  my $label1 = $mw->Label( -text => $text);
  $label1->grid (-column=>0, -row=>$row, -ipadx=>$padx, -sticky => 'e' );
  my $label2 = $mw->Label( -textvariable => $text_variable);
  $label2->grid (-column=>1, -row=>$row, -ipadx=>$padx, -sticky => 'w');
  return $label2;
}

sub on_list_clicked() {
  my ($self, $listbox, $mw) = @_;
  my $key = $listbox->get('active');
  my $dialog = $mw->DialogBox( -title   => "Plot command",
			       -buttons => [ "Replot", "Cancel" ],
			     );
  $dialog->add("Label", -text => "Plot command")->pack();
  my $gnuplots = $self->get('-gnuplots');
  my $gnuplot = $gnuplots->{$key};
  my $plot_cmd = $gnuplot->{'plot_cmd'};
  print "plot_cmd $plot_cmd\n";
  my $entry = $dialog->add("Entry", -width => 150)->pack();
  $entry->insert(0,$plot_cmd);
  print "selected key $key\n";
  my $answer = $dialog->Show();
  print "selected $answer\n";
  if ($answer eq "Replot") {
    my $new_plot_cmd = $entry->get();
    print("new_plot_cmd $new_plot_cmd \n");
    $gnuplot->{'plot_cmd'} = $new_plot_cmd;
    my $exp = $gnuplot->{'exp'};
    print "exp $exp\n";
    $exp->send($new_plot_cmd."\n");
    my $timeout = 1;
    $exp->expect($timeout);
  }
}

sub update_time_range() {
  my ($self) = @_;
  my $gnuplots = $self->get('-gnuplots');
  $time_range = $tr_entry->get();
  foreach my $key (keys %{$gnuplots}) {
    print "update_range_for_key $key ($gnuplots->{$key})\n";
    my $gnuplot = $gnuplots->{$key};
    my $plot_cmd = $gnuplot->{'plot_cmd'};
    my $exp = $gnuplot->{'exp'};
    print "plot_cmd $time_range [$plot_cmd]\n";
    $plot_cmd =~ s/\[.*\]/$time_range/;
    print "new_plot_cmd $plot_cmd\n\n";
    $gnuplot->{'plot_cmd'} = $plot_cmd;
    $exp->send($plot_cmd."\n");
    my $timeout = 1;
    $exp->expect($timeout);
  }
}

sub on_load() {
  my ($self, $mw) = @_;
  my $fs = $mw->FileSelect(-directory => $paparazzi_home."/var");
  my $file_name = $fs->Show();
  if (defined $file_name) {
    print "file_name: $file_name\n";
    $self->load_log($file_name);
  }
}

sub on_plot() {
  my ($self, $msg_name, $field_name, $field_pos) = @_;
  #    print "in on_plot msg_name $msg_name field_name $field_name field_pos $field_pos\n";

  my $key = $msg_name.".".$field_name.".".$field_pos;
  $self->gen_data_file($msg_name);
  $self->add_plot($key);
}



sub load_log() {
  my ($self, $filename) = @_;
  $log_filename = $filename;
  my $nb_lines = 0;
  open(INFILE,  $filename) or die print STDERR "Cant open $filename: $!";
  my $log = $self->get('-log');
  $log_date = undef;
  my $line;
  while ($line = <INFILE>) {     # assigns each line in turn to $_
    if ($line =~ /(^\d+\.\d+) (\w+) (.+)/) {
      $log_date = $1 unless defined $log_date;
      my $rel_date = $1 - $log_date;
      push (@{$log}, { date=>$rel_date, type=>$2, args=>$3});
      $nb_lines++;
    }
  }
  close INFILE;
  $self->configure('-log' => $log);
  $self->configure( '-log_start_date' => $log_date);
  $log_duration = "aaa";


  print STDERR "read $nb_lines lines\n"
}



sub add_plot() {
  my ($self, $data_key) = @_;

  $data_key =~ /([^\.]+).([^\.]+).([^\.]+)/ or return;
  my ($msg_name, $field_name, $field_pos) = ($1, $2, $3);

  my $gnuplots = $self->get('-gnuplots');

  my $exp = new Expect();
  $exp->raw_pty(1);

  my $rpos = $field_pos + 3;
  my $nb_plots = scalar(keys(%{$gnuplots}));
  my $h = $nb_plots * 240;
  my $plot_cmd = "plot $time_range \"/tmp/plot_data.$msg_name\" using 1:$rpos t \"$field_name\" w l";
  my $pid = $exp->spawn("/usr/bin/gnuplot", ("-geometry","1600x200+0+$h" ));
  my $gnuplot  = { 'plot_cmd' => $plot_cmd,
		   'exp' => $exp
		 };
  $gnuplots->{$data_key} = $gnuplot;
  $self->configure('-gnuplots' => $gnuplots);

  $exp->send($plot_cmd."\n");
  my $timeout = 1;
  $exp->expect($timeout);

  my $listbox = $self->get('-listbox');
  $listbox->insert('end', "$data_key");

}

sub remove_plot() {
 my ($self, $data_key) = @_;
 my $gnuplots = $self->get('-gnuplots');
 my $gnuplot = $gnuplots->{$data_key};
 my $exp = $gnuplot->{'exp'};
 $exp->soft_close();
 $gnuplots->{$data_key} = undef;
 $self->configure('-gnuplots' => $gnuplots);
 my $listbox = $self->get('-listbox');
#  my $idx = $listbox->index($key);
#  $listbox->delete($idx);
}


sub gen_data_file() {
  my ($self, $msg_name) = @_;
  my $nb_msgs = 0; 
  my $tmp_file = "/tmp/plot_data.$msg_name";
  open(OUTFILE, ">".$tmp_file) or die "Can t open $tmp_file: $!";
  foreach (@{$self->get('-log')}) {
#    print "$_->{type} eq $msg_name \n";
    if ($_->{type} eq $msg_name) {
      print OUTFILE "$_->{date} $_->{type} $_->{args}\n";
      $nb_msgs++;
    }
  } 
  close OUTFILE;
  print STDERR "$nb_msgs $msg_name msgs\n";
}

sub catchSigTerm() {
  my ($self) = @_;
  printf("in catchSigTerm\n");
  my %gnuplots = $self->get('-gnuplots');
  foreach my $key (keys %gnuplots) {
    print ("killing $key  (%gnuplots{$key}->{'pid'})\n");
    $self->kill_gnuplot($key);
  }
}



$SIG{TERM} = \&catchSigTerm ;
#$SIG{KILL} = \&catchSigTerm ;
my $ploter = Ploter->new();
#$ploter->load_log("../../var/log_04_07_01__13_06_33");
Tk::MainLoop();
$ploter->catchSigTerm();
printf STDOUT "ploter over\n";

1;
