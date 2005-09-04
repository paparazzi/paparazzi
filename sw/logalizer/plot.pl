#!/usr/bin/perl -w
use Getopt::Long;
use Tk;

package Ploter;

use Tk::LabEntry;
use Tk::FileSelect;
use Tk::DialogBox;
use XML::Parser;
use XML::DOM;
use Expect;
use Data::Dumper;
use Getopt::Long;

my $paparazzi_lib;
BEGIN {
  $paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
    $ENV{PAPARAZZI_SRC}."/sw/lib/perl" : "/usr/lib/paparazzi/";
}
use lib ($paparazzi_lib);

#use ChildrenSpawner;
@ISA = qw(Subject);
use strict;
use warnings;
use diagnostics;
use Paparazzi::Environment;

use Subject;


my $log_date;
my $log_duration;
my $log_filename;

my $time_range="[]";
my $tr_entry;

my $pi = 3.14159;

sub populate {
  my ($self, $args) = @_;
  my $paparazzi_src = Paparazzi::Environment::paparazzi_src();
  my $paparazzi_home = Paparazzi::Environment::paparazzi_home();
  Paparazzi::Environment::check_paparazzi_home();
#  $args->{-variables} = {paparazzi_home => $paparazzi_home};		#A quoi ça sert ?
#  $args->{-bin_base_dir} = $paparazzi_src;											#A quoi ça sert ?
#  $self->SUPER::populate($args);																#A quoi ça sert ?
  $self->configspec(-log => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-log_start_date => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-protocol => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-listbox => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-gnuplots => [S_NOINIT, S_PASSIVE, S_RDWR, S_OVRWRT, S_NOPRPG, undef]);
  $self->configspec(-variables => [S_SUPER,    S_SUPER,    S_SUPER,  S_SUPER,  S_SUPER, {}]);
  #  print("in Ploter::populate\n");
}

sub completeinit {
  my $self = shift;
  $self->SUPER::completeinit;
  $self->configure('-protocol' => undef);	#A retirer je suppose
  $self->build_gui();
  #  print("in Ploter::completeinit\n");
}

#
# XML
#
sub parse_messages_xml() {
  my $self = shift;
  my $filename = Paparazzi::Environment::paparazzi_src()."/conf/messages.xml";
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
	$self->{messages} = $doc;
  print STDOUT "successfully parsed $filename\n";
  return  $doc;
}

#
# Menus
#
sub build_menu_msg() {
  my ($self, $doc, $menubar) = @_;
  my $data_menu = $menubar->cascade(-label => "~Data");
	$self->{data_menu} = $data_menu;
}

sub update_menu_msg() {
  my ($self, $ac_name, $ac_id) = @_;
  my $data_menu = $self->{data_menu};
	my $doc = $self->{messages};
	my $menubar = $self->{menubar};
  my $ac_menu = $data_menu->cascade(-label => "$ac_name ($ac_id)");
	$self->parse_protocol($doc, $ac_menu, $ac_id);
}

sub parse_protocol() {
  my ($self, $doc, $ac_menu, $ac_id) = @_;
  my $protocol = $doc->getElementsByTagName('protocol')->[0];
	print "found protocol \n";
	$self->parse_class($protocol, $ac_menu, $ac_id);
}

sub parse_class() {
  my ($self, $protocol, $ac_menu, $ac_id) = @_;
	my $class_name;
  foreach my $class ($protocol->getElementsByTagName('class')) {
    $class_name = $class->getAttribute('name');
		if ($class_name eq "telemetry_ap") {
			print "found telemetry_ap class \n";
			$self->parse_msg($class, $ac_menu, $ac_id);
		}
	}
}

sub parse_msg() {
  my ($self, $class, $ac_menu, $ac_id) = @_;
	my $msg_name;
  foreach my $message ($class->getElementsByTagName('message')) {
    $msg_name = $message->getAttribute('name');
		#	print "found message $msg_name \n";
		my $msg_menu = $ac_menu->cascade(-label => $msg_name);
		$self->build_field_commands($message, $msg_name, $msg_menu, $ac_id);
	}
}

sub build_field_commands() {
  my ($self, $message, $msg_name, $msg_menu, $ac_id) = @_;
  my $no_field = 0;
  foreach my $field ($message->getElementsByTagName('field')) {
    my $field_name = $field->getAttribute('name');
    my $field_unit = $field->getAttribute('unit');
		#	print "found field $field_name \n";
		my $no_field1 = $no_field;
		my $file_menu = $msg_menu->command(-label => $field_name,
																		   -command => sub {on_plot($self, $msg_name, $field_name, $no_field1, $ac_id, $field_unit)});
		$no_field++;
	}
}

sub build_log_menu() {
  my ($self, $mainwindow, $menubar) = @_;
  my $log_menu = $menubar->cascade(-label => "~Log",); 
	my $new_log_menu = $log_menu->command(-label => "~Open new log",
			  	     			 -command => sub { on_new_log($self, $mainwindow)});
#	my $add_log_menu = $log_menu->command(-label => "~Add new log",
#			  	     			 -command => sub { on_add_log($self, $mainwindow)});
}

sub build_gui() {
  my ($self) = @_;
  my $width = 700;
  my $height = 300;
  my $mw = MainWindow->new;
  $mw->geometry(sprintf("%dx%d", $width, $height));
  $mw->title("Paparazzi (gnu)plotter");

  my $mb = $mw->Menu();
	$self->{menubar} = $mb;
  $self->build_log_menu($mw, $mb);
#  my $log_menu = $mb->command(-label => "~Log", 
#			      -command => sub { on_load($self, $mw)});
  $self->build_menu_msg($self->parse_messages_xml(), $mb);
#  $self->build_compiled_msg_menu($mb);
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

  my $button = $mw->Button (-text => "update",
														-command=> sub { update_time_range($self)},
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
  my $gnuplots = $self->get('-gnuplots');
  my $gnuplot = $gnuplots->{$key};
  my $plot_cmd = $gnuplot->{'plot_cmd'};
  my $dialog;
  if ($gnuplot->{'normal'}) {
	  $dialog = $mw->DialogBox( -title   => "Plot command",
				       -buttons => [ "Replot", "Print", "Remove", "Add points", "Cancel" ],
				     ); }
	else {
	  $dialog = $mw->DialogBox( -title   => "Plot command",
				       -buttons => [ "Replot", "Remove", "Cancel" ],
				     ); }
	
  $dialog->add("Label", -text => "Plot command")->pack();
	#	print "plot_cmd $plot_cmd\n";
  my $entry = $dialog->add("Entry", -width => 150)->pack();
  $entry->insert(0,$plot_cmd);
  print "selected key $key\n";
  my $answer = $dialog->Show();
  print "selected $answer\n";
	my ($new_plot_cmd, $exp);
	my $timeout = 1;
  if ($answer eq "Add points") {
    $key =~ /([^\.]+).([^\.]+).([^\.]+).([^\.]+)/ or return;
  	my ($msg_name, $ac_id, $field_name, $field_pos) = ($1, $2, $3, $4);
    my $rpos = $field_pos + 3;

    my $last_plot_cmd = $entry->get();
    my $plot_points_cmd = "\"/tmp/plot_data.$msg_name.$ac_id\" using 1:$rpos w p not";
    $new_plot_cmd = $last_plot_cmd.", ".$plot_points_cmd;
    $entry->configure(-text => $new_plot_cmd);
    
    print("add points to $key \n");
    $gnuplot->{'plot_cmd'} = $new_plot_cmd;
    $exp = $gnuplot->{'exp'};
    $exp->send($new_plot_cmd."\n");
    $exp->expect($timeout);
  }
  if ($answer eq "Replot") {
    $new_plot_cmd = $entry->get();
    print("new_plot_cmd $new_plot_cmd \n");
    $gnuplot->{'plot_cmd'} = $new_plot_cmd;
    $exp = $gnuplot->{'exp'};
#     print "exp $exp\n";
    $exp->send($new_plot_cmd."\n");
    $exp->expect($timeout);
  }
  if ($answer eq "Print") {
    $self->print_plot($key, $entry);
  }
  if ($answer eq "Remove") {
    $self->remove_plot($key);
  }
}

sub update_time_range() {
  my ($self) = @_;
  my $gnuplots = $self->get('-gnuplots');
  $time_range = $tr_entry->get();
  foreach my $key (keys %{$gnuplots}) {
    my $gnuplot = $gnuplots->{$key};
    my $exp = $gnuplot->{'exp'};
		if (defined $exp) {
    	print "update_range_for_key $key ($gnuplots->{$key})\n";
  	  my $plot_cmd = $gnuplot->{'plot_cmd'};
	    print "plot_cmd $time_range [$plot_cmd]\n";
  	  $plot_cmd =~ s/\[.*\]/$time_range/;
    	print "new_plot_cmd $plot_cmd\n\n";
	    $gnuplot->{'plot_cmd'} = $plot_cmd;
	    $exp->send($plot_cmd."\n");
	    my $timeout = 1;
	    $exp->expect($timeout);
		}
  }
}

sub open_log() {
  my ($self, $mw) = @_;
  my $fs = $mw->FileSelect(-directory => Paparazzi::Environment::paparazzi_home()."/var/logs");
  $fs->geometry("600x350");
  my $file_name = $fs->Show();
  if (defined $file_name) {
    print "file_name: $file_name\n";
    $self->load_log($file_name);
  }
}

sub on_plot() {
  my ($self, $msg_name, $field_name, $field_pos, $ac_id, $field_unit) = @_;
  	print "in on_plot msg_name $msg_name, field_name $field_name, field_pos $field_pos, ac_id $ac_id, field_unit $field_unit \n";

  my $key = $msg_name.".".$ac_id.".".$field_name.".".$field_pos;
  $self->gen_data_file($msg_name, $ac_id);
  $self->add_plot($key, $field_unit);
}

sub on_new_log() {
  my ($self, $mainwindow) = @_;
  my $log;
	push (@{$log}, {date=>"", ac_id=>"", type=>"", args=>""});
  $self->configure('-log' => $log);
	$self->open_log($mainwindow);
}

sub load_log() {
  my ($self, $filename) = @_;
  $log_filename = $filename;
  my $nb_lines = 0;
  open(INFILE,  $filename) or die print STDERR "Cant open $filename: $!";
  my $log = $self->get('-log');
  $log_date = undef;
	my $ac_list;
  my $line;
  while ($line = <INFILE>) {     # assigns each line in turn to $_
    if ($line =~ /(^\d+\.\d+) (\d+) (\w+) (.+)/) {
      $log_date = $1 unless defined $log_date;
      my $rel_date = $1 - $log_date;
      push (@{$log}, {date=>$rel_date, ac_id=>$2, type=>$3, args=>$4});
			$self->{ac_list}->{name}->{$2} = 1;
#			push (@{$self->{ac_list}->{$2}}, {date=>$rel_date, type=>$3, args=>$4});
      $nb_lines++;
    }
  }
  close INFILE;
  $self->configure('-log' => $log);
  $self->configure( '-log_start_date' => $log_date);
  $log_duration = "aaa";


  print STDERR "read $nb_lines lines\n";
	$self->parse_conf();
}

sub parse_conf() {
  my ($self) = @_;
  my $filename = Paparazzi::Environment::paparazzi_src()."/conf/conf.xml";
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
	my ($aircraft_name, $aircraft_id);
	my $aircraft_no = 0;
  foreach my $aircraft ($doc->getElementsByTagName('aircraft')) {
    $aircraft_name = $aircraft->getAttribute('name');
    $aircraft_id = $aircraft->getAttribute('ac_id');
		#	print "search aircraft $aircraft_name with id $aircraft_ac_id... \n";
		if (defined $self->{ac_list}->{name}->{$aircraft_id}) {
			print "found aircraft $aircraft_name \n";
			$self->{ac_list}->{name}->{$aircraft_id} = $aircraft_name;
			$aircraft_no++;
			$self->update_menu_msg($aircraft_name, $aircraft_id);
		}
	}
	print "=> found $aircraft_no aircrafts \n";
}

sub add_plot() {
  my ($self, $data_key, $field_unit) = @_;

  $data_key =~ /([^\.]+).([^\.]+).([^\.]+).([^\.]+)/ or return;
  my ($msg_name, $ac_id, $field_name, $field_pos) = ($1, $2, $3, $4);
	my $ac_name = $self->{ac_list}->{name}->{$ac_id};

  my $gnuplots = $self->get('-gnuplots');

  my $exp = new Expect();
  $exp->raw_pty(1);

  my $rpos = $field_pos + 3;
  my $nb_plots = scalar(keys(%{$gnuplots}));
  my $h = $nb_plots * 260;
	my $timeout = 1;
  
  #Do not open again already open keys. Just show it.
  if (defined $gnuplots->{$data_key}) {
    print ("$data_key is already open \n");
    $exp = $gnuplots->{$data_key}->{'exp'};
    $exp->send($gnuplots->{$data_key}->{'plot_cmd'}."\n");
    $exp->expect($timeout);
  }
  #Key is not in current list. Show it.
  else {
    my $plot_cmd = "plot $time_range \"/tmp/plot_data.$msg_name.$ac_id\" using 1:$rpos t \"$ac_name : $field_name ($field_unit)\" w l";
    my $pid = $exp->spawn("/usr/bin/gnuplot", ("-geometry", "1350x200+0+$h", "-title", "$msg_name.$field_name"));
    $pid->log_stdout(0);
    my $gnuplot  =	{	'plot_cmd' => $plot_cmd,
	  	     						'exp' => $exp,
											'normal' => 1
		   							};
    $gnuplots->{$data_key} = $gnuplot;
    $self->configure('-gnuplots' => $gnuplots);
  
    $exp->send($plot_cmd."\n");
    $exp->expect($timeout);
  
    my $listbox = $self->get('-listbox');
    $listbox->insert('end', "$data_key");
  }
}

sub print_plot() {
  my ($self, $data_key, $entry) = @_;

  $data_key =~ /([^\.]+).([^\.]+).([^\.]+).([^\.]+)/ or return;
  my ($msg_name, $ac_id, $field_name, $field_pos) = ($1, $2, $3, $4);
	my $ac_name = $self->{ac_list}->{name}->{$ac_id};

  my $set_terminal_cmd = "set terminal jpeg giant size 1350,400";
  my $set_output_cmd = "set output \"$log_filename\_print/$ac_name.$msg_name.$field_name.jpg\"";
  my $plot_cmd = $entry->get();
  my $print_cmd = "$set_terminal_cmd; $set_output_cmd; $plot_cmd";
  
  mkdir $log_filename."_print/";
  
  my $gnuplots = $self->get('-gnuplots');
  my $exp = new Expect();
  $exp->raw_pty(1);
  
  my $pid = $exp->spawn("/usr/bin/gnuplot", ("-geometry", "1x1+0+0")) or die "Don't find gnuplot";
  $pid->log_stdout(0);
  print("Printing $plot_cmd \n");
  $exp->send($print_cmd."\n");
  my $timeout = 1;
  $exp->expect($timeout);
  $exp->hard_close();
}

sub remove_plot() {
  my ($self, $data_key) = @_;
  my $gnuplots = $self->get('-gnuplots');
  my $gnuplot = $gnuplots->{$data_key};
  my $exp = $gnuplot->{'exp'};
#   $exp->soft_close();
  $exp->hard_close();
  $gnuplots->{$data_key} = undef;
  $self->configure('-gnuplots' => $gnuplots);
  my $listbox = $self->get('-listbox');
  print "remove $data_key\n";
  my $idx = $listbox->index('active');
  $listbox->delete($idx);
}


sub gen_data_file() {
  my ($self, $msg_name, $ac_id) = @_;
  my $nb_msgs = 0; 
  my $tmp_file = "/tmp/plot_data.$msg_name.$ac_id";
  open(OUTFILE, ">".$tmp_file) or die "Can t open $tmp_file: $!";
  foreach (@{$self->get('-log')}) {
#    print "$_->{type} eq $msg_name \n";
    if (($_->{type} eq $msg_name && $_->{ac_id} == $ac_id) || $_->{type} eq " ") {
      print OUTFILE "$_->{date} $_->{type} $_->{args}\n";
      $nb_msgs++;
    }
  } 
  close OUTFILE;
  print STDERR "Number of messages $nb_msgs for $msg_name \n";
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

sub tan { sin($_[0]) / cos($_[0])  }



$SIG{TERM} = \&catchSigTerm ;
#$SIG{KILL} = \&catchSigTerm ;
my $ploter = Ploter->new();
#$ploter->load_log("../../var/log_05_08_04__12_50_09");
Tk::MainLoop();
$ploter->catchSigTerm();
printf STDOUT "ploter over\n";

1;
