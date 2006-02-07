#!/usr/bin/perl
use CGI::Form;
use Expect;

my $paparazzi_lib;
BEGIN {
#  $paparazzi_lib = (defined $ENV{PAPARAZZI_SRC}) ?
#    $ENV{PAPARAZZI_SRC}."/sw/lib/perl" : "/usr/lib/paparazzi/";
  $paparazzi_lib = "/home/poine/work/paparazzi_savannah/paparazzi3/sw/lib/perl";
}
use lib ($paparazzi_lib);

use strict;
use warnings;
#use diagnostics;
#use Paparazzi::Environment;
use Paparazzi::Log;

my $PAPARAZZI_HOME = "/home/poine/work/paparazzi_savannah/paparazzi3";

my $query = new CGI::Form;
print $query->header;
print $query->start_html("Paparazzi plotter");
process_query($query);
print_prompt($query);
print_log_info();
print_tail();
print $query->end_html;


my @logs = ();
my @aircrafts = ();
my @messages = ();
my @fields = ();
my $log_info = undef;
my $log_data = undef;



sub print_prompt { 
  my($query) = @_; 
  print "<H1> Paparazzi plotter</H1>\n";
  print $query->startform;
  print "<TABLE>\n";
  print "  <TR>\n";
  print "    <TD>File</TD>";
  print "    <TD>Aircraft</TD>";
  print "    <TD>Message</TD>";
  print "    <TD>Field</TD>";
  print "  </TR>\n";
  print "  <TR>\n";
  print "    <TD>\n";
  print $query->popup_menu('file', \@logs, $logs[0]);
  print "    </TD>\n";
  print "    <TD>\n";
  print $query->popup_menu('aircraft', \@aircrafts, $aircrafts[0]);
  print "    </TD>\n";
  print "    <TD>\n";
  print $query->popup_menu('message', \@messages, $messages[0]);
  print "    </TD>\n";
  print "    <TD>\n";
  print $query->popup_menu('field', \@messages, $messages[0]);
  print "    </TD>\n";
  print "    <TD>\n";
  print $query->submit('Action','Update');
  print "    </TD>\n";
  print "  </TR>\n";
  print "</TABLE>\n";
  print $query->endform;
  print "<HR>\n";
} 

use POSIX qw(strftime);

sub print_log_info {
  if (defined $log_info) {
    my $now_string = POSIX::strftime "%a %b %e %H:%M:%S %Y", localtime($log_info->{date});
    print "date :".$now_string."\n<br>";
    my $url = gen_activity_plot($log_info->{data_file});
    my $nb_messages = $log_data->{nb_messages};
    print "nb message :".$nb_messages."\n<br>";
    my $duration = $log_data->{duration};
    print "duration :".$duration."s\n<br>";
    print"<img src=\"$url\">\n";
#    print "data_file ".$log_info->{data_file}."\n<br>";
  }


}

sub gen_activity_plot {
  my ($data_file) = @_;
  $log_data = Paparazzi::Log::read_data($data_file);
  my $raw_data = $log_data->{raw_data};
  my @res = ();
  my $data_filename = "/tmp/foo.dat";
  open(OUTFILE, ">".$data_filename) or die "Can t open $data_filename: $!";
  my ($idx, $time, $step) = (0, 0, 2);
  my $active_aircrafts;
  while ( $time < $log_data->{duration}) {
    ($idx, $active_aircrafts) = get_active_aircrafts($raw_data, $idx, $step);
    print OUTFILE "$time \t";
    foreach my $a_ac (@{$active_aircrafts}) {
      print OUTFILE $a_ac." ";
    }
    print OUTFILE "\n";
    $time+=$step;
  }
  close OUTFILE;

  my $plot_cmd = "plot \"$data_filename\" using 1:11 w p t \"alalalala\"";
#  for (my $i=2; $i < $#{@$active_aircrafts}+1; $i++) {
  for (my $i=2; $i < 3; $i++) {
    $plot_cmd =  $plot_cmd."; replot \"$data_filename\" using 1:$i w p t \"ouou\"";
  }
  print "$plot_cmd\n<br>";
  my $url = gen_plot("png size 640,480", "bar.png", $plot_cmd);
  return $url;
}

sub get_active_aircrafts {
  my ($raw_data, $idx, $step) = @_;
  my @a_ac = ();
  my $aircrafts = $log_info->{aircrafts};
  for (my $i=0; $i <= $#{@$aircrafts}; $i++) {
    $a_ac[$i] = 0;
  }
  my $start_time = $raw_data->[$idx]->{time};
  my $curtime;
  do {
    $curtime = $raw_data->[$idx]->{time};
    $a_ac[$raw_data->[$idx]->{sender}] = $raw_data->[$idx]->{sender};
    $idx++;
  } while ( $curtime < $start_time + $step and $idx < $#{@$raw_data});
  return ($idx, \@a_ac);
}


sub gen_plot {
  my ($terminal, $filename, $plot_cmd ) = @_;

  my $set_terminal_cmd = "set terminal $terminal";
  my $set_output_cmd = "set output \"$PAPARAZZI_HOME/var/plot/$filename\"";
  my $print_cmd = "$set_terminal_cmd; $set_output_cmd; $plot_cmd";
  my $exp = new Expect();
  $exp->raw_pty(1);
  $Expect::Debug = 10;  
  my $pid = $exp->spawn("/usr/bin/gnuplot", ("-geometry", "1x1+0+0")) or printf "Don't find gnuplot";
  $pid->log_stdout(0);
  print("Printing $print_cmd <br>\n");
  $exp->send($print_cmd."\n");
  my $timeout = 5;
  my $foo = $exp->expect($timeout);
  print "foo $foo<br>\n";
  $exp->hard_close();

  return "http://ornette:8889/var/plot/".$filename;
}



sub process_query {
#  print "in process_query<br>\n";
  my($query) = @_;
  @logs = Paparazzi::Log::get_available();
  my @files = $query->param('file');
  if ($#files >=0 ) {
    my $file = $files[0];
    $log_info = Paparazzi::Log::read_infos($file);

    @aircrafts = @{$log_info->{aircrafts}};
  }
#  my(@values,$key);
#  foreach $key ($query->param) {
#    print "<STRONG>$key</STRONG> -> ";
#    @values = $query->param($key);
#    print join(", ",@values),"<BR>\n";
#  }
#  print "leaving process_query\n";
}




 
sub print_tail {
 print "<HR>\n";
 print "<ADDRESS>Poine.</ADDRESS><BR>\n";
}
