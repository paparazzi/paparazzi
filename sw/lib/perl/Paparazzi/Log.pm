package Paparazzi::Log;

use strict;

use XML::Parser;
use XML::DOM;
use Paparazzi::Environment;
use Paparazzi::Configuration;
#use Paparazzi::Ploter;


sub get_available {
  my $log_dir = Paparazzi::Environment::paparazzi_home()."/var/logs";
  opendir(DIR, $log_dir) || print "can't opendir $log_dir: $!\n";
  my @logs = grep { /^.*\.log/ && -f "$log_dir/$_" } readdir(DIR);
  closedir DIR;
  return @logs;
}

sub read_infos {
  my ($filename) = @_;
  $filename = Paparazzi::Environment::paparazzi_home()."/var/logs"."/".$filename;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);

  my $configuration = $doc->getElementsByTagName('configuration')->[0];
  my $timeofday = $configuration->getAttribute('time_of_day');
  my $data_file = $configuration->getAttribute('data_file');

  my $conf = $doc->getElementsByTagName('conf')->[0];
  my $configuration = Paparazzi::Configuration::parse($conf);
  my $protocol = $doc->getElementsByTagName('protocol');
  my $prt = parse_protocol($protocol);
  return { date => $timeofday, data_file => $data_file , configuration => $configuration };
}

sub read_data {
  my ($filename) = @_;
  $filename = Paparazzi::Environment::paparazzi_home()."/var/logs"."/".$filename;
  my @data = ();
  open(INFILE,  $filename) or print STDOUT "Cant open $filename: $!";
  while (my $line = <INFILE>) {
    if ($line =~ /(^\d+\.\d+) (\d+) (\w+) (.+)/) {
      push @data, {time => $1, sender => $2, msg_id => $3, args => $4};
    }
  }
  close INFILE;
  my $duration = $#data >=0 ? @data[$#data]->{time} : 0;
  return {nb_messages => $#data+1, duration => $duration, raw_data => \@data};
}

use POSIX qw(strftime);

sub html_print_summary {
  my ($log_info, $log_data) = @_;
  my $now_string = POSIX::strftime "%a %b %e %H:%M:%S %Y", localtime($log_info->{date});
  print "date : ".$now_string."\n<br>";
  my $nb_messages = $log_data->{nb_messages};
  print "nb message : ".$nb_messages."\n<br>";
  my $duration = $log_data->{duration};
  print "duration : ".$duration."s\n<br>";
}


sub gen_activity_plot {
  my ($log_info, $log_data) = @_;
  my $raw_data = $log_data->{raw_data};
  my @res = ();
  my $data_filename = "/tmp/foo.dat";
  open(OUTFILE, ">".$data_filename) or die "Can t open $data_filename: $!";
  my ($idx, $time, $step) = (0, 0, 2);
  my $active_aircrafts;
  while ( $time < $log_data->{duration}) {
    ($idx, $active_aircrafts) = get_active_aircrafts($log_info, $raw_data, $idx, $step);
    print OUTFILE "$time \t";
    foreach my $a_ac (@{$active_aircrafts}) {
      print OUTFILE $a_ac." ";
    }
    print OUTFILE "\n";
    $time+=$step;
  }
  close OUTFILE;
  my $ac = $log_info->{configuration}->{aircrafts}->[0]->{name};
  my $plot_cmd = "plot \"$data_filename\" using 1:2 w p t \"$ac\"";
  for (my $i=3; $i < $#{@$active_aircrafts}+3; $i++) {
    $ac = $log_info->{configuration}->{aircrafts}->[$i-2]->{name};
    $plot_cmd =  $plot_cmd.", \"$data_filename\" using 1:$i w p t \"$ac\"";
  }

  my $url = Paparazzi::Ploter::gen_plot("png size 800,240", "bar.png", $plot_cmd);
  return $url;
}

sub get_active_aircrafts {
  my ($log_info, $raw_data, $idx, $step) = @_;
  my @a_ac = ();
  my $aircrafts = $log_info->{configuration}->{aircrafts};
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


sub parse_protocol {


}






1;
