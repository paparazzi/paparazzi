package Paparazzi::Log;

use strict;

use XML::Parser;
use XML::DOM;

my $PAPARAZZI_HOME = "/home/poine/work/paparazzi_savannah/paparazzi3";

sub get_available {
  my $log_dir = $PAPARAZZI_HOME."/var/logs";
  opendir(DIR, $log_dir) || print "can't opendir $log_dir: $!\n";
  my @logs = grep { /^.*\.log/ && -f "$log_dir/$_" } readdir(DIR);
  closedir DIR;
  return @logs;
}

sub read_infos {
  my ($filename) = @_;
  $filename = $PAPARAZZI_HOME."/var/logs"."/".$filename;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);

  my $configuration = $doc->getElementsByTagName('configuration')->[0];
  my $timeofday = $configuration->getAttribute('time_of_day');
  my $data_file = $configuration->getAttribute('data_file');

  my $conf = $doc->getElementsByTagName('conf');
  my $aircrafts = parse_configuration($conf->[0]);
  my $protocol = $doc->getElementsByTagName('protocol');
  return { date => $timeofday, data_file => $data_file , aircrafts => $aircrafts };
}

sub read_data {
  my ($filename) = @_;
  $filename = $PAPARAZZI_HOME."/var/logs"."/".$filename;
  my @data = ();
  open(INFILE,  $filename) or print STDOUT "Cant open $filename: $!";
  while (my $line = <INFILE>) {
    if ($line =~ /(^\d+\.\d+) (\d+) (\w+) (.+)/) {
      push @data, {time => $1, sender => $2, msg_id => $3, args => $4};
    }
  }
  close INFILE;
  my $duration = @data[$#data]->{time};
  return {nb_messages => $#data, duration => $duration, raw_data => \@data};
}


sub parse_configuration {
  my ($conf) = @_;
  my @ret = ();
  my @aircrafts = $conf->getElementsByTagName('aircraft');
  foreach my $aircraft (@aircrafts) {
    push @ret, parse_aircraft($aircraft);
  }
  return \@ret;

}

sub parse_aircraft {
  my ($aircraft) = @_;
  my $ac_name = $aircraft->getAttribute('name');
#  print "aircraft : $ac_name<br>\n";
  return $ac_name;
}


1;
