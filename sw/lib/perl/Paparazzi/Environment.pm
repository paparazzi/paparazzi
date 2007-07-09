package Paparazzi::Environment;

use File::NCopy;
use Getopt::Long;
use XML::DOM;

use constant INST_PREFIX => "/usr";

my $paparazzi_src = undef;
my $paparazzi_home = $ENV{HOME}."/paparazzi";

if (defined $ENV{PAPARAZZI_SRC}) {
  $paparazzi_src = $ENV{PAPARAZZI_SRC};
  $paparazzi_home =  $ENV{PAPARAZZI_SRC};
}
if (defined $ENV{PAPARAZZI_HOME}) {
  $paparazzi_home = $ENV{PAPARAZZI_HOME}
}
#else {
#  $paparazzi_home = "/usr/share/paparazzi" unless defined $ENV{PAPARAZZI_SRC};
#}
#print "\nEnvironment : ";
#if (defined $paparazzi_src) {
#  print "source directory mode\n  paparazzi_src  $paparazzi_src\n";
#}
#else {
#  print "system mode\n  inst_prefix     INST_PREFIX";
#}
#print "  paparazzi_home $paparazzi_home\n\n";


sub set_env {
  my ($pps, $pph) = @_;
  $paparazzi_src = $pps;
  $paparazzi_home = $pph;
}


sub parse_command_line {
  my ($options) = @_;
  my $getopt_h = {"b=s" => \$options->{ivy_bus}};
  foreach my $option (keys %{$options}) {
    $getopt_h->{$option."=s"} = \$options->{$option};
  }
  return GetOptions (%{$getopt_h});
}

sub check_paparazzi_home {
  unless (defined $paparazzi_src) {
    unless (-e $paparazzi_home) {
      print "\nDirectory $paparazzi_home doesn't exist\n";
      print "This directory is needed to store user configuration and data\n";
      print "Shall I create it and populate it with examples? (Y/n)\n";
      my $ans = <STDIN>;
      chop($ans);
      if ($ans eq "" || $ans eq "Y" || $ans eq "y") {
	print "Creating directory $paparazzi_home\n";
	mkdir($paparazzi_home, 0755);
	print "Copying default config and examples\n";
	my $copier = File::NCopy->new(recursive => 1);
	foreach my $dir ("conf", "var", "data") {
	  $copier->copy(INST_PREFIX."/share/paparazzi/".$dir, $paparazzi_home);
	}
      	print "done.\n\n";
      }
      else {
	print "exiting...\n";
	exit(1);
      }
    }
  }
}

sub read_config {
  my $filename = $paparazzi_home."/conf/conf.xml";
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  my $conf = $doc->getElementsByTagName("conf")->[0];
  my $aircrafts = $conf->getElementsByTagName("aircraft");;
  foreach my $aircraft (@{$aircrafts}){
    my $name = $aircraft->getAttribute('name');
#    print ("name $name\n");
  }
}

sub paparazzi_src {
  return $paparazzi_src;
}

sub paparazzi_home {
  return $paparazzi_home;
}

sub get_config {
  my ($file) = @_;
  return $paparazzi_home."/conf/".$file;
}

sub get_data {
  my ($file) = @_;
  return $paparazzi_home."/data/".$file;
}

sub get_default_map {
  my $filename = $paparazzi_home."/conf/conf.xml";
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  my $map_conf = $doc->getElementsByTagName("map")->[0];
  my $calib_file = $map_conf->getAttribute('location');
  $calib_file = get_data($calib_file);
#  print "in Paparazzi::Environment::get_default_map $calib_file\n";
  return $calib_file;
}


1;
