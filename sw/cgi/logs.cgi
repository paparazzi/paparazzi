#!/usr/bin/perl

# displays available logs

use strict;
use warnings;

use Cwd;
use CGI ":standard";
my $paparazzi_src;
my $paparazzi_lib;
BEGIN {
  $paparazzi_src = getcwd()."/../..";
  $paparazzi_lib = $paparazzi_src."/sw/lib/perl";
}
use lib ($paparazzi_lib);
my $paparazzi_home = $paparazzi_src;

use Paparazzi::Environment;
Paparazzi::Environment::set_env($paparazzi_src, $paparazzi_home);

use Paparazzi::Log;

my $query = new CGI();
print $query->header();
print $query->start_html("Paparazzi Logs");

my @logs = Paparazzi::Log::get_available();

my $log_info = undef;
my $log_data = undef;
my @files = $query->param('file');
if ($#files >=0 ) {
  $log_info = Paparazzi::Log::read_infos($files[0]);
  $log_data = Paparazzi::Log::read_data($log_info->{data_file});
}

print "\n<H1> Paparazzi Logs.</H1>\n";
print "<H2> Logs.</H2>\n";
print $query->startform();
print $query->popup_menu('file', \@logs, $logs[0]);
print $query->submit('Action','Update');
print $query->endform();
if (defined $log_info) {
  print $query->path_info()."<br>";
  Paparazzi::Log::html_print_summary($log_info, $log_data);
#  my $url = Paparazzi::Log::gen_activity_plot($log_info, $log_data);
#  print"<img src=\"$url\">\n"; 
}
print "<HR>\n";
print "<a href=\"../index.html\">home</a><BR>\n";
print "<HR>\n";
print "<ADDRESS>Poine.</ADDRESS><BR>\n";

