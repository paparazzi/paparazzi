package Paparazzi::Ploter;

use Paparazzi::Environment;

use Expect;

sub gen_plot {
  my ($terminal, $filename, $plot_cmd ) = @_;

  my $set_terminal_cmd = "set terminal $terminal";
  my $set_output_cmd = "set output \"".Paparazzi::Environment::paparazzi_home()."/var/plot/$filename\"";
  my $print_cmd = "$set_terminal_cmd; $set_output_cmd; $plot_cmd";
  my $exp = new Expect();
  $exp->raw_pty(1);
  $Expect::Debug = 10;
  my $pid = $exp->spawn("/usr/bin/gnuplot", ("-geometry", "1x1+0+0")) or printf "Don't find gnuplot";
  $pid->log_stdout(0);
  #  print("Printing $print_cmd <br>\n");
  $exp->send($print_cmd."\n");
  my $timeout = 1;
  my $foo = $exp->expect($timeout);
  $exp->hard_close();
  return "http://ornette:8889/var/plot/".$filename;
}



1;
