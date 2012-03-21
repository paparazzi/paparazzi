#!/usr/bin/perl -w

use Test::More tests => 6;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use Program;
use Proc::Background;
use Ivy;

$|++; 

####################
# Make the airframe
my $make_compile_options = "AIRCRAFT=Microjet clean_ac sim.compile";
my $compile_output = run_program(
	"Attempting to build the sim firmware.",
	$ENV{'PAPARAZZI_SRC'},
	"make $make_compile_options",
	0,1);
unlike($compile_output, '/Aircraft \'Microjet\' not found in/', "The compile output does not contain the message \"Aircraft \'Microjet\' not found in\"");
unlike($compile_output, '/\bError\b/i', "The compile output does not contain the word \"Error\"");

# Start the server process
my $server_command = "$ENV{'PAPARAZZI_HOME'}/sw/ground_segment/tmtc/server";
my @server_options = qw(-n);
my $server = Proc::Background->new($server_command, @server_options);
sleep 2; # The service should die in this time if there's an error
ok($server->alive(), "The server process started successfully");

# Start the sim process
my $link_command = "$ENV{'PAPARAZZI_HOME'}/sw/simulator/launchsitl";
my @link_options = qw(-a -boot -norc);
sleep 2; # The service should die in this time if there's an error
my $link = Proc::Background->new($link_command, @link_options);
ok($link->alive(), "The launchsitl process started successfully");

# Open the Ivy bus and read from it...
# TODO: learn how to read and write to the Ivy bus

# Shutdown the server and link processes
ok($server->die(), "The server process shutdown successfully.");
ok($link->die(), "The launchsitl process shutdown successfully.");

################################################################################
# functions used by this test script.
sub run_program
{
        my $message = shift;
        my $dir = shift;
        my $command = shift;
        my $verbose = shift;
        my $dont_fail_on_error = shift;

        warn "$message\n" if $verbose;
        if (defined $dir)
        {
                $command = "cd $dir;" . $command;
        }
        my $prog = new Program("bash");
        my $fh = $prog->open("-c \"$command\"");
	warn "Running command: \"". $prog->last_command() ."\"\n" if $verbose;
        $fh->autoflush(1);
        my @output;
        while (<$fh>)
        {
		warn $_ if $verbose;
		chomp $_;
                push @output, $_;
        }
        $fh->close;
        my $exit_status = $?/256;
        unless ($exit_status == 0)
        {
                if ($dont_fail_on_error)
                {
                        warn "Error: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n" if $verbose;
                }
                else
                {
                        die "Error: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n";
                }
        }
        return wantarray ? @output : join "\n", @output;
}

