#!/usr/bin/perl -w

#
# Reads conf/conf.xml (can be symlink to e.g. conf/conf_tests.xml)
# and compiles all targets of all aircrafts.
#
# Mandatory environment variables:
#  PAPARAZZI_SRC : path to paparazzi source directory
#  PAPARAZZI_HOME : path to paparazz home directory containing the conf
#
# optional environment variables:
#  TEST_VERBOSE : set to 1 to print the compile output even if there was no error
#
# environment variables passed on to make:
#  J=AUTO : detect number of CPUs to set jobs for parallel compilation
#
# Example on how to test compile all aircrafts/targets in your current conf.xml
# with parallel compilation and treating all warnings as errors:
#  J=AUTO USER_CFLAGS=-Werror prove tests/examples
#

use Test::More;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use XML::Simple;
use Program;
use Data::Dumper;
use Config;

$|++;
my $xmlSimple = XML::Simple->new(ForceArray => 1);
my $conf = $xmlSimple->XMLin("$ENV{'PAPARAZZI_HOME'}/conf/conf.xml");

ok(1, "Parsed the configuration file");
foreach my $aircraft (sort keys%{$conf->{'aircraft'}})
{
	my $airframe = $conf->{'aircraft'}->{$aircraft}->{'airframe'};
	my $airframe_config = $xmlSimple->XMLin("$ENV{'PAPARAZZI_HOME'}/conf/$airframe");
	foreach my $process (sort keys %{$airframe_config->{'firmware'}})
	{
        #warn "EX: [$aircraft] ". Dumper($airframe_config->{'firmware'}->{$process}->{'target'});
        foreach my $target (sort keys %{$airframe_config->{'firmware'}->{$process}->{'target'}})
        {
            #warn "AIRCRAFT: [$aircraft] TARGET: [$target]\n";
            my $make_options = "AIRCRAFT=$aircraft clean_ac $target.compile";
            my ($exit_status, $output) = run_program(
                "Attempting to build the firmware $target for the aircraft $aircraft.",
                $ENV{'PAPARAZZI_SRC'},
                "make $make_options",
                $ENV{'TEST_VERBOSE'},1);
            # print output if it failed and we didn't already print it in verbose mode
            warn "$output\n" if $exit_status && !$ENV{'TEST_VERBOSE'};
            ok($exit_status == 0, "Compile aircraft: $aircraft, target: $target");
        }
	}
}

done_testing();

################################################################################
# functions used by this test script.
sub run_program
{
        my $message = shift;
        my $dir = shift;
        my $command = shift;
        my $verbose = shift;
        my $dont_fail_on_error = shift;

        warn "\n$message\n" if $verbose;
        if (defined $dir)
        {
                $command = "cd $dir;" . $command;
        }
        my $prog = new Program("bash");
        #$prog->redirect('none');
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
                my $err_msg = "\nError: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n";
                if ($dont_fail_on_error)
                {
                        warn $err_msg;
                }
                else
                {
                        die $err_msg;
                }
        }
        my $output_string = join "\n", @output;
        return ($exit_status, $output_string);
}

