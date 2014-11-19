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
#  SHOW_WARNINGS_FULL : set to 1 to print the complete compile output if there were warnings
#  HIDE_WARNINGS : set to 1 to disable printing of warnings
#
# environment variables passed on to make:
#  J=AUTO : detect number of CPUs to set jobs for parallel compilation
#
# Examples on how to test compile all aircrafts/targets in your current conf.xml:
# only showing full compile output if there has been an error, if there were warnings only print those
#  prove test/examples
# with parallel compilation and showing full output during compilation
#  J=AUTO prove tests/examples -v
# with parallel compilation and treating all warnings as errors:
#  J=AUTO USER_CFLAGS=-Werror prove tests/examples
#

use Test::More;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use XML::Simple;
use Data::Dumper;
use Config;
use IPC::Run qw( run );
use Cwd;

$|++;
my $xmlSimple = XML::Simple->new(ForceArray => 1);
my $conf = $xmlSimple->XMLin("$ENV{'PAPARAZZI_HOME'}/conf/conf.xml");


sub get_num_targets
{
    my $num_targets = 0;
    foreach my $aircraft (sort keys%{$conf->{'aircraft'}})
    {
        my $airframe = $conf->{'aircraft'}->{$aircraft}->{'airframe'};
        my $airframe_config = $xmlSimple->XMLin("$ENV{'PAPARAZZI_HOME'}/conf/$airframe");
        foreach my $process (sort keys %{$airframe_config->{'firmware'}})
        {
            foreach my $target (sort keys %{$airframe_config->{'firmware'}->{$process}->{'target'}})
            {
                $num_targets++;
            }
        }
    }
    return $num_targets;
}
plan tests => get_num_targets()+1;

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
            my ($exit_status, $warnings, $output) = run_program(
                "Attempting to build the firmware $target for the aircraft $aircraft.",
                $ENV{'PAPARAZZI_SRC'},
                "make $make_options",
                $ENV{'TEST_VERBOSE'});

            # if we didn't already print output in verbose mode,
            # print if it failed
            if ($exit_status && !$ENV{'TEST_VERBOSE'}) {
                warn "$output\n";
            }
            # if successful, still print warnings if requested
            elsif ($warnings && !$ENV{'HIDE_WARNINGS'}) {
                if (!$ENV{'TEST_VERBOSE'}) {
                    warn "\nWarning: AIRCRAFT=$aircraft target=$target compiled sucessfully but had warnings:\n";
                    if ($ENV{'SHOW_WARNINGS_FULL'}) {
                        warn "$output\n";
                    }
                    else {
                        warn "$warnings\n";
                    }
                }
                if ($ENV{'SHOW_WARNINGS_FULL'}) {
                    warn "\nAIRCRAFT=$aircraft target=$target compiled sucessfully but had warnings.\n\n";
                }
            }
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

    warn "\n$message\n" if $verbose;
    warn "Running command: \"". $command ."\"\n" if $verbose;

    # change into specified dir and remember current working dir
    my $working_dir = cwd;
    if (defined $dir) {
        chdir $dir;
    }

    my $warnings = '';
    my $stderr_and_out = '';

    my $stdout_handler = sub {
        print @_ if $verbose;
        $stderr_and_out .= $_[0];
    };
    my $stderr_handler = sub {
        print @_ if $verbose;
        # check if output on stderr contains warnings, but ignoring "Warning: low altitude"
        if ($_[0] =~ /warning/i && $_[0] !~ /Warning: low altitude/) {
            $warnings .= $_[0]."\n";
            #warn "\ndetected warning in $_[0]\n";
        }
        $stderr_and_out .= $_[0];
    };
    my $dummy_in;
    my $run = run([split ' ', $command], \$dummy_in, $stdout_handler, $stderr_handler);
    my $exit_status = $?/256;

    # change back to original dir
    chdir $working_dir;

    unless ($exit_status == 0)
    {
        warn "\nError: The command \"". $command ."\" failed to complete successfully. Exit status: $exit_status\n";
    }

    return ($exit_status, $warnings, $stderr_and_out);
}

