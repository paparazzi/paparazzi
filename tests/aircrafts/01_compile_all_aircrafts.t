#!/usr/bin/perl -w

#
# Reads a conf XML file and compiles all targets of all aircrafts.
#
# Mandatory environment variables:
#  PAPARAZZI_SRC : path to paparazzi source directory
#  PAPARAZZI_HOME : path to paparazz home directory containing the conf
#
# optional environment variables:
#  CONF_XML : path to conf file to read, conf/conf.xml by default
#  TEST_VERBOSE : set to 1 to print the compile output even if there was no error
#  SHOW_WARNINGS_FULL : set to 1 to print the complete compile output if there were warnings
#  HIDE_WARNINGS : set to 1 to disable printing of warnings
#
# environment variables passed on to make:
#  J=AUTO : detect number of CPUs to set jobs for parallel compilation (default)
#
# Examples on how to test compile all aircrafts/targets in your current conf.xml:
# only showing full compile output if there has been an error, if there were warnings only print those
#  prove test/aircrafts
# with parallel compilation and showing full output during compilation
#  J=AUTO prove tests/aircrafts -v
# without parallel compilation and treating all warnings as errors:
#  J=1 USER_CFLAGS=-Werror prove tests/aircrafts
#
# Example on how to test a specific conf.xml and disabling printing of warnings:
#  CONF_XML=conf/airframes/flixr/conf.xml HIDE_WARNINGS=1 prove tests/aircrafts
#

use Test::More;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use XML::Simple;
use Data::Dumper;
use Config;
use IPC::Run qw( run );
use Cwd;

$|++;
# make sure that all elements are arrays and only fold arrays on name for aircraft, firmware and target
my $xmlSimple = XML::Simple->new(ForceArray => 1, KeyAttr => {aircraft => 'name', firmware => 'name', target => 'name'});
my $conf_xml_file = $ENV{'CONF_XML'};
if ($conf_xml_file eq "") {
    $conf_xml_file = "$ENV{'PAPARAZZI_HOME'}/conf/conf.xml";
}
my $conf = $xmlSimple->XMLin($conf_xml_file);


sub get_num_targets
{
    my $num_targets = 0;
    foreach my $aircraft (sort keys%{$conf->{'aircraft'}})
    {
        my $airframe = $conf->{'aircraft'}->{$aircraft}->{'airframe'};
        my $airframe_config = eval { $xmlSimple->XMLin("$ENV{'PAPARAZZI_HOME'}/conf/$airframe") };
        warn "Parsing airframe $airframe: $@" if ($@);
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
plan tests => get_num_targets()+2;

ok(1, "Parsed the $conf_xml_file configuration file");

my @invalid_airframes;

foreach my $aircraft (sort keys%{$conf->{'aircraft'}})
{
    my $airframe = $conf->{'aircraft'}->{$aircraft}->{'airframe'};
    my $airframe_config = eval { $xmlSimple->XMLin("$ENV{'PAPARAZZI_HOME'}/conf/$airframe") };
    if ($@)
    {
        warn "Skipping aircraft $aircraft: $@";
        push @invalid_airframes, "$airframe: $@";
    }
    foreach my $process (sort keys %{$airframe_config->{'firmware'}})
    {
        #warn "EX: [$aircraft] ". Dumper($airframe_config->{'firmware'}->{$process}->{'target'});
        foreach my $target (sort keys %{$airframe_config->{'firmware'}->{$process}->{'target'}})
        {
            diag("compiling AIRCRAFT: [$aircraft] TARGET: [$target]");
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

# check if we had missing/invalid airframe files in conf
ok(scalar @invalid_airframes eq 0, "All airframe files are valid.");
foreach (@invalid_airframes) { warn "Missing or invalid airframe file '$_'\n" }

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
        warn @_ if $verbose;
        $stderr_and_out .= $_[0];
    };
    my $stderr_handler = sub {
        warn @_ if $verbose;
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

