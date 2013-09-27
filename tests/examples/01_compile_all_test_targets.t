#!/usr/bin/perl -w

use Test::More;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use XML::Simple;
use Program;
use Data::Dumper;
use Config;

$|++;
my $examples = XMLin("$ENV{'PAPARAZZI_SRC'}/conf/conf_tests.xml");

use Data::Dumper;

ok(1, "Parsed the tests_conf file");
foreach my $example (sort keys%{$examples->{'aircraft'}})
{
	#next unless $example =~ m#easystar#i;
	my $airframe = $examples->{'aircraft'}->{$example}->{'airframe'};
	my $airframe_config = XMLin("$ENV{'PAPARAZZI_SRC'}/conf/$airframe");
	foreach my $process (sort keys %{$airframe_config->{'firmware'}})
	{
		if ($process =~ m#setup|fixedwing|rotorcraft|lisa_test_progs#)
		{
			#warn "EX: [$example] ". Dumper($airframe_config->{'firmware'}->{$process}->{'target'});
			foreach my $target (sort keys %{$airframe_config->{'firmware'}->{$process}->{'target'}})
			{
				next unless scalar $airframe_config->{'firmware'}->{$process}->{'target'}->{$target}->{'board'};

				#warn "EXAMPLE: [$example] TARGET: [$target]\n";
				my $make_options = "AIRCRAFT=$example clean_ac $target.compile";
				my $output = run_program(
					"Attempting to build the firmware $target for the airframe $example.",
					$ENV{'PAPARAZZI_SRC'},
					"make $make_options",
					$ENV->{'TEST_VERBOSE'},1);
				unlike($output, '/\bError\b/i', "The make output for the $example target $target does not contain the word \"Error\"");
			}
		}
		elsif ($process =~ m#target#)
		{
			#warn "EXT: [$example] ". Dumper($airframe_config->{'firmware'}->{$process});
			foreach my $target (sort keys %{$airframe_config->{'firmware'}->{$process}})
			{
				next unless scalar $airframe_config->{'firmware'}->{$process}->{$target}->{'board'};

				#warn "EXAMPLET: [$example] TARGET: [$target]\n";
				my $make_options = "AIRCRAFT=$example clean_ac $target.compile";
				my $output = run_program(
					"Attempting to build the firmware $target for the airframe $example.",
					$ENV{'PAPARAZZI_SRC'},
					"make $make_options",
					$ENV->{'TEST_VERBOSE'},1);
				unlike($output, '/\bError\b/i', "The make output for the $example target $target does not contain the word \"Error\"");
			}
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
                        warn "Error: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n";
                }
                else
                {
                        die "Error: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n";
                }
        }
        return wantarray ? @output : join "\n", @output;
}

