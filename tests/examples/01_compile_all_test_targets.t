#!/usr/bin/perl -w

use Test::More;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use XML::Simple;
use Program;
use Data::Dumper;
use Config;

$|++;
my $xmlSimple = XML::Simple->new(ForceArray => 1);
my $examples = $xmlSimple->XMLin("$ENV{'PAPARAZZI_SRC'}/conf/conf.xml");

ok(1, "Parsed the tests configuration file");
foreach my $example (sort keys%{$examples->{'aircraft'}})
{
	my $airframe = $examples->{'aircraft'}->{$example}->{'airframe'};
	my $airframe_config = $xmlSimple->XMLin("$ENV{'PAPARAZZI_SRC'}/conf/$airframe");
	foreach my $process (sort keys %{$airframe_config->{'firmware'}})
	{
        #warn "EX: [$example] ". Dumper($airframe_config->{'firmware'}->{$process}->{'target'});
        foreach my $target (sort keys %{$airframe_config->{'firmware'}->{$process}->{'target'}})
        {
            #warn "EXAMPLE: [$example] TARGET: [$target]\n";
            my $make_options = "AIRCRAFT=$example clean_ac $target.compile";
            my ($exit_status, $output) = run_program(
                "Attempting to build the firmware $target for the aircraft $example.",
                $ENV{'PAPARAZZI_SRC'},
                "make $make_options",
                $ENV{'TEST_VERBOSE'},1);
            # print output if it failed and we didn't already print it in verbose mode
            warn "$output\n" if $exit_status && !$ENV{'TEST_VERBOSE'};
            ok($exit_status == 0, "Compile aircraft: $example, target: $target");
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
                if ($dont_fail_on_error)
                {
                        warn "\nError: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n";
                }
                else
                {
                        die "\nError: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n";
                }
        }
        my $output_string = join "\n", @output;
        return ($exit_status, $output_string);
}

