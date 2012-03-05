#!/usr/bin/perl -w

use Test::More;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use XML::Simple;
use Program;

$|++; 
my $examples = XMLin("$ENV{'PAPARAZZI_SRC'}/conf/conf.xml.example");

ok(1, "Parsed the example file");
foreach my $example (sort keys%{$examples->{'aircraft'}})
{
	my $airframe = $examples->{'aircraft'}->{$example}->{'airframe'};
	my $airframe_config = XMLin("$ENV{'PAPARAZZI_SRC'}/conf/$airframe");
	foreach my $process (sort keys %{$airframe_config->{'firmware'}})
	{
		foreach my $target (sort keys %{$airframe_config->{'firmware'}->{$process}->{'target'}})
		{
			my $make_upload_options = "AIRCRAFT=$example $target.compile";
			my $upload_output = run_program(
        			"Attempting to build the firmware $target for the airframe $example.",
        			$ENV{'PAPARAZZI_SRC'},
        			"make $make_upload_options",
        			0,1);
			unlike($upload_output, '/Error/i', "The upload output does not contain the word \"Error\"");
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
                        warn "Error: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n" if $verbose;
                }
                else
                {
                        die "Error: The command \"". $prog->last_command() ."\" failed to complete successfully. Exit status: $exit_status\n";
                }
        }
        return wantarray ? @output : join "\n", @output;
}

