#!/usr/bin/perl -w

use Test::More tests => 3;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use Program;

$|++; 

####################
# Make the airframe
my $make_compile_options = "AIRCRAFT=LisaLv11_Booz2v12_RC clean_ac ap.compile";
my $compile_output = run_program(
	"Attempting to build and upload the firmware.",
	$ENV{'PAPARAZZI_SRC'},
	"make $make_compile_options",
	0,1);
unlike($compile_output, '/Aircraft \'LisaLv11_Booz2v12_RC\' not found in/', "The compile output does not contain the message \"Aircraft \'LisaLv11_Booz2v12_RC\' not found in\"");
unlike($compile_output, '/Error/i', "The compile output does not contain the word \"Error\"");

####################
# Upload the airframe
my $make_upload_options = "AIRCRAFT=LisaLv11_Booz2v12_RC BOARD_SERIAL=LISA-L-000156 ap.upload";
my $upload_output = run_program(
	"Attempting to build and upload the firmware.",
	$ENV{'PAPARAZZI_SRC'},
	"make $make_upload_options",
	0,1);
unlike($upload_output, '/Error/i', "The upload output does not contain the word \"Error\"");


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

