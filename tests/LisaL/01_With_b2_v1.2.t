#!/usr/bin/perl -w

use Test::More tests => 8;
use lib "$ENV{'PAPARAZZI_SRC'}/tests/lib";
use Program;
use Proc::Background;

$|++; 

####################
# Make the airframe
my $make_compile_options = "AIRCRAFT=LisaLv11_Booz2v12_RC clean_ac ap.compile";
my $compile_output = run_program(
	"Attempting to build the firmware.",
	$ENV{'PAPARAZZI_SRC'},
	"make $make_compile_options",
	0,1);
unlike($compile_output, '/Aircraft \'LisaLv11_Booz2v12_RC\' not found in/', "The compile output does not contain the message \"Aircraft \'LisaLv11_Booz2v12_RC\' not found in\"");
unlike($compile_output, '/\bError\b/i', "The compile output does not contain the word \"Error\"");

####################
# Upload the airframe
my $make_upload_options = "AIRCRAFT=LisaLv11_Booz2v12_RC BOARD_SERIAL=LISA-L-000156 ap.upload";
my $upload_output = run_program(
	"Attempting to build and upload the firmware.",
	$ENV{'PAPARAZZI_SRC'},
	"make $make_upload_options",
	0,1);
SKIP: {
	skip "The requested hardware isn't available on this host.", 1 unless $make_upload_options =~ m#Error: unable to open ftdi device: device not found#;
	unlike($upload_output, '/\bError\b/i', "The upload output does not contain the word \"Error\"");
}

# Start the server process
my $server_command = "$ENV{'PAPARAZZI_HOME'}/sw/ground_segment/tmtc/server";
my @server_options = qw(-n);
my $server = Proc::Background->new($server_command, @server_options);
sleep 2; # The service should die in this time if there's an error
ok($server->alive(), "The server process started successfully");

# Start the link process
my $link_command = "$ENV{'PAPARAZZI_HOME'}/sw/ground_segment/tmtc/link";
my @link_options = qw(-d /dev/ttyUSB0 -s 57600 -transport xbee -xbee_addr 123);
sleep 2; # The service should die in this time if there's an error
my $link = Proc::Background->new($link_command, @link_options);
ok($link->alive(), "The link process started successfully");

# Open the Ivy bus and read from it...
SKIP : {
        skip "Skipping testing of the hardware since we can't load the Ivy module. Please install IO::Socket::Multicast", 1 unless eval("use Ivy; 1");
        ok(1, "We can load the Ivy module.");
        # TODO: learn how to read and write to the Ivy bus
}

# Shutdown the server and link processes
ok($server->die(), "The server process shutdown successfully.");
ok($link->die(), "The link process shutdown successfully.");

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

