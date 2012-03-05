=head1 SYNOPSIS

	use Program;
	my $m_program = new Program('ls');

=head1 DESCRIPTION

Program is a generic program wrapper that allows easy use of programs on
multipul platforms with the correct file handle redirection.

=head1 FUNCTIONS

=cut

package		Program;

###################
# Standard Modules
use strict;
use Config;
use FileHandle;

###################
# Variables
use vars qw(@ISA @EXPORT @EXPORT_OK $VERSION);
@ISA = qw();
@EXPORT = qw();
@EXPORT_OK = qw();
$VERSION = 0.04;

# This hash contains the values for redirection.
my %m_redirect_values;

# Standard values
$m_redirect_values{'none'} = '';
$m_redirect_values{'stdout&stderr2stdout'} = '2>&1';
$m_redirect_values{'stdout&stderr2stderr'} = '1>&2';
$m_redirect_values{'stdout2stderr&stderr2stdout'} = '3>&1 1>&2 2>&3 3>&-';

# Platform specific values
if ($Config{'osname'} eq 'MSWin32')
{
	if ($Config{'osvers'} ge 4.0)
	{
		$m_redirect_values{'stdout2stdnull'} = '1>NUL';
		$m_redirect_values{'stderr2stdnull'} = '2>NUL';
	}
	else
	{
		$m_redirect_values{'stdout2stdnull'} = '1>';
		$m_redirect_values{'stderr2stdnull'} = '2>';
	}
}
else
{
	$m_redirect_values{'stdout2stdnull'} = '1>/dev/null';
	$m_redirect_values{'stderr2stdnull'} = '2>/dev/null';
}

###################
# Functions

################################################################################
# Function:	new()

=head2 B<new()>

 Description:	This method returns an objective interface the the passed program.
 Arguments:	$program
 Return:	class
 Usage:		my $ls = new Program('ls');

=cut

sub new
{
	my $class = shift;
	my $program = shift;

	my $self = bless {}, $class;
	$self->{'PROGRAM'} = $program;
	$self->{'CHOMP'} = 1;
	$self->{'REDIRECT'} = 'stdout&stderr2stdout';
	$self->{'LAST_COMMAND'} = "No commands have been executed yet.";
	$self->{'EXIT_STATUS'} = 0;

	return $self;
}

################################################################################
# Function:	strip_new_lines()

=head2 B<strip_new_lines()>

 Description:	This functions is used to set wether the output from a command
 		has trailing new lines removed.
 Arguments:	0 to turn chomping off
 		1 to turn chomping on
 Default:	1
 Return:	true if chomping is on else undef
 Usage:		$ls->strip_new_lines(1);
 		$ls->strip_new_lines(0);

=cut 

sub strip_new_lines
{
	my $self = shift;
	my $value = shift;

	if (scalar $value)
	{
		$self->{'CHOMP'} = $value;
	}
	else
	{
		return $self->{'CHOMP'};
	}
}


################################################################################
# Function:	redirect()

=head2 B<redirect()>

 Description:	This functions is used to set the STDOUT and STDERR redirection
 		for commands executed by the program.
 Arguments:	possible values are stdout&stderr2stdout, stdout&stderr2stderr
 		stdout2stderr&stderr2stdout, stdout2stdnull, stderr2stdnull
 Default:	sdtout&stderr2stdout
 Return:	redirection option if nothing passed else set the redirection
 Usage:		$ls->redirect('stderr2sdtnull');

=cut 

sub redirect
{
	my $self = shift;
	my $value = shift;

	if (scalar $value)
	{
		$self->{'REDIRECT'} = $value;
	}
	else
	{
		return $m_redirect_values{$self->{'REDIRECT'}};
	}
}

################################################################################
# Function:	last_command()

=head2 B<last_command()>

 Description:	This function returns the last command executed.
 Arguments:	None.
 Return:	The last command
 Usage:		print $ls->last_command();

=cut

sub last_command
{
	my $self = shift;
	return $self->{'LAST_COMMAND'};
}

################################################################################
# Function:	output()

=head2 B<output()>

 Description:	This function returns the output from a program.
 Arguments:	command to execute.
 Return:	array if called in an array context else string
 Usage:		my $output = $ls->output("-l");
		my @output = $ls->output("-l");

=cut

sub output
{
	my $self = shift;
	my $command = shift;
	my $exec_command = "$self->{'PROGRAM'} $command " . $self->redirect();
	$self->{'LAST_COMMAND'} = $exec_command;

	my @output_list;
	my $output_line;
	if (wantarray)
	{
		@output_list = `$exec_command`;
		foreach my $line (@output_list)
		{
			chomp $line if $self->strip_new_lines();
		}
	}
	else
	{
		$output_line = `$exec_command`;
		chomp $output_line if $self->strip_new_lines();
	}
	$self->{'EXIT_STATUS'} = $?/256;

	return wantarray ? @output_list : $output_line;
}

################################################################################
# Function:	status()

=head2 B<status()>

 Description:	This function returns the exit status of the last
 		command.
 Arguments:	None.
 Return:	exit status.
 Usage:		my $status = $ls->status();

=cut

sub status
{
	my $self = shift;
	return $self->{'EXIT_STATUS'};
}

################################################################################
# Function:	success()

=head2 B<success()>

 Description:	This function returns true if the last command was successful
 Arguments:	None.
 Return:	true for success else undef.
 Usage:		if ($ls->success())
 		{
			Print "Success\n";
		}
		else
		{
			Print "Failure\n";
		}

=cut

sub success
{
	my $self = shift;
	return 1 if $self->status() eq 0;
	return undef
}

################################################################################
# Function:	open()

=head2 B<open()>

 Description:	This function returns an open file handle for the passed command
 		NOTE: Since the exit status of the file handle cannot be
		retrieved by this module the user must check the exit status of
		the file handle using the $fh->error() method. See IO::Handle.
 Arguments:	command to execute.
 Return:	open file handle.
 Usage:		my $fh = $ls->open("-l)
 		while (<$fh>)
		{
			print "$_\n";
		}

=cut

sub open
{
	my $self = shift;
	my $command = shift;

	my $exec_command = "$self->{'PROGRAM'} $command " . $self->redirect() ." |";
	$self->{'LAST_COMMAND'} = $exec_command;
	$self->{'EXIT_STATUS'} = 0;

	my $fh = new FileHandle($exec_command);
	return $fh;
}

1;

__END__

=head1 SEE ALSO

 FileHandle

=head1 AUTHOR

 Bernard Davison bernard@gondwana.com.au

=head1 COPYRIGHT

 Copyright (C) 2000, Gondwanatech.

=cut

