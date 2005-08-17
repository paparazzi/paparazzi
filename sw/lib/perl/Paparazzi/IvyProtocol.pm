package Paparazzi::IvyProtocol;

use strict;
use XML::DOM;
use Text::CSV;
use Carp;
use Ivy;

use Data::Dumper;

my $classes_by_name = {};
my $ivy = undef;
my $app_name = "";
my $req_id = 0;

sub init {
  my (%options) = @_;
  my $opt = \%options;
  read_protocol($opt->{-file}, 'ground');
  $app_name = $opt->{-app_name};
  Ivy->init( -ivyBus        => $opt->{-ivy_bus},
             -appName       => $opt->{-app_name},
             -loopMode      => $opt->{-loop_mode},
             -messWhenReady => 'READY',
           );
  $ivy = Ivy->new ();
  $ivy->start();
}

sub send_request {
  my ($sender_name, $msg_class, $msg_name, $known_fields, $user_cb) = @_;
  print "in send request $msg_class $msg_name\n";
  print Dumper($known_fields);
  my $result_name = $msg_name;#."_RES";
  my $regexp = get_regexp($sender_name, $msg_class, $msg_name, $known_fields);
  $regexp =~ s/$msg_name/$result_name/;
  $regexp =~ s/$sender_name/$req_id $sender_name/;
  $ivy->bindRegexp($regexp, [\&on_res_received, $msg_class, $msg_name, $known_fields, $regexp, $user_cb]);
  my $request_name = $msg_name."_REQ";
  my $msg = print_msg("", $msg_class, $request_name, $known_fields, 0);
  $msg =~ s/$app_name/$app_name $req_id/;
  $ivy->sendMsgs($msg);
  $req_id++;
}

sub on_res_received {
  my ($sender_name, $msg_class, $msg_name, $known_fields, $regexp, $user_cb, @matched_regexps) = @_;
  print "on res received\n";
  print Dumper($known_fields);
  $ivy->bindRegexp($regexp);
  on_msg_received($sender_name, $msg_class, $msg_name, $known_fields, $user_cb, @matched_regexps);

}

sub send_msg {
  my ($msg_class, $msg_name, $fields) = @_;
  my $msg = print_msg("", $msg_class, $msg_name, $fields, 0);
  $ivy->sendMsgs($msg);
}

sub bind_msg {
  my ($sender_name, $msg_class, $msg_name, $known_fields, $cb) = @_;
  my $regexp = get_regexp($sender_name, $msg_class, $msg_name, $known_fields);
  if ($cb) {
     $ivy->bindRegexp($regexp, [\&on_msg_received, $msg_class, $msg_name, $known_fields, $cb]);  }
  else {
    $ivy->bindRegexp($regexp);
  }
}

sub on_msg_received {
  my ($sendername, $msg_class, $msg_name, $known_fields, $cb, @matched_regexps) = @_;
  #  print STDERR "##### on_msg_received ".Dumper(@_);
  my $ret = {};
  my $msg =  $classes_by_name->{$msg_class}->{$msg_name};
  unless (defined $msg) {
    print STDERR "in IvyProtocol::on_msg_received : unknown message $msg_class $msg_name\n";
    return;
  }
  my $nb_fields = @{$msg};
  foreach my $field (@{$msg}) {
    my $field_name = $field->{name};
    if ($known_fields->{$field_name}) {
      $ret->{$field_name} = $known_fields->{$field_name};
    } else {
      my $val = shift @matched_regexps;
      if (exists $field->{format} and $field->{format} eq 'csv') {
#	print "in IvyProtocol::on_msg_received : val $val\n";
	my $csv = Text::CSV->new();
	$csv->parse($val);
	my @list = $csv->fields();
	pop @list if $list[$#list] eq '';
	$ret->{$field_name} = \@list;
      }
      else {
	$ret->{$field_name} = $val;
      }
    }
  }
  #  print STDERR "ret : ".Dumper($ret)."\n";
  my ($fun, @fun_args) = @{$cb};
  $fun->($sendername, $msg_class, $msg_name, $ret, @fun_args);
}


sub get_regexp {
  my ($sender_name, $msg_class, $msg_name, $known_fields) = @_;
  return print_msg($sender_name, $msg_class, $msg_name, $known_fields, 1);
}


sub print_msg {
  my ($sender_name, $msg_class, $msg_name, $args, $is_regexp) = @_;
  warn "no such class : $msg_class" unless defined $classes_by_name->{$msg_class};
  my $msg =  $classes_by_name->{$msg_class}->{$msg_name};
  if (!defined $msg) {
    warn "unknown message  $msg_class, $msg_name\n";
    return;
  }
  my $nb_fields = @{$msg};
  my $output = $sender_name ne "" ? $sender_name : $app_name;
  $output = "^".$output if ($is_regexp);
  $output .= " $msg_name";
  for (my $i=0; $i<$nb_fields; $i++) {
    my $field_name = $msg->[$i]->{name};
    my $prefix = $msg->[$i]->{prefix};
    my $value = $args->{$field_name};
    $output .= " ";
    $output .= $prefix if ($prefix);
    if ($is_regexp and not $value) {
      $output .= get_field_regexp($msg->[$i]->{type});
    } else {
      confess "missing value for field $field_name in message $msg_name" unless defined $value;
      $output .= $value;
    }
  }
  return $output;
}

sub get_field_regexp {
  my ($type) = @_;
  return "(.+)\$" if ($type eq "ssv");
  return "(\\S+)";
}

sub read_protocol {
 my ($filename, $classe_name) = @_;
 my $parser = XML::DOM::Parser->new();
 my $doc = $parser->parsefile($filename);
 my $protocol = $doc->getElementsByTagName('protocol')->[0];
 foreach my $class ($protocol->getElementsByTagName('class')) {
   if ($class->getAttribute("name") eq $classe_name) {
     my $messages_by_name = {};
     foreach my $message ($class->getElementsByTagName('message')) {
       my $message_name = $message->getAttribute("name");
       my @msg_a = ();
       if ($classe_name eq "aircraft_info") { push @msg_a, { name =>"id", type =>"string"}};
       foreach my $field ($message->getElementsByTagName('field')) {
	 my $field_name = $field->getAttribute("name");
	 my $field_h = {   name => $field->getAttribute("name"),
			   type => $field->getAttribute("type"),
			   format => $field->getAttribute("format"),
			   unit => $field->getAttribute("unit"),
			   values => $field->getAttribute("values"),
		       };
	 push @msg_a, $field_h;
	 #	 print "$classe_name $message_name $field_name\n";
       }
       $messages_by_name->{$message_name} = \@msg_a;
     }
   $classes_by_name->{$classe_name} = $messages_by_name;
   }
 }
 # use Data::Dumper;
 # print Dumper($messages_by_name);
}




1;

