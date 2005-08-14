package Paparazzi::IvyProtocol;

use strict;
use XML::DOM;
use Carp;
use Ivy;

use Data::Dumper;

my $classes_by_name = {};
my $ivy = undef;
my $app_name = "";

sub init {
  my (%options) = @_;
  read_protocol(%options->{-file}, 'ground');
  $app_name = %options->{-app_name};
  Ivy->init( -ivyBus        => %options->{-ivy_bus},
             -appName       => %options->{-app_name},
             -loopMode      => %options->{-loop_mode},
             -messWhenReady => 'READY',
           );
  $ivy = Ivy->new ();
  $ivy->start();
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
  my $nb_fields = @{$msg};
  for (my $i=0; $i<$nb_fields; $i++) {
    my $field_name = $msg->[$i]->{name};
    if ($known_fields->{$field_name}) {
      $ret->{$field_name} = $known_fields->{$field_name};
    } else {
      $ret->{$field_name} = shift @matched_regexps;
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
  return "" if !defined $msg;
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



# sub bind_message {
#   my ($msg_class, $msg_name, $args, $ivy, $callback) = @_;
#   my $regexp = get_regexp($msg_class, $msg_name, $args);
#   print ((defined $callback ? "binding":"removing binding")." on \'$regexp\'\n");
#   $ivy->bindRegexp ($regexp, $callback);
# }

# my $req_id = int rand(65534);
# # TODO: make real timeout instead of single request
# my $res_regexp = undef;

# sub request_message {
#   my ($msg_class, $msg_name, $args, $ivy, $callback) = @_;
#   my $message =  $classes_by_name->{$msg_class}->{$msg_name};
#   return unless defined $message;
#   # unbind previous request
#   $ivy->bindRegexp($res_regexp) if defined $res_regexp;
#   $res_regexp = "^(".$args->{id}.") ".$msg_name."_RES ".++$req_id;
#   foreach my $field (@{$message}) {$res_regexp.= " (\\S+)" unless $field->{name} eq "id"};
#   print "res_regexp \'$res_regexp\'\n";
#   $ivy->bindRegexp ($res_regexp, $callback);
#   my $req_msg = $args->{id}." ".$msg_name."_REQ ".$req_id;
#   print "req_msg \'$req_msg\'\n";
#   $ivy->sendMsgs($req_msg);
#   return $req_id;
# }

# sub bind_request_message {
#   my ($msg_class, $msg_name, $args, $ivy, $callback) = @_;
  


# }


# sub get_regexp {
#   my ($msg_class, $msg_name, $args) = @_;
#   warn "no such class : $msg_class" unless defined $classes_by_name->{$msg_class};
#   my $message =  $classes_by_name->{$msg_class}->{$msg_name};
#   return "" if !defined $message;
#   my $nb_fields = @{$message};
#   my $regexp = "";
#   if( $msg_class eq "ground") {
#     $regexp = "^".$msg_class." ".$msg_name;
#     foreach (@{$message}) {$regexp.= " (\\S+)"};
#   }
#   elsif ( $msg_class eq "aircraft_info") {
#     $regexp = "^(".$args->{id}.") ".$msg_name;
#     foreach my $field (@{$message}) {$regexp.= " (\\S+)" unless $field->{name} eq "id"};
#   }
#   else {
#     $regexp = "^.*".$msg_name."\\s+";
#     foreach (@{$message}) {$regexp.= " (\\S+)"};
#   }
#   return $regexp;
# }

# sub get_values_by_name {
#   my ($msg_class, $msg_name, $ivy_args) = @_;
#   my $values_by_name = {};
#   my $message =  $classes_by_name->{$msg_class}->{$msg_name};
#   return {} unless defined $message;
#   for (my $i=0; $i<@{$message}; $i++) {
#     my $field = $message->[$i];
#     $values_by_name->{$field->{name}} = $ivy_args->[$i+1];
#   }
#   return $values_by_name;
# }

# sub getMsg {
#  my ($msg_class, $msg_name, $args) = @_;
#  my $message =  $classes_by_name->{$msg_class}->{$msg_name};
#  my $str = "";
#  if ( $msg_class eq "ground") { 
#    $str .= $msg_class." ".$msg_name;
#  }
#  else {
#    $str .= $msg_name;
#  }
#  foreach my $field (@{$message}) {
#    $str.= " ".$args->{$field->{name}};
#  }
#  return $str;
# }

# sub sendMsg {
#   my ($ivy, $msg_class, $msg_name, $args) = @_;
#   $ivy->sendMsgs(getMsg($msg_class, $msg_name, $args));
# }




1;

