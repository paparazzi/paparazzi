package Paparazzi::IvyProtocol;

use strict;
use XML::DOM;

use Data::Dumper;

my $classes_by_name = {};

my $req_id = int rand(65534);
# TODO: make real timeout instead of single request
my $res_regexp = undef;

sub request_message {
  my ($msg_class, $msg_name, $args, $ivy, $callback) = @_;
  my $message =  $classes_by_name->{$msg_class}->{$msg_name};
  return unless defined $message;
  # unbind previous request
  $ivy->bindRegexp($res_regexp) if defined $res_regexp;
  $res_regexp = "^(".$args->{id}.") ".$msg_name."_RES ".++$req_id;
  foreach my $field (@{$message}) {$res_regexp.= " (\\S+)" unless $field->{name} eq "id"};
  print "res_regexp \'$res_regexp\'\n";
  $ivy->bindRegexp ($res_regexp, $callback);
  my $req_msg = $args->{id}." ".$msg_name."_REQ ".$req_id;
  print "req_msg \'$req_msg\'\n";
  $ivy->sendMsgs($req_msg);
  return $req_id;
}

sub bind_request_message {
  my ($msg_class, $msg_name, $args, $ivy, $callback) = @_;
  


}

sub bind_message {
  my ($msg_class, $msg_name, $args, $ivy, $callback) = @_;
  my $regexp = get_regexp($msg_class, $msg_name, $args);
  print ((defined $callback ? "binding":"removing binding")." on \'$regexp\'\n");
  $ivy->bindRegexp ($regexp, $callback);
}

sub get_regexp {
  my ($msg_class, $msg_name, $args) = @_;
  warn "no such class : $msg_class" unless defined $classes_by_name->{$msg_class};
  my $message =  $classes_by_name->{$msg_class}->{$msg_name};
  return "" if !defined $message;
  my $nb_fields = @{$message};
  my $regexp = "";
  if( $msg_class eq "ground") {
    $regexp = "^".$msg_class." ".$msg_name;
    foreach (@{$message}) {$regexp.= " (\\S+)"};
  }
  elsif ( $msg_class eq "aircraft_info") {
    $regexp = "^(".$args->{id}.") ".$msg_name;
    foreach my $field (@{$message}) {$regexp.= " (\\S+)" unless $field->{name} eq "id"};
  }
  else {
    $regexp = "^.*".$msg_name."\\s+";
    foreach (@{$message}) {$regexp.= " (\\S+)"};
  }
  return $regexp;
}

sub get_values_by_name {
  my ($msg_class, $msg_name, $ivy_args) = @_;
  my $values_by_name = {};
  my $message =  $classes_by_name->{$msg_class}->{$msg_name};
  return {} unless defined $message;
  for (my $i=0; $i<@{$message}; $i++) {
    my $field = $message->[$i];
    $values_by_name->{$field->{name}} = $ivy_args->[$i+1];
  }
  return $values_by_name;
}

sub getMsg {
 my ($msg_class, $msg_name, $args) = @_;
 my $message =  $classes_by_name->{$msg_class}->{$msg_name};
 my $str = "";
 if ( $msg_class eq "ground") { 
   $str .= $msg_class." ".$msg_name;
 }
 else {
   $str .= $msg_name;
 }
 foreach my $field (@{$message}) {
   $str.= " ".$args->{$field->{name}};
 }
 return $str;
}

sub sendMsg {
  my ($ivy, $msg_class, $msg_name, $args) = @_;
  $ivy->sendMsgs(getMsg($msg_class, $msg_name, $args));
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

