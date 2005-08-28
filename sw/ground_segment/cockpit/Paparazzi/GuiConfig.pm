package Paparazzi::GuiConfig;

use strict;
require XML::DOM;

use Paparazzi::Traces;

my $resources = {};

sub init {
  my ($foo, $filename) = @_;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  my $sections_xml = $doc->getElementsByTagName('section');
  foreach my $section (@{$sections_xml}) {
    my $section_name = $section->getAttribute('name');
    my $resources_xml = $section->getElementsByTagName('resource');
    foreach my $resource (@{$resources_xml}) {
      my $res_name = $resource->getAttribute('name');
      my $res_val = $resource->getAttribute('value');
      $resources->{$section_name}->{$res_name} = $res_val;
    }
  }
}

sub get_resource {
  my ($section_name, $resource_name) = @_;
  unless (defined $resources->{$section_name} and defined $resources->{$section_name}->{$resource_name}) {
    trace(TRACE_ERROR, "GuiConfig::get_resource : non existant section/resource $section_name $resource_name");
  }
  my $value = $resources->{$section_name}->{$resource_name};
  trace(TRACE_JUNK, "GuiConfig::get_resource : $section_name $resource_name -> $value");
  return $value;
}
