package Paparazzi::MissionD;

use Tk::ROText;
use XML::DOM;

use base qw/Tk::Frame/;
use strict;

Construct Tk::Widget 'MissionD';

sub ClassInit {
  my ($class, $mw) = @_;
  $class->SUPER::ClassInit($mw);
}

sub Populate {
  my ($self, $args) = @_;
  $self->SUPER::Populate($args);
  my $text = $self->Scrolled('ROText',
			     -scrollbars => 'osoe',
			    );
  $text->pack(-fill => 'both', -expand => "1");
  $self->Advertise('text' => $text);
  $self->{cur_block} = -1;
  $self->{cur_stage} = -1;
#  $self->ConfigSpecs();
#  $self->Delegates();
}

use Data::Dumper;

sub get_block_id {
  my ($no_block) = @_;
  return "block_".$no_block;
}

sub get_stage_id {
  my ($no_block, $no_stage) = @_;
  return "stage_".$no_block."_".$no_stage;
}

sub set_block_and_stage {
  my ($self, $new_block, $new_stage) = @_;
  my $text = $self->Subwidget('text');
  if ($self->{cur_block} != $new_block) {
    $text->tagConfigure(get_block_id($self->{cur_block}), -background => undef);
    $text->tagConfigure(get_block_id($new_block), -background => 'green3');
    $text->tagConfigure(get_stage_id($self->{cur_block}, $self->{cur_stage}), -background => undef);
    $text->tagConfigure(get_stage_id($new_block, $new_stage), -background => 'green1');
    $self->{cur_block} = $new_block;
    $self->{cur_stage} = $new_stage;
  }
  else {
    if ($self->{cur_stage} != $new_stage) {
      $text->tagConfigure(get_stage_id($self->{cur_block}, $self->{cur_stage}), -background => undef);
      $text->tagConfigure(get_stage_id($self->{cur_block}, $new_stage), -background => 'green1');
      $self->{cur_stage} = $new_stage;
    }
  }
}

sub load_flight_plan {
  my ($self, $xmldata) = @_;
  
  my $text = $self->Subwidget('text');
  if (Tk::Exists($text)) {
    $text->delete('0.0', 'end');
  }
  
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parse($xmldata);

  my ($blocks, $blocks_stages);

  foreach my $stage ($doc->getElementsByTagName('stage')) {
    my $block_name = $stage->getAttribute('block_name');
    my $block_no = $stage->getAttribute('block');
    my $stage_no = $stage->getAttribute('stage');
    my $stage_text = "";
    my $stage_kids = $stage->getChildNodes();
    foreach my $kid (@{$stage_kids}) {
      $stage_text = $stage_text.$kid->toString() if $kid->getNodeType() != TEXT_NODE;
    }
    $blocks_stages->{$block_name}->{$stage_text} = get_stage_id($block_no, $stage_no);
    $blocks->{$block_name} = get_block_id($block_no) unless defined $blocks->{block_name};
  }

  #  print Dumper(\$blocks);
  #  print Dumper(\$blocks_stages);

  foreach my $block ($doc->getElementsByTagName('block')){
    my $block_name = $block->getAttribute('name');
    foreach my $line (split (/(\n)/, $block->toString())) {
      my $key = $line;
      $key =~ s/^\s*//; # remove any leading whitespace
      $key =~ s/\s*$//; # remove any trailing whitespace
      if ($key ne "") {
	my $block_id = $blocks->{$block_name};
	my $stage_id = $blocks_stages->{$block_name}->{$key};
	my $tags = [$block_id];
	push(@{$tags}, ($stage_id)) if defined $stage_id;
	$self->Subwidget('text')->insert('end', $line."\n", $tags);
      }
    }
  }
}

1;
