#!/usr/bin/perl -w

use strict;
use File::Basename;
use Getopt::Long;
use Data::Dumper;
use XML::DOM;

my $destdir="/usr";
my $install = undef;
my $uninstall = undef;
my @sections;

GetOptions("install" => \$install, 
	   "uninstall" => \$uninstall,
	   "destdir=s" => \$destdir);

read_xml("./conf/install.xml");

foreach my $section (@sections) {
  my ($inst_dir, $files) = @{$section};
  do_install($inst_dir, $files) if ($install);
  do_uninstall($inst_dir, $files) if ($uninstall);
}

sub do_install {
  my ($dest_dir, $files) = @_;
  `install -d $dest_dir`;# or warn "creation of directory $dest_dir failed";
  foreach my $file (@{$files}) {
    my ($path, $new_name) = @{$file};
    print "installing file $path in $dest_dir ".($new_name?"as $new_name":"")."\n";
    my $cmd = "install $path $dest_dir".($new_name?"/$new_name":"");
    `$cmd`;# or warn "intall of $path failed";
  }
}

sub do_uninstall {
  my ($dest_dir, $files) = @_;
  foreach my $file (@{$files}) {
    my ($path, $new_name) = @{$file};
    my $to_be_removed = $dest_dir."/".($new_name?"$new_name":basename($path));
    print "removing $to_be_removed\n";
    `rm -f $to_be_removed`;
  }
}

sub read_xml {
  my ($filename) = @_;
  my $parser = XML::DOM::Parser->new();
  my $doc = $parser->parsefile($filename);
  my $cp = $doc->getElementsByTagName("install")->[0];
  my $sections = $cp->getElementsByTagName("section");
  foreach my $section (@{$sections}) {
    my $section_name = $section->getAttribute('name');
    my $dest_loc = $destdir."/".$section->getAttribute('dest');
    my $files = $section->getElementsByTagName("file");
    my $file_a = [];
    foreach my $file (@{$files}) {
      push @{$file_a}, [$file->getAttribute('name'), $file->getAttribute('new_name')];
    }
    my $dirs =  $section->getElementsByTagName("directory");
    foreach my $dir (@{$dirs}) {
      my $dirname=$dir->getAttribute('name');
      opendir(DIR,$dirname);
      my @dir_files = grep { -f "$dirname/$_" } readdir(DIR);
      foreach my $foo (@dir_files) { print "$foo\n"; }


      map { s#^(.*)#$dirname/$1# } @dir_files;
      closedir(DIR);
      foreach my $file (@dir_files) {
	push @{$file_a}, [$file, $file];
      }
    }
    push @sections, [$dest_loc, $file_a];
  }
}

sub read_fs_dir {
#  my (



}
