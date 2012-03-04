#!/usr/bin/perl -w

use Test::More tests => 5;

$|++; 

ok(-f "$ENV{'PAPARAZZI_SRC'}/sw/ground_segment/tmtc/server", "The server program exists");
ok(-f "$ENV{'PAPARAZZI_SRC'}/sw/ground_segment/tmtc/link", "The link program exists");
ok(-f "$ENV{'PAPARAZZI_SRC'}/sw/ground_segment/tmtc/messages", "The messages program exists");
ok(-f "$ENV{'PAPARAZZI_SRC'}/sw/ground_segment/tmtc/settings", "The settings program exists");
ok(-f "$ENV{'PAPARAZZI_SRC'}/sw/ground_segment/cockpit/gcs", "The gcs program exists");

use Data::Dumper;
#warn Dumper(\%ENV);

