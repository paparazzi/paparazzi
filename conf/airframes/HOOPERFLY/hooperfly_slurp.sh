#!/bin/bash

cd $PAPARAZZI_HOME

find conf -type f -name "$1*" | cpio -p -d -v $1_slurp
cp conf/conf.xml           $1_slurp/conf/conf.xml.$1
cp conf/control_panel.xml  $1_slurp/conf/control_panel.xml.$1
