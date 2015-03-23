# Hey Emacs, this is a -*- makefile -*-
#
# Copyright (C) 2003-2014 The Paparazzi Team
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.

# Quiet compilation
Q=@

include ../../Makefile.ocaml
OCAMLBUILD=ocamlbuild -use-ocamlfind

GEN_ML := $(shell ls *.ml)
GEN_BYTE = $(GEN_ML:.ml=.byte)
GEN_NATIVE = $(GEN_ML:.ml=.native)
GENS = $(basename $(GEN_ML))

all: byte

#
# build without creating links, then make symlinks ourselves so they still have .out ending
#
byte:
	@echo "Build generators (bytecode)"
	$(Q)$(OCAMLBUILD) -no-links $(GEN_BYTE)
	$(Q)$(foreach file,$(GENS), ln -fs _build/$(file).byte $(file).out;)

native:
	@echo "Build generators (native)"
	$(Q)$(OCAMLBUILD) -no-links $(GEN_NATIVE)
	$(Q)$(foreach file,$(GENS), ln -fs _build/$(file).native $(file).out;)

clean:
	$(Q)rm -f *.cm* *.out *~ .depend
	$(Q)$(OCAMLBUILD) -clean

.PHONY: all byte native clean
