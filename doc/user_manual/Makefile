
# 
#   $Id$
#   Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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
# 




all: fly_by_wire.png fly_by_wire.eps overall.png overall.eps
	latex paparazzi
	makeindex paparazzi
	latex paparazzi
	latex paparazzi
	pdflatex paparazzi


paparazzi.dvi: paparazzi.tex
	latex $<

paparazzi.pdf: paparazzi.tex
	pdflatex $<

%.png:%.dia
	dia -e $@ -t png $<
%.eps:%.dia
	dia -e $@ -t png $<

clean:
	rm -rf *~ paparazzi.dvi paparazzi.pdf fly_by_wire.png *.log *.aux *.info  *.eps *.idx *.ilg *.ind *.out *.texi *.hind
