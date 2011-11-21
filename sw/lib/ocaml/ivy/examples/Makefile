# $Id: Makefile,v 1.4 2006/07/28 16:38:53 poine Exp $

OCAMLC = ocamlc -I .. 
OCAMLMLI = ocamlc -I .. 
OCAMLOPT = ocamlopt
OCAMLDEP=ocamldep

all: ivyprobe.out glibivyprobe.out

ivyprobe.out : ivyprobe.cmo ivyivyprobe.cmo
	$(OCAMLC) -custom -o $@ unix.cma ivy-ocaml.cma $^

glibivyprobe.out : ivyprobe.cmo glibivyprobe.cmo
	$(OCAMLC) -custom -o $@ unix.cma glibivy-ocaml.cma $^

#tkivyprobe.out : ivyprobe.cmo tkivyprobe.cmo
#	$(OCAMLC) -custom -o $@ unix.cma -I +labltk labltk.cma -I . $^
#tkivyprobe.cmo : OCAMLFLAGS=-I +labltk

%.cmo : %.ml
	$(OCAMLC) -c $<

clean:
	\rm -f *.cm* *.o *.a .depend *~ *.out *.opt .depend *.so

.depend:
	$(OCAMLDEP) $(INCLUDES) *.mli *.ml > .depend

ifneq ($(MAKECMDGOALS),clean)
include .depend
endif
