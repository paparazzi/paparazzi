# $Id: Makefile,v 1.20 2007/09/04 19:05:43 brisset Exp $

DESTDIR = /

DEBUG  = n


OCAMLC = ocamlc
OCAMLMLI = ocamlc
OCAMLOPT = ocamlopt -unsafe
OCAMLDEP=ocamldep

ifeq ($(DEBUG),y)
OCAMLFLAGS = -g
else
OCAMLFLAGS =
endif

OCAMLOPTFLAGS=
CFLAGS+=-Wall
OCAMLINC=-I `ocamlc -where`
GLIBINC=`pkg-config --cflags glib-2.0`

LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
  FPIC=-fPIC
endif



IVY = ivy.ml ivyLoop.ml

IVYCMO= $(IVY:.ml=.cmo)
IVYCMI= $(IVY:.ml=.cmi)
IVYMLI= $(IVY:.ml=.mli)
IVYCMX= $(IVY:.ml=.cmx)

GLIBIVY = ivy.ml glibIvy.ml

GLIBIVYCMO= $(GLIBIVY:.ml=.cmo)
GLIBIVYCMI= $(GLIBIVY:.ml=.cmi)
GLIBIVYCMX= $(GLIBIVY:.ml=.cmx)

TKIVY = ivy.ml tkIvy.ml

TKIVYCMO= $(TKIVY:.ml=.cmo)
TKIVYCMI= $(TKIVY:.ml=.cmi)
TKIVYCMX= $(TKIVY:.ml=.cmx)

UNAME = $(shell uname -s)

UNAME = $(shell uname -s)

ifeq ("$(UNAME)","Darwin")
  LIBRARYS = -L /opt/paparazzi/lib -L /opt/local/lib
  INCLUDES = -I /opt/paparazzi/include -I /opt/local/include/
endif

LIBS = ivy-ocaml.cma ivy-ocaml.cmxa glibivy-ocaml.cma glibivy-ocaml.cmxa

all : $(LIBS)

DISTRO=`ocamlc -version`

deb :
	cp debian/changelog.$(DISTRO) debian/changelog
	dpkg-buildpackage -rfakeroot

ivy : ivy-ocaml.cma ivy-ocaml.cmxa
glibivy : glibivy-ocaml.cma glibivy-ocaml.cmxa
tkivy : tkivy-ocaml.cma tkivy-ocaml.cmxa

INST_FILES = $(IVYCMI) $(IVYMLI) glibIvy.cmi $(LIBS) libivy-ocaml.a libglibivy-ocaml.a dllivy-ocaml.so dllglibivy-ocaml.so ivy-ocaml.a glibivy-ocaml.a
# tkIvy.cmi  libtkivy-ocaml.a  dlltkivy-ocaml.so tkivy-ocaml.a

install : $(LIBS)
	mkdir -p $(DESTDIR)/`ocamlc -where`
	cp $(INST_FILES) $(DESTDIR)/`ocamlc -where`

desinstall :
	cd `ocamlc -where`; rm -f $(INST_FILES)

ivy-ocaml.cma : $(IVYCMO) civy.o civyloop.o
	ocamlmklib -o ivy-ocaml $^ $(LIBRARYS)  -livy

ivy-ocaml.cmxa : $(IVYCMX) civy.o civyloop.o
	ocamlmklib -o ivy-ocaml $^ $(LIBRARYS)  -livy

glibivy-ocaml.cma : $(GLIBIVYCMO) civy.o cglibivy.o
	ocamlmklib -o glibivy-ocaml $^ $(LIBRARYS) -lglibivy  `pkg-config --libs glib-2.0` -lpcre

glibivy-ocaml.cmxa : $(GLIBIVYCMX) civy.o cglibivy.o
	ocamlmklib -o glibivy-ocaml $^ $(LIBRARYS) -lglibivy `pkg-config --libs glib-2.0` -lpcre

tkivy-ocaml.cma : $(TKIVYCMO) civy.o ctkivy.o
	ocamlmklib -o tkivy-ocaml $^ $(LIBRARYS) -livy -ltclivy

tkivy-ocaml.cmxa : $(TKIVYCMX) civy.o ctkivy.o
	ocamlmklib -o tkivy-ocaml $^ $(LIBRARYS) -livy -ltclivy

.SUFFIXES:
.SUFFIXES: .ml .mli .mly .mll .cmi .cmo .cmx .c .o .out .opt

.ml.cmo :
	$(OCAMLC) $(OCAMLFLAGS) $(INCLUDES) -c $<
.c.o :

	$(CC) -Wall -c $(FPIC) $(INCLUDES) $(OCAMLINC) $(GLIBINC) $<
.mli.cmi :
	$(OCAMLMLI) $(OCAMLFLAGS) -c $<
.ml.cmx :
	$(OCAMLOPT) $(OCAMLOPTFLAGS) -c $<
.mly.ml :
	ocamlyacc $<
.mll.ml :
	ocamllex $<
.cmo.out :
	$(OCAMLC) -custom -o $@ unix.cma -I . ivy-ocaml.cma $< -cclib -livy
.cmx.opt :
	$(OCAMLOPT) -o $@ unix.cmxa -I . ivy-ocaml.cmxa $< -cclib -livy

clean:
	\rm -fr *.cm* *.o *.a .depend *~ *.out *.opt .depend *.so *-stamp debian/ivy-ocaml debian/files debian/ivy-ocaml.debhelper.log debian/ivy-ocaml.substvars debian/*~

.depend:
	$(OCAMLDEP) $(INCLUDES) *.mli *.ml > .depend

ifneq ($(MAKECMDGOALS),clean)
include .depend
endif
