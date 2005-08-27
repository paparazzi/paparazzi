OCAMLOPT0 = ocamlopt
OCAMLC    = ocamlc

MLFLAGS   = -I +lablgtk2 -I +lablGL -I +camlimages -I ../../lib/ocaml

OCAMLOPT = $(OCAMLOPT0) $(OCAMLOPT_OPTIONS)

SRC = mapGL.ml

OBJS= $(SRC:.ml=.cmo)

LINK= $(OCAMLC) $(MLFLAGS)
LIBS_CI = ci_core.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_png.cma \
	ci_xpm.cma ci_ps.cma ci_freetype.cma
STDLIBS = unix.cma str.cma xml-light.cma lablgtk.cma lablgnomecanvas.cma lablgl.cma lablgtkgl.cma $(LIBS_CI)
ADD_LIBS = glibivy-ocaml.cma lib-pprz.cma xlib-pprz.cma
CLIBS = -cclib -lpthread

all: mapGL

clean:
	\rm -f mapGL *.cm* *.o *.a *~ *.opt *.out *.top *.output *obj *exe \
	stars_lexer.ml stars_parser.mli stars_parser.ml .depend

# Executables
mapGL: mapGL.ml
	$(OCAMLC) $(MLFLAGS) $(STDLIBS) gtkInit.cmo $(ADD_LIBS) -o $@ $< $(CLIBS) # To check
	cat ../../../pprz_src_test.sh > $@
	echo 'ocaml -I +lablgtk2 -I +lablGL -I +camlimages -I $$PAPARAZZI_SRC/sw/lib/ocaml $(STDLIBS) gtkInit.cmo $(ADD_LIBS) $$PAPARAZZI_SRC/sw/ground_segment/visu3d/$< $$*' >> $@
	chmod a+x $@

# Do not edit below this line

.depend:
	ocamldep *.mli *.ml *.mly *.mll > .depend

.SUFFIXES: .ml .mli .cmo .cmi .cmx

.ml.cmo:
	$(OCAMLC) $(MLFLAGS) -labels -w s -c $<
.mli.cmi:
	$(OCAMLC) $(MLFLAGS) -labels -w s -c $<
.ml.cmx:
	$(OCAMLOPT) $(MLFLAGS) -labels -w s -c $<

ifneq ($(MAKECMDGOALS),clean) 
-include .depend
endif
