OCAMLOPT0 = ocamlopt
OCAMLC    = ocamlc

MLFLAGS   = -I +lablgtk2 -I +lablGL -I +camlimages -I ../../lib/ocaml

OCAMLOPT = $(OCAMLOPT0) $(OCAMLOPT_OPTIONS)

SRC = mapGL.ml

OBJS= $(SRC:.ml=.cmo)

LINK= $(OCAMLC) $(MLFLAGS)
LIBS_CI = ci_core.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_png.cma \
	ci_xpm.cma ci_ps.cma ci_freetype.cma
STDLIBS = unix.cma str.cmxa xml-light.cma lablgtk.cma lablgl.cma lablgtkgl.cma $(LIBS_CI)
ADD_LIBS = lib.cma xlib.cma glibivy-ocaml.cma
CLIBS = -cclib -lpthread

all: mapGL.opt

clean:
	\rm -f *.cm* *.o *.a *~ *.opt *.out *.top *.output *obj *exe \
	stars_lexer.ml stars_parser.mli stars_parser.ml .depend

# Executables
mapGL.out: $(OBJS)
	$(OCAMLC) $(MLFLAGS) $(STDLIBS) gtkInit.cmo $(ADD_LIBS) -o $@ $(OBJS_3D) $(OBJS) $(CLIBS)

mapGL.opt: $(OBJS:.cmo=.cmx)
	$(OCAMLOPT) $(MLFLAGS) $(STDLIBS:.cma=.cmxa) gtkInit.cmx $(ADD_LIBS:.cma=.cmxa) -o $@ $(OBJS:.cmo=.cmx) $(CLIBS)

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
