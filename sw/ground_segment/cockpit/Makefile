OCAMLC=ocamlc -g
OCAMLOPT=ocamlopt
INCLUDES=-I +lablgtk2 -I +camlimages -I ../../lib/ocaml
LIBS=glibivy-ocaml.cma lablgtk.cma ci_core.cma ci_png.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_ps.cma lib.cma lablgnomecanvas.cma xlib.cma
CMXA=$(LIBS:.cma=.cmxa)

SRC = map2d.ml
CMO = $(SRC:.ml=.cmo)
CMX = $(SRC:.ml=.cmx)

all : map2d.opt


map2d.out : $(CMO)
	$(OCAMLC) $(INCLUDES) $(LIBS) gtkInit.cmo $(CMO) -o $@


map2d.opt : $(CMX)
	$(OCAMLOPT) str.cmxa unix.cmxa xml-light.cmxa $(INCLUDES) $(CMXA) gtkInit.cmx $(CMX) -o $@

map2d.run:
	lablgtk2 str.cma unix.cma xml-light.cma -I +camlimages -I ../../lib/ocaml glibivy-ocaml.cma ci_core.cma ci_png.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_ps.cma lib.cma xlib.cma map2d.ml


.SUFFIXES: .ml .mli .cmo .cmi .cmx

.ml.cmo:
	$(OCAMLC) $(INCLUDES) -labels -w s -c $<
.mli.cmi:
	$(OCAMLC) $(INCLUDES) -labels -w s -c $<
.ml.cmx:
	$(OCAMLOPT) $(INCLUDES) -labels -w s -c $<

clean:
	rm -f *~* *.cm* *.o *.out *.opt
