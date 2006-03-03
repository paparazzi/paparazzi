OCAMLC=ocamlc
OCAMLOPT=ocamlopt
INCLUDES=-I +lablgtk2 -I +camlimages -I ../../lib/ocaml
LIBS=glibivy-ocaml.cma lablgtk.cma ci_core.cma ci_png.cma ci_gif.cma ci_jpeg.cma ci_tiff.cma ci_bmp.cma ci_ppm.cma ci_ps.cma lib-pprz.cma lablgnomecanvas.cma xlib-pprz.cma
CMXA=$(LIBS:.cma=.cmxa)


all : map2d

opt : map2d.opt


map2d : map2d.ml
	$(OCAMLC) -thread -custom $(INCLUDES) $(LIBS) threads.cma gtkThread.cmo gtkInit.cmo $< -o $@

map2d.opt : map2d.cmx
	$(OCAMLOPT) $(INCLUDES) str.cmxa unix.cmxa xml-light.cmxa $(LIBS:.cma=.cmxa) gtkInit.cmx $< -o $@


.SUFFIXES: .ml .mli .cmo .cmi .cmx

.ml.cmo:
	$(OCAMLC) $(INCLUDES) -labels -w s -c $<
.mli.cmi:
	$(OCAMLC) $(INCLUDES) -labels -w s -c $<
.ml.cmx:
	$(OCAMLOPT) $(INCLUDES) -labels -w s -c $<

clean:
	rm -f *~* *.cm* *.o *.out *.opt map2d
