# Quiet compilation
Q=@

doc_pprz_algebra.pdf: headfile.tex
	pdflatex $<
bib:
	bibtex headfile

clean:
	$(Q)rm -f *~ *.aux *.bbl *.blg *.log *.out *.toc *.dvi *.ps
	$(Q)find . -name '*~' -exec rm -f {} \;
