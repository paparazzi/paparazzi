doc_libeknav.pdf: headfile.tex
	pdflatex $<
bib:
	bibtex headfile

clean:
	rm -f *~ *.aux *.bbl *.blg *.log *.out *.toc *.pdf *.dvi *.ps
	find . -name '*~' -exec rm -f {} \;
