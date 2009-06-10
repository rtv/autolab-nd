DOCDIR = $(prefix)/share/doc

# build docs manually with 'make doc'
doc:
	./sourcedocs.sh > additional.txt
	(cat Doxyfile; echo "INPUT += ./additional.txt") > input
	doxygen input
	rm -f additional.txt input

# install the latest docs
doc-install: doc 
	install -d autolab-nd $(DOCDIR)
	cp -r -p -v -f html $(DOCDIR)/autolab-nd

# zap the built docs
clean:
	rm -rf html additional.txt

