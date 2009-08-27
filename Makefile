# Makefile for Pinocchio

dirs = Pinocchio AttachWeights
# DemoUI is a windows prog, and on windows, VisualC++ is used,
# so we can ignore it here
# dirs = Pinocchio AttachWeights DemoUI

# Define a standard makePerDir command, which goes into
# each dir, and runs make $(makerule)
define makePerDir
for dir in $(dirs); \
do \
	cd $$dir && { $(MAKE) $(makerule); cd ..; }; \
done
endef

nullstring :=

all: makerule = $(nullstring)
depend: makerule = depend 
clean: makerule = clean 

all depend clean:
	$(makePerDir)


#all:
#	cd Pinocchio && $(MAKE)
#	cd DemoUI && $(MAKE)
#
#depend:
#	cd Pinocchio && $(MAKE) depend
#	cd DemoUI && $(MAKE) depend
#
#clean:
#	cd Pinocchio && $(MAKE) clean
#	cd DemoUI && $(MAKE) clean


# DO NOT DELETE
