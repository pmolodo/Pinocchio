# Makefile for Pinocchio

all:
	cd Pinocchio && $(MAKE)
	cd DemoUI && $(MAKE)

depend:
	cd Pinocchio && $(MAKE) depend
	cd DemoUI && $(MAKE) depend

clean:
	cd Pinocchio && $(MAKE) clean
	cd DemoUI && $(MAKE) clean


# DO NOT DELETE
