# Makefile for DemoUI
CC = g++
CCFLAGS = -c -Wall -O3 -I../Pinocchio/
LIBS = -lm -L../Pinocchio/ -lpinocchio -lfltk -lfltk_gl

OBJECTS = demoUI.o MyWindow.o defmesh.o processor.o motion.o filter.o

DemoUI: $(OBJECTS)
	$(CC) $(OBJECTS) $(LIBS) -o DemoUI

.cpp.o:
	$(CC) $(CCFLAGS) $<


#do a makedepend and remove all the external dependencies from the makefile
#works on my system, no one else should need to do it.
depend:
	makedepend *.cpp > /dev/null >& 1
	perl -pi -e 's/\.c\.o/\.o/' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/\/u[Ss]r[a-zA-Z0-9\+\-\/\.\_]*//' Makefile
	perl -pi -e 's/^[a-zA-Z].*\.o\:\s*$$//' Makefile

clean:
	rm -f $(OBJECTS) DemoUI *~ core *.out *.bak


# DO NOT DELETE

MyWindow.o:   MyWindow.h
MyWindow.o: ../Pinocchio/mesh.h ../Pinocchio/vector.h
MyWindow.o: ../Pinocchio/hashutils.h ../Pinocchio/mathutils.h
MyWindow.o: ../Pinocchio/Pinocchio.h ../Pinocchio/rect.h
MyWindow.o: ../Pinocchio/transform.h DisplayMesh.h ../Pinocchio/skeleton.h
MyWindow.o: ../Pinocchio/graphutils.h
defmesh.o: defmesh.h ../Pinocchio/attachment.h ../Pinocchio/mesh.h
defmesh.o: ../Pinocchio/vector.h ../Pinocchio/hashutils.h
defmesh.o: ../Pinocchio/mathutils.h 
defmesh.o: ../Pinocchio/Pinocchio.h ../Pinocchio/rect.h
defmesh.o: ../Pinocchio/skeleton.h ../Pinocchio/graphutils.h
defmesh.o: ../Pinocchio/transform.h filter.h ../Pinocchio/matrix.h
defmesh.o: ../Pinocchio/vector.h ../Pinocchio/transform.h DisplayMesh.h
defmesh.o: ../Pinocchio/mesh.h motion.h ../Pinocchio/intersector.h
defmesh.o: ../Pinocchio/vecutils.h
demoUI.o:  MyWindow.h
demoUI.o: ../Pinocchio/mesh.h ../Pinocchio/vector.h ../Pinocchio/hashutils.h
demoUI.o: ../Pinocchio/mathutils.h 
demoUI.o: ../Pinocchio/Pinocchio.h ../Pinocchio/rect.h
demoUI.o: ../Pinocchio/transform.h DisplayMesh.h processor.h
filter.o: filter.h ../Pinocchio/matrix.h ../Pinocchio/mathutils.h
filter.o:  ../Pinocchio/Pinocchio.h
filter.o: ../Pinocchio/vector.h ../Pinocchio/transform.h
filter.o: ../Pinocchio/vector.h ../Pinocchio/hashutils.h ../Pinocchio/deriv.h
motion.o: motion.h ../Pinocchio/transform.h ../Pinocchio/vector.h
motion.o: ../Pinocchio/hashutils.h ../Pinocchio/mathutils.h
motion.o:  ../Pinocchio/Pinocchio.h
motion.o: ../Pinocchio/skeleton.h ../Pinocchio/graphutils.h
motion.o: ../Pinocchio/utils.h 
processor.o:  processor.h MyWindow.h
processor.o: ../Pinocchio/mesh.h ../Pinocchio/vector.h
processor.o: ../Pinocchio/hashutils.h ../Pinocchio/mathutils.h
processor.o:  ../Pinocchio/Pinocchio.h
processor.o: ../Pinocchio/rect.h ../Pinocchio/transform.h DisplayMesh.h
processor.o: ../Pinocchio/skeleton.h ../Pinocchio/graphutils.h
processor.o: ../Pinocchio/utils.h ../Pinocchio/debugging.h
processor.o: ../Pinocchio/attachment.h ../Pinocchio/mesh.h
processor.o: ../Pinocchio/skeleton.h ../Pinocchio/transform.h
processor.o: ../Pinocchio/pinocchioApi.h ../Pinocchio/quaddisttree.h
processor.o: ../Pinocchio/dtree.h ../Pinocchio/indexer.h
processor.o: ../Pinocchio/multilinear.h ../Pinocchio/intersector.h
processor.o: ../Pinocchio/vecutils.h ../Pinocchio/pointprojector.h
processor.o: ../Pinocchio/debugging.h ../Pinocchio/attachment.h defmesh.h
processor.o: filter.h ../Pinocchio/matrix.h ../Pinocchio/vector.h motion.h
