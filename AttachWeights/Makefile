CC = g++
CCFLAGS = -c -O3 -Wall -fPIC

attachWeights : stdfx.o AttachWeights.o
gcc -O3 -Wall -fPIC -o attachWeights stdafx.o AttachWeights.o ../Pinocchio/libpinocchio.a -lstdc++

stdfx.o : stdafx.cpp stdafx.h
$(CC) $(CCFLAGS) stdafx.cpp
AttachWeights.o : AttachWeights.cpp stdafx.h
$(CC) $(CCFLAGS) AttachWeights.cpp
clean :
rm attachWeights stdafx.o AttachWeights.o
