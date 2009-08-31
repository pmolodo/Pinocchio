CC = g++
CCFLAGS = -c -O3 -Wall -fPIC
#CCFLAGS = -c -g3 -O0 -Wall -fPIC

TARGET = AttachWeights

$(TARGET) : stdfx.o $(TARGET).o
	gcc -O3 -Wall -fPIC -o $(TARGET) stdafx.o $(TARGET).o ../Pinocchio/libpinocchio.a -lstdc++

stdfx.o : stdafx.cpp stdafx.h
	$(CC) $(CCFLAGS) stdafx.cpp
	
$(TARGET).o : stdafx.h
	$(CC) $(CCFLAGS) AttachWeights.cpp

clean :
	rm -f *.o
	rm -f attachWeights 
