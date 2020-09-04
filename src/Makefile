CC = x86_64-linux-gnu-g++ -g
INCPATH = -I/usr/xenomai/include -I/usr/xenomai/include -I/usr/xenomai/include/posix -I../include -Wl,@/usr/xenomai/lib/posix.wrappers

LIBS = -lnative -L/usr/xenomai/lib -lpthread_rt -lxenomai -lpthread -lrt -lm -L/home/igh/etherlabmaster-code/lib/.libs -lethercat_rtdm
 
CFLAGS = -D_GNU_SOURCE -D_REENTRANT -D__XENO__ 

TARGET = robot 

# Files 
SOURCES2 = control.cpp 
OBJECTS2 = control.o 

SOURCES1 = communication.cpp 
OBJECTS1 = communication.o 

# Build rules 
$(TARGET): $(OBJECTS1) $(OBJECTS2) 
	$(CC) $(LFLAGS) -o $(TARGET) $(OBJECTS1) $(OBJECTS2) $(LIBS) 

# Compile 
$(OBJECTS2) : $(SOURCES2)
	$(CC) $(INCPATH) $(CFLAGS) -o $(OBJECTS2) -c $(SOURCES2) -lm

# Compile 
$(OBJECTS1) : $(SOURCES1)
	$(CC) $(INCPATH) $(CFLAGS) -o $(OBJECTS1) -c $(SOURCES1) -lm

clean: 
	rm $(TARGET) *.o 
