CC = g++
CFLAGS = -g -Wall -std=c++11 -pedantic
all: calib_local calib_online

OPENCV = `pkg-config opencv --cflags --libs`
THOR = -lthor_static
LIBS = $(OPENCV) $(THOR)

calib_local:
	$(CC) $(CFLAGS) -o calib_local calib_local.cc $(LIBS)

calib_online:
	$(CC) $(CFLAGS) -o calib_online calib_online.cc $(LIBS)

clean:
	rm calib_local
	rm calib_online
