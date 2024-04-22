
#LINUX
COMMON=-O2 -I../../include -L../../lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../../lib
LIBS = -lmujoco -lglfw
CC = g++

# include folder 
INCDIR = -I./include -I../../include

HEADDIR = include

ROOT = main

all:

	$(CC) $(COMMON) $(INCDIR) main.c $(LIBS) -o $(ROOT)

main.o:
	$(CC) $(COMMON) -c main.c

clean:
	rm *.o $(ROOT)
