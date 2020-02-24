CC=clang

OPENNI2_INCLUDE=/home/ablemagic/Documents/2-Linux/OpenNI-Linux-x64-2.3/Include
OPENNI2_REDIST=/home/ablemagic/Documents/2-Linux/OpenNI-Linux-x64-2.3/Redist

CFLAGS=-shared -fPIC -g -I ../src -I ${OPENNI2_INCLUDE}
LIBS=-L$(OPENNI2_REDIST) -lOpenNI2

SRC=$(shell find launchpad -name '*.cpp')
HEADERS=$(shell find launchpad -name '*.h')

libMagicMotion.so: $(find ../src -name *.c -name *.cpp -name *.h)
	${CC} ../src/magic_motion.cpp ${CFLAGS} -o $@ ${LIBS}

.PHONY: clean
clean:
	rm -f libMagicMotion.so

