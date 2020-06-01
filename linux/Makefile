CC=clang

ifndef OPENNI2_REDIST
	error OPENNI2_REDIST is not set
endif

ifndef OPENNI2_INCLUDE
	error OPENNI2_INCLUDE is not set
endif

CFLAGS=-shared -fPIC -O2 -std=c++11 -I ../src -I ${OPENNI2_INCLUDE} $(shell pkg-config opencv4 --cflags)
LIBS=-L$(OPENNI2_REDIST) -lOpenNI2 -pthreads $(shell pkg-config opencv4 --libs)

SRC=$(shell find launchpad -name '*.cpp')
HEADERS=$(shell find launchpad -name '*.h')

libMagicMotion.so: $(find ../src -name *.c -o -name *.cpp -o -name *.h)
	${CC} ../src/magic_motion.cpp ${CFLAGS} -o $@ ${LIBS}

.PHONY: clean
clean:
	rm -f libMagicMotion.so

