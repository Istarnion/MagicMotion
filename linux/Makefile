CC=clang

ifndef OPENNI2_REDIST
	error OPENNI2_REDIST is not set
endif

ifndef OPENNI2_INCLUDE
	error OPENNI2_INCLUDE is not set
endif

CFLAGS=-shared -fPIC -g -I ../src -I ${OPENNI2_INCLUDE}
LIBS=-L$(OPENNI2_REDIST) -lOpenNI2 -pthreads

SRC=$(shell find launchpad -name '*.cpp')
HEADERS=$(shell find launchpad -name '*.h')

libMagicMotion.so: $(find ../src -name *.c -o -name *.cpp -o -name *.h)
	${CC} ../src/magic_motion.cpp ${CFLAGS} -o $@ ${LIBS}

.PHONY: clean
clean:
	rm -f libMagicMotion.so

