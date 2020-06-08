CC=clang

HAS_OPENNI=false
HAS_OPENCV=false

SENSOR_INTERFACE=SENSOR_RECORDING
#SENSOR_INTERFACE=SENSOR_OPENNI

CFLAGS=-shared -fPIC -O2 -std=c++11 -pthreads -I ../src -I ../miniz -D${SENSOR_INTERFACE}
LIBS=-lm

ifeq (${HAS_OPENNI},true)
ifndef OPENNI2_REDIST
	error OPENNI2_REDIST is not set
endif

ifndef OPENNI2_INCLUDE
	error OPENNI2_INCLUDE is not set
endif

CFLAGS += -I ${OPENNI2_INCLUDE}
LIBS += -L$(OPENNI2_REDIST) -lOpenNI2
endif

ifeq (${HAS_OPENCV},true)
	CFLAGS += -DHAS_OPENCV $(shell pkg-config opencv4 --cflags)
	LIBS += $(shell pkg-config opencv4 --libs)
endif

SRC=$(shell find launchpad -name '*.cpp')
HEADERS=$(shell find launchpad -name '*.h')

libMagicMotion.so: $(find ../src -name *.c -o -name *.cpp -o -name *.h)
	${CC} ../src/magic_motion.cpp ${CFLAGS} -o $@ ${LIBS}

.PHONY: clean
clean:
	rm -f libMagicMotion.so

