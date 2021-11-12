SHELL:=/bin/bash
CC=clang

ifeq ($(shell uname),Darwin)
	OS=macOS
else
	OS=Linux
endif

HAS_OPENCV=false
HAS_OPENNI=false

SCENE=SCENE_VIEWER # For viewing real time data from cameras, or recordings
#SCENE=SCENE_INSPECTOR # For stepping through cloud recordings, and manually correcting them

CFLAGS=-O2 $(shell sdl2-config --cflags) -pthreads -I src -I imgui -I ImGuizmo -I miniz -std=c++11 -D${SCENE}
LIBS=-lSDL2 -lMagicMotion -lm

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

ifeq (${OS},macOS)
	LIBS += -lc++ -framework OpenGl -framework CoreFoundation
	MAGICMOTION=libMagicMotion.dylib
	MAGICMOTION_PATH=macOS/Build/Release
else
	LIBS += -L. -lstdc++ -Wl,-rpath,.
	MAGICMOTION=libMagicMotion.so
	MAGICMOTION_PATH=linux
endif

SRC=$(shell find launchpad -name '*.cpp')
HEADERS=$(shell find launchpad -name '*.h')


magicmotion_test: ${SRC} ${HEADERS} ${MAGICMOTION}
	cp -R ${OPENNI2_REDIST}/OpenNI2 ./
	cp ${OPENNI2_REDIST}/libOpenNI2.* ./
	${CC} ${CFLAGS} launchpad/main.cpp -o $@ ${LIBS}


magicmotion_server: server/server.cpp ${MAGICMOTION}
	cp -R ${OPENNI2_REDIST}/OpenNI2 ./
	cp ${OPENNI2_REDIST}/libOpenNI2.* ./
	${CC} ${CFLAGS} server/server.cpp -o $@ ${LIBS}


${MAGICMOTION_PATH}/${MAGICMOTION}: $(shell find src -type f)
ifeq (${OS},macOS)
	pushd macOS && make && popd
else
	pushd linux && make -B && popd
endif

${MAGICMOTION}: ${MAGICMOTION_PATH}/${MAGICMOTION}
	cp ${MAGICMOTION_PATH}/${MAGICMOTION} ./

.PHONY: clean
clean:
	rm -f magicmotion_test
	rm -f magicmotion_server
	rm -f ${MAGICMOTION}
	rm -rf *.dSYM
	rm -rf OpenNI2
	rm -f libOpenNI2*
	rm -f boxes.ser
	rm -f sensors.ser

