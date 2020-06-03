SHELL:=/bin/bash
CC=clang

ifeq ($(shell uname),Darwin)
	OS=macOS
else
	OS=Linux
endif

CFLAGS=-g $(shell sdl2-config --cflags) -I src -I imgui -I ImGuizmo -I miniz -std=c++11 $(shell pkg-config opencv4 --cflags)
LIBS=-lSDL2 -L$(OPENNI2_REDIST) -lOpenNI2 -lMagicMotion -lm -lpthread $(shell pkg-config opencv4 --libs)

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

EXE=magicmotion_test

${EXE}: ${SRC} ${HEADERS} ${MAGICMOTION}
	cp -R ${OPENNI2_REDIST}/OpenNI2 ./
	cp ${OPENNI2_REDIST}/libOpenNI2.* ./
	${CC} ${CFLAGS} launchpad/main.cpp -o $@ ${LIBS}

${MAGICMOTION}: $(shell find src -type f) ${MAGICMOTION_PATH}/${MAGICMOTION}
ifeq (${OS},macOS)
	pushd macOS && xcodebuild && popd
else
	pushd linux && make -B && popd
endif
	cp ${MAGICMOTION_PATH}/${MAGICMOTION} ./

.PHONY: clean
clean:
	rm -f ${EXE}
	rm -f ${MAGICMOTION}
	rm -rf *.dSYM
	rm -rf OpenNI2
	rm -f libOpenNI2*
	rm -f boxes.ser
	rm -f sensors.ser

