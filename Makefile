CC=clang
ifeq ($(shell uname),Darwin)
	OS=macOS
else
	OS=Linux
endif

OPENNI2_REDIST=/Users/istarnion/Google\ Drive/School/ntnu_spring2019/graphics/project/lib/OpenNI/Redist

CFLAGS=-g $(shell sdl2-config --cflags) -I src -I imgui
LIBS=-lSDL2 -L$(OPENNI2_REDIST) -lOpenNI2 -lMagicMotion -lc++

ifeq (${OS},macOS)
	LIBS += -framework OpenGl -framework CoreFoundation
endif

SRC=$(shell find launchpad -name '*.cpp')
HEADERS=$(shell find launchpad -name '*.h')

EXE=magicmotion_test

${EXE}: ${SRC} ${HEADERS} macOS/Build/Debug/libMagicMotion.dylib
	cp macOS/Build/Debug/libMagicMotion.dylib ./
	${CC} ${CFLAGS} launchpad/main.cpp -o $@ ${LIBS}

.PHONY: clean
clean:
	rm -f ${EXE}
	rm -rf macOS/Builds

