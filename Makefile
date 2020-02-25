CC=clang
ifeq ($(shell uname),Darwin)
	OS=macOS
else
	OS=Linux
endif

CFLAGS=-g $(shell sdl2-config --cflags) -I src -I imgui -I ImGuizmo
LIBS=-lSDL2 -L$(OPENNI2_REDIST) -lOpenNI2 -lMagicMotion -lm

ifeq (${OS},macOS)
	LIBS += -lc++ -framework OpenGl -framework CoreFoundation
	MAGICMOTION=libMagicMotion.dylib
	MAGICMOTION_PATH=macOS/Build/Debug
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

${MAGICMOTION}: ${MAGICMOTION_PATH}/${MAGICMOTION}
	cp ${MAGICMOTION_PATH}/${MAGICMOTION} ./

.PHONY: clean
clean:
	rm -f ${EXE}
	rm -f ${MAGICMOTION}

