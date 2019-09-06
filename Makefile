CC=clang

CFLAGS=-g $(shell sdl2-config --cflags) -I src -I imgui
LIBS=$(shell sdl2-config --libs) -framework OpenGl -framework CoreFoundation -lc++

SRC=$(shell find src -name *.cpp)
HEADERS=$(shell find src -name *.h)

EXE=code

${EXE}: ${SRC} ${HEADERS}
	${CC} ${CFLAGS} src/main.cpp -o $@ ${LIBS}

.PHONY: clean
clean:
	rm -f ${EXE}

