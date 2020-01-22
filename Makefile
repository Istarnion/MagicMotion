CC=clang

CFLAGS=-g $(shell sdl2-config --cflags) -I src -I imgui -I OpenNI/Include
LIBS=$(shell sdl2-config --libs) -framework OpenGl -framework CoreFoundation -lc++ -L OpenNI/Redist -lOpenNI2

SRC=$(shell find src -name '*.cpp')
HEADERS=$(shell find src -name '*.h')

EXE=launchpad
SHARED_LIB=libMagicMotion.so

${EXE}: ${SRC} ${HEADERS} ${SHARED_LIB}
	${CC} ${CFLAGS} src/main.cpp -o $@ ${LIBS}

${SHARED_LIB}:
	${CC} ${CFLAGS}

.PHONY: clean
clean:
	rm -f ${EXE}
	rm -f ${SHARED_LIB}

.PHONY: test
test:
	${CC} ${CFLAGS} test.cpp -o test ${LIBS}
	./test
	@rm -f test

