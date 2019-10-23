CC=clang

CFLAGS=-g $(shell sdl2-config --cflags) -I src -I imgui -I OpenNI/Include
LIBS=$(shell sdl2-config --libs) -framework OpenGl -framework CoreFoundation -lc++ -L OpenNI/Redist -lOpenNI2

SRC=$(shell find src -name '*.cpp')
HEADERS=$(shell find src -name '*.h')

EXE=code

${EXE}: ${SRC} ${HEADERS}
	${CC} ${CFLAGS} src/main.cpp -o $@ ${LIBS}

.PHONY: clean
clean:
	rm -f ${EXE}

.PHONY: test
test:
	${CC} ${CFLAGS} test.cpp -o test ${LIBS}
	./test
	@rm -f test

