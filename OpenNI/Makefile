CC = clang
CFLAGS = -std=c++11 -I${OPENNI2_INCLUDE} $(shell sdl2-config --cflags) -Wall -Wextra
LIBS = -lstdc++ -lm $(shell sdl2-config --libs) -L${OPENNI2_REDIST} -lOpenNI2

SOURCES = src/istari.cpp src/graphics.cpp src/memory.cpp src/ikea.cpp src/gl.cpp src/sensor_interface.cpp

PLATFORM = $(shell uname) # Will be Darwin for macOS
EXE = IKEAMeatballs

.PHONY: host
host:
	${CC} src/host_posix.cpp -g -std=c++11 -ldl -o ${EXE}

.PHONY: gamecode
gamecode:
	rm -rf gamecode.so bin
	${CC} ${SOURCES} -g ${CFLAGS} -shared -fPIC -undefined dynamic_lookup ${LIBS} -o gamecode.so
	install_name_tool -change libOpenNI2.dylib ${OPENNI2_REDIST}/libOpenNI2.dylib gamecode.so

.PHONY: release
release:
	${CC} ${SOURCES} -DRELEASE -O3 ${CFLAGS} ${LIBS} -o ${EXE}

.PHONY: clean
clean:
	rm -rf bin ${EXE} gamecode.so gamecode.so.dSYM ${EXE}.dSYM

