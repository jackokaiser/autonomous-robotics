CC=g++
CFLAGS=-Wall -g
OBJECTS=$(patsubst src/%.cpp,%.o,$(wildcard src/*.cpp))
OBJECTS_DIR=$(patsubst src/%.cpp,objects/%.o,$(wildcard src/*.cpp))
INC_PATH=include/
VPATH=include:src:objects:lib:bin
LFLAGS=-I /usr/local/include/opencv/ -L /usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_calib3d -lopencv_legacy -lopencv_imgproc -lopencv_video
# if looking for library flags, try command "${libname}-config --ldflags"
# or "pkg-config ${libname} --libs"

all : main

main : $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $(OBJECTS_DIR) $(LFLAGS)
	mv $@ bin/

%.o : %.cpp
	$(CC) $(CFLAGS) -c $< -I $(INC_PATH)
	mv $@ objects/


archive :
	tar -cvf archive.tar makefile src/ include/ data/
	tar -rvf archive.tar --no-recursion bin objects lib
	gzip archive.tar

TAGS :
	find . -type f -iname "*.h" -or -iname "*.cpp" -or -iname "*.hpp" | xargs etags -a

clean :
	rm -f objects/*.o bin/* archive.tar.gz
