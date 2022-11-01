LIB=-lm -lraylib
CFLAGS= -O3
EXENAME=springs.out

all: exe

debug: CFLAGS = -g -Wall # Change the flags for debug mode
debug: clean
debug: exe

exe: main.o vec3.o system.o spring.o spring_list.o
	g++ $(CFLAGS) main.o vec3.o system.o spring.o spring_list.o -o $(EXENAME) $(LIB)

main.o: main.cpp
	g++ $(CFLAGS) -c main.cpp

vec3.o: vec3.cpp
	g++ $(CFLAGS) -c vec3.cpp

system.o: system.cpp
	g++ $(CFLAGS) -c system.cpp

spring.o: spring.cpp
	g++ $(CFLAGS) -c spring.cpp

spring_list.o: spring_list.cpp
	g++ $(CFLAGS) -c spring_list.cpp

run: exe
	./$(EXENAME)

clean:
	rm *.out *.o
