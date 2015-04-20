CIBLE = main
SRCS =  Main.cpp Camera.cpp Mesh.cpp Ray.cpp BoundingMesh.cpp Utils.cpp Interface.cpp
LIBS =  -lglut -lGLU -lGL -lm

CC = g++
CPP = g++

FLAGS = -Wall -O0 -std=c++11 -g

CFLAGS = $(FLAGS)
CXXFLAGS = $(FLAGS)

OBJS = $(SRCS:.cpp=.o)

$(CIBLE): $(OBJS)
	g++ $(LDFLAGS) -o $(CIBLE) $(OBJS) $(LIBS)
clean:
	rm -f  *~  $(CIBLE) $(OBJS)

Camera.o: Camera.cpp Camera.h Vec3.h
Mesh.o: Mesh.cpp Mesh.h Vec3.h
Main.o: Main.cpp Vec3.h Camera.h Mesh.h Utils.h BoundingMesh.h
Utils.o: Utils.cpp Utils.h Mesh.h
BSHNode.o : BSHNode.cpp BSHNode.h Mesh.h Vec3.h Utils.h Camera.h
Ray.o : Ray.cpp Ray.h Mesh.h Vec3.h
BoundingMesh.o : BoundingMesh.cpp BoundingMesh.h Mesh.h Vec3.h
Interface.o : Interface.cpp Interface.h Utils.h


