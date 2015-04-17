CIBLE = main
SRCS =  Main.cpp Camera.cpp Mesh.cpp Utils.cpp BSHNode.cpp Ray.cpp BoundingMesh
LIBS =  -lglut -lGLU -lGL -lm

CC = g++
CPP = g++

FLAGS = -Wall -O2 -std=c++11

CFLAGS = $(FLAGS)
CXXFLAGS = $(FLAGS)

OBJS = $(SRCS:.cpp=.o)

$(CIBLE): $(OBJS)
	g++ $(LDFLAGS) -o $(CIBLE) $(OBJS) $(LIBS)
clean:
	rm -f  *~  $(CIBLE) $(OBJS)

Camera.o: Camera.cpp Camera.h Vec3.h
Mesh.o: Mesh.cpp Mesh.h Vec3.h
Main.o: Main.cpp Vec3.h Camera.h Mesh.h Utils.h
Utils.o: Utils.cpp Utils.h Mesh.h
BSHNode.o : BSHNode.cpp BSHNode.h Mesh.h Vec3.h Utils.h Camera.h
Ray.o : Ray.cpp Ray.h Mesh.h Vec3.h
BoundingMesh.o : BoundingMesh.cpp BoundingMesh.h Mesh.h Vec3.h


