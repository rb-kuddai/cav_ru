CC=g++

INCS= -I ./include
LIBS= -L ./lib -L ./ 

CFLAGS= $(INCS) -std=c++98 -Wall -O3

CPP_FILES= $(wildcard src/*.cpp)
OBJ_FILES= $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))

ifeq ($(findstring MINGW,$(shell uname)),MINGW)
	LFLAGS = $(LIBS) -lglut -lglu32 -lopengl32
endif

ifeq ($(findstring Linux,$(shell uname)),Linux)
	LFLAGS = $(LIBS) -lglut -lGLU -lGL
endif

skinning: $(OBJ_FILES) Skinning.cpp
	$(CC) Skinning.cpp $(OBJ_FILES) $(CFLAGS) $(LFLAGS) -o skinning

obj/%.o: src/%.cpp | obj
	$(CC) $< -c $(CFLAGS) -o $@

obj:
	mkdir obj

clean:
	rm $(OBJ_FILES)
