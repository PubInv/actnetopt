INC_DIR := headers
SRC_DIR := src
OBJ_DIR := object
LOCAL_LIB := /usr/local/lib

CC       := g++
# DEBUG_AND_PROFILE := -pg -g
# CPPFLAGS := -I$(INC_DIR) -MMD -MP $(DEBUG_AND_PROFILE)
CPPFLAGS := -I$(INC_DIR) $(DEBUG_AND_PROFILE)
CXXFLAGS := -std=c++11 -Wall -I/usr/local/include/SDL2 -I/usr/X11R6/include -D_THREAD_SAFE -DDEBUG -DDLIB_NO_GUI_SUPPORT -I/Users/robertread/PubInv/dlib
LDLIBS   := -L/usr/local/lib -lSDL2 $(DEBUG_AND_PROFILE) 



SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
DEPS    := $(wildcard $(OBJ_DIR)/*.d)

.PHONY: clean

all: playground test

playground: $(OBJ_DIR)/playground.o $(OBJ_DIR)/TriLadder.o 
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

test: $(OBJ_DIR)/derivative.o $(OBJ_DIR)/TriLadder.o 
	$(CC) $(CXXFLAGS) $(LDFLAGS) $(OBJ_DIR)/derivative.o $(OBJ_DIR)/TriLadder.o $(LDLIBS) $(LOCAL_LIB)/libboost_unit_test_framework.a -o $@

$(OBJ_DIR)/playground.o: $(SRC_DIR)/playground.cpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS) -c $(SRC_DIR)/playground.cpp -o $(OBJ_DIR)/playground.o

$(OBJ_DIR)/derivative.o: $(SRC_DIR)/derivative.cpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/derivative.cpp $(OUTPUT_OPTION) 

$(OBJ_DIR)/TriLadder.o: $(SRC_DIR)/TriLadder.cpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/TriLadder.cpp -o $(OBJ_DIR)/TriLadder.o

clean: ; $(RM) $(DEPS) $(OBJECTS)

include $(DEPS)

$(MAKEFILE_LIST): ;
%:: %,v
%:: RCS/%,v
%:: RCS/%
%:: s.%
%:: SCCS/s.%
