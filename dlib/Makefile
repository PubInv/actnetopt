INC_DIR := headers
SRC_DIR := src
OBJ_DIR := object
LOCAL_LIB := /usr/local/lib

CC       := g++
# DEBUG_AND_PROFILE := -pg -g
# CPPFLAGS := -I$(INC_DIR) -MMD -MP $(DEBUG_AND_PROFILE)
CPPFLAGS := -I$(INC_DIR) $(DEBUG_AND_PROFILE)
# CXXFLAGS := -std=c++11 -Wall -I/usr/local/include/SDL2 -I/usr/X11R6/include -D_THREAD_SAFE -DDEBUG -DDLIB_NO_GUI_SUPPORT -I/Users/robertread/PubInv/dlib -I/Users/robertread/PubInv/restbed/source
CXXFLAGS := -std=c++11 -Wall -I/usr/local/include/SDL2 -I/usr/X11R6/include -D_THREAD_SAFE -DDLIB_NO_GUI_SUPPORT -I/Users/robertread/PubInv/dlib -I/Users/robertread/PubInv/restbed/source
LDLIBS   := -L/usr/local/lib -lSDL2 $(DEBUG_AND_PROFILE) -lrestbed

SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
DEPS    := $(wildcard $(OBJ_DIR)/*.d)

.PHONY: clean

all: playground_runner test http_service http_service3d http_service_posthole test3d

playground_runner: $(OBJ_DIR)/playground.o $(OBJ_DIR)/TriLadder.o $(OBJ_DIR)/Invert.o $(OBJ_DIR)/Obstacle.o $(OBJ_DIR)/playground_runner.o $(OBJ_DIR)/ActNetUtility.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

http_service: $(OBJ_DIR)/http_service.o $(OBJ_DIR)/playground.o $(OBJ_DIR)/TriLadder.o $(OBJ_DIR)/Invert.o $(OBJ_DIR)/Obstacle.o $(OBJ_DIR)/ActNetUtility.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

http_service3d: $(OBJ_DIR)/http_service3d.o $(OBJ_DIR)/playground3d.o $(OBJ_DIR)/Tetrahelix.o $(OBJ_DIR)/Invert3d.o $(OBJ_DIR)/Obstacle.o $(OBJ_DIR)/ActNetUtility.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

http_service_posthole: $(OBJ_DIR)/http_service_posthole.o $(OBJ_DIR)/ActNetUtility.o $(OBJ_DIR)/posthole.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

test: $(OBJ_DIR)/derivative.o $(OBJ_DIR)/TriLadder.o $(OBJ_DIR)/Invert.o $(OBJ_DIR)/Obstacle.o
	$(CC) $(CXXFLAGS) $(LDFLAGS) $(OBJ_DIR)/derivative.o $(OBJ_DIR)/TriLadder.o $(OBJ_DIR)/Obstacle.o $(OBJ_DIR)/Invert.o $(OBJ_DIR)/ActNetUtility.o $(LDLIBS) $(LOCAL_LIB)/libboost_unit_test_framework.a -o $@

test3d: $(OBJ_DIR)/derivative3d.o $(OBJ_DIR)/Tetrahelix.o $(OBJ_DIR)/ActNetUtility.o $(OBJ_DIR)/Invert3d.o $(OBJ_DIR)/Obstacle.o $(OBJ_DIR)/playground3d.o
	$(CC) $(CXXFLAGS) $(LDFLAGS) $(OBJ_DIR)/Tetrahelix.o $(OBJ_DIR)/derivative3d.o $(OBJ_DIR)/Invert3d.o $(OBJ_DIR)/ActNetUtility.o  $(OBJ_DIR)/Obstacle.o $(OBJ_DIR)/playground3d.o  $(LDLIBS) $(LOCAL_LIB)/libboost_unit_test_framework.a -o $@

$(OBJ_DIR)/ActNetUtility.o: $(SRC_DIR)/ActnetUtility.cpp $(SRC_DIR)/ActnetUtility.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS) -c $(SRC_DIR)/ActnetUtility.cpp -o $(OBJ_DIR)/ActnetUtility.o

$(OBJ_DIR)/playground.o: $(SRC_DIR)/playground.cpp $(SRC_DIR)/playground.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS) -c $(SRC_DIR)/playground.cpp -o $(OBJ_DIR)/playground.o

$(OBJ_DIR)/playground3d.o: $(SRC_DIR)/playground3d.cpp $(SRC_DIR)/playground3d.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS) -c $(SRC_DIR)/playground3d.cpp -o $(OBJ_DIR)/playground3d.o

$(OBJ_DIR)/derivative.o: $(SRC_DIR)/derivative.cpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/derivative.cpp $(OUTPUT_OPTION)

$(OBJ_DIR)/derivative3d.o: $(SRC_DIR)/derivative3d.cpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/derivative3d.cpp $(OUTPUT_OPTION)

$(OBJ_DIR)/TriLadder.o: $(SRC_DIR)/TriLadder.cpp $(SRC_DIR)/TriLadder.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/TriLadder.cpp -o $(OBJ_DIR)/TriLadder.o

$(OBJ_DIR)/Tetrahelix.o: $(SRC_DIR)/Tetrahelix.cpp $(SRC_DIR)/Tetrahelix.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/Tetrahelix.cpp -o $(OBJ_DIR)/Tetrahelix.o

$(OBJ_DIR)/Invert.o: $(SRC_DIR)/Invert.cpp $(SRC_DIR)/Invert.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/Invert.cpp -o $(OBJ_DIR)/Invert.o

$(OBJ_DIR)/Invert3d.o: $(SRC_DIR)/Invert3d.cpp $(SRC_DIR)/Invert3d.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/Invert3d.cpp -o $(OBJ_DIR)/Invert3d.o

$(OBJ_DIR)/posthole.o: $(SRC_DIR)/posthole.cpp $(SRC_DIR)/posthole.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/posthole.cpp -o $(OBJ_DIR)/posthole.o

$(OBJ_DIR)/Obstacle.o: $(SRC_DIR)/Obstacle.cpp $(SRC_DIR)/Obstacle.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/Obstacle.cpp -o $(OBJ_DIR)/Obstacle.o

$(OBJ_DIR)/http_service.o: $(SRC_DIR)/http_service.cpp $(SRC_DIR)/custom_logger.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/http_service.cpp -o $(OBJ_DIR)/http_service.o

$(OBJ_DIR)/http_service3d.o: $(SRC_DIR)/http_service3d.cpp $(SRC_DIR)/custom_logger.hpp $(SRC_DIR)/playground3d.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/http_service3d.cpp -o $(OBJ_DIR)/http_service3d.o

$(OBJ_DIR)/http_service_posthole.o: $(SRC_DIR)/http_service_posthole.cpp $(SRC_DIR)/custom_logger.hpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/http_service_posthole.cpp -o $(OBJ_DIR)/http_service_posthole.o

$(OBJ_DIR)/playground_runner.o: $(SRC_DIR)/playground_runner.cpp
	$(CC) $(CXXFLAGS) $(CPPFLAGS)  -c $(SRC_DIR)/playground_runner.cpp -o $(OBJ_DIR)/playground_runner.o


clean: ; $(RM) $(DEPS) $(OBJECTS)

include $(DEPS)

$(MAKEFILE_LIST): ;
%:: %,v
%:: RCS/%,v
%:: RCS/%
%:: s.%
%:: SCCS/s.%
