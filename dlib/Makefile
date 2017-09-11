INC_DIR := headers
SRC_DIR := src
OBJ_DIR := object

CC       := g++
CPPFLAGS := -I$(INC_DIR) -MMD -MP
CXXFLAGS := -std=c++11 -Wall -I/usr/local/include/SDL2 -I/usr/X11R6/include -D_THREAD_SAFE -DDEBUG -DDLIB_NO_GUI_SUPPORT -I/Users/robertread/PubInv/dlib
LDLIBS   := -L/usr/local/lib -lSDL2



SOURCES := $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS := $(SOURCES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
DEPS    := $(wildcard $(OBJ_DIR)/*.d)

.PHONY: clean

main: $(OBJECTS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(OBJECTS): $(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $(OUTPUT_OPTION) $<

clean: ; $(RM) $(DEPS) $(OBJECTS)

include $(DEPS)

$(MAKEFILE_LIST): ;
%:: %,v
%:: RCS/%,v
%:: RCS/%
%:: s.%
%:: SCCS/s.%
