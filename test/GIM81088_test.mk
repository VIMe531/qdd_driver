# Makefile

# compiler flags
CXX       := gcc
CXXFLAGS  := -std=c++11 -Wall -O2 \
			-I../GIM81088

# target name
TARGET    := gim81088_test

# source files and objects
SRCS      := GIM81088_test.cpp \
			../GIM81088/GIM81088_Driver.cpp
OBJS      := $(SRCS:.cpp=.o)

.PHONY: all clean

# default
all: $(TARGET)

# link
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lstdc++ -lm

# create objects
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# remove files
clean:
	rm -f $(TARGET) $(OBJS)

