# Makefile

# compiler flags
CXX       := gcc
CXXFLAGS  := -std=c++11 -Wall -O2 \
			-I./ \
			-I../Cybergear

# target name
TARGET    := cybergear_test

# source files and objects
SRCS      := Cybergear_test.cpp \
			../Cybergear/Cybergear_Driver.cpp
OBJS      := $(SRCS:.cpp=.o)

.PHONY: all clean

# default
all: $(TARGET)

# link
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lstdc++

# create objects
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# remove files
clean:
	rm -f $(TARGET) $(OBJS)

