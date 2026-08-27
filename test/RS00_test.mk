# Makefile

# compiler flags
CXX       := gcc
CXXFLAGS  := -std=c++11 -Wall -O2 \
			-I./ \
			-I../ROBSTRIDE

# target name
TARGET    := RS00_test

# source files and objects
SRCS      := RS00_test.cpp \
			../ROBSTRIDE/RS00_Driver.cpp
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

