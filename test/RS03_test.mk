# Makefile

# compiler flags
CXX       := gcc
CXXFLAGS  := -std=c++11 -Wall -O2 \
			-I./ \
			-I../ROBSTRIDE

# target name
TARGET    := RS03_test

# source files and objects
SRCS      := RS03_test.cpp \
			../ROBSTRIDE/RS03_Driver.cpp
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

