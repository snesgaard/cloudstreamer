CC = g++
INCLUDE = -I/usr/local/include/pcl-1.7/ -I/usr/include/eigen3/ -I/usr/include/vtk-5.8/
LINK = -L/usr/local/lib/
CFLAGS = -c -std=c++11 -Wall -O3 -fpic -g
LFLAGS =
LIBS = -lzmq -lboost_thread -lboost_system -lpcl_visualization -lpcl_common \
				-lvtkCommon -lvtkFiltering -lvtkRendering

BUILDDIR = ./build/
BINDIR = ./
SRCDIR = ./

TARGET=cloudstreamer

_SRC = main.cpp util.cpp
SRC = $(_SRC:%=$(SRCDIR)%)
OBJ = $(_SRC:%.cpp=$(BUILDDIR)%.o)


all: directories $(TARGET)

directories:
	@mkdir -p $(BUILDDIR)
	@mkdir -p $(BINDIR)

$(TARGET): $(OBJ)
	$(CC) $(LINK) $(LFLAGS) $(OBJ) -o $(BINDIR)$(TARGET) $(LIBS)

$(BUILDDIR)%.o: $(SRCDIR)%.cpp
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@

clean:
	@echo "Clearing build directories..."
	@rm -rf $(BUILDDIR)
	@rm -f $(TARGET)
