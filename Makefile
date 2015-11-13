CC = g++
INCLUDE = -I./lib/
LINK = -L./lib/
CFLAGS = -c -std=c++11 -Wall -O2 -fpic -g `pkg-config --cflags opencv`
LFLAGS =
LIBS = -ldc1394 `pkg-config --libs opencv` -lpthread -lzmq

BUILDDIR = ./build/
BINDIR = ./
SRCDIR = ./

TARGET=pgcameradaemon

_SRC = main.cpp network.cpp camera.cpp
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
