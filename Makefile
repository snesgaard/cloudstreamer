CC = g++
INCLUDE =
LINK =
CFLAGS = -c -std=c++11 -Wall -O2 -fpic -g
LFLAGS =
LIBS = -lzmq

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
