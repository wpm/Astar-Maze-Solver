TARGET = grid
OBJECTS = main.o
CXXFLAGS = -g -I/src/boost-trunk

$(TARGET): $(OBJECTS)
	g++ $(OBJECTS) -o $@

.PHONY : clean
clean:
	rm -rf $(TARGET) $(OBJECTS)
