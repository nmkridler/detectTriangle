all: orange

CFLAGS=-fPIC -g -Wall `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` -lGLEW -lglut -lGL
INCLUDE = -I/usr/local/include/boost/ -I/usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect

orange:  *.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o orange

