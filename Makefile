all: orange

HOMEDIR=/home/nick/detectTriangle/
CFLAGS=-fPIC -g -Wall `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` -lGLEW -lglut -lGL -lGLU
INCLUDE = -I/usr/local/include/boost/ -I/usr/local/include/libfreenect 
LOCAL_INCLUDE = -I$(HOMEDIR) -I$(HOMEDIR)/include
FREE_LIBS = -L/usr/local/lib -lfreenect

orange:  *.cpp src/*.cpp
	$(CXX) $(INCLUDE) $(LOCAL_INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o orange

