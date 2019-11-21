CFLAGS=-std=c++11 -g -O3 -Ofast
# -O3 -Ofast

visual:
	g++ $(CFLAGS)   -o giscup dsengine.cpp -ldslab -lshp `wx-config --cflags --libs --gl-libs` -l GL -lGLU

benchmark: benchmark.cpp
	g++  -o benchmark benchmark.cpp $(CFLAGS) -lshp 

docs:
	doxygen doxy.cfg
