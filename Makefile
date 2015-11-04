CFLAGS=-I../boost_1_58_0  -std=c++11 -O3 -Ofast

visual:
	g++ $(CFLAGS)   -o giscup dsengine.cpp -ldslab -lshp `wx-config --cflags --libs --gl-libs`

benchmark: benchmark.cpp
	g++  -o benchmark benchmark.cpp $(CFLAGS) -lshp 

docs:
	doxygen doxy.cfg
