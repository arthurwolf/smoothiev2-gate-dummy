CXX ?= g++
all:
	mkdir -p output
	$(CXX) -std=gnu++14 -Isim/v2 tests/v2_conveyor_flags_test.cpp -o output/v2-conveyor-flags-test
	output/v2-conveyor-flags-test
