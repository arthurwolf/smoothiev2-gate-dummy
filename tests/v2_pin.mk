CXX ?= g++
all:
	mkdir -p output
	$(CXX) -std=c++14 -Wall -Wextra -Werror -Isim/v2/hal tests/v2_pin_test.cpp -o output/v2-pin-test
	output/v2-pin-test
