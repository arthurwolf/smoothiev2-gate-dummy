CXX ?= g++
all:
	mkdir -p output
	$(CXX) -std=gnu++14 -O1 -Isim/v2/hal tests/v2_pwm_test.cpp -o output/v2-pwm-test
	output/v2-pwm-test
