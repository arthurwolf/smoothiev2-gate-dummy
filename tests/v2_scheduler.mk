CXX ?= g++
all:
	mkdir -p output
	$(CXX) -std=gnu++14 -O1 -Isim/v2 tests/v2_scheduler_test.cpp sim/v2/v2_step_timer.cpp sim/v2/v2_scheduler.cpp -o output/v2-scheduler-test
	output/v2-scheduler-test
