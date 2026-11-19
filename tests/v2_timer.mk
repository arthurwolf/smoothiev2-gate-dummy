CXX ?= g++
all:
	mkdir -p output
	$(CXX) -std=c++14 -Wall -Wextra -Werror -Isim/v2 tests/v2_timer_test.cpp sim/v2/v2_step_timer.cpp sim/v2/v2_scheduler.cpp sim/v2/tmr_setup.cpp -o output/v2-timer-test
	output/v2-timer-test
