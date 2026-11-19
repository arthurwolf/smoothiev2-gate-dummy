CXX ?= g++
all:
	mkdir -p output
	$(CXX) -std=gnu++14 -O1 -DSIM_V2_COOPERATIVE_TASKS -Isim/v2/rtos -Isim/v2 -Isim/v2/hal tests/v2_mutex_test.cpp sim/v2/v2_task.cpp sim/v2/v2_scheduler.cpp sim/v2/v2_step_timer.cpp -o output/v2-mutex-test
	output/v2-mutex-test
