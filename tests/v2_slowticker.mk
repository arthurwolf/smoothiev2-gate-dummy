CXX ?= g++
V2 ?= ../smoothieware-microros/src/SmoothieV2/Firmware
all:
	mkdir -p output
	$(CXX) -std=gnu++14 -O1 -fpermissive -Isim/v2/rtos -Isim/v2 -I$(V2)/src/libs tests/v2_slowticker_test.cpp sim/v2/v2_step_timer.cpp sim/v2/v2_scheduler.cpp sim/v2/tmr_setup.cpp sim/v2/v2_freertos_timers.cpp $(V2)/src/libs/SlowTicker.cpp -o output/v2-slowticker-test
	output/v2-slowticker-test
