CXX ?= g++
V2 ?= ../smoothieware-microros/src/SmoothieV2/Firmware
all:
	mkdir -p output
	$(CXX) -std=gnu++14 -O1 -g -ffunction-sections -fdata-sections -w -Isim -Isim/v2 -Isim/v2/hal -I$(V2)/src -I$(V2)/src/robot tests/v2_capture_test.cpp sim/v2/sim_v2_capture.cpp $(V2)/src/robot/Block.cpp -Wl,--gc-sections -o output/v2-capture-test
	output/v2-capture-test
