CXX ?= g++
V2 := ../smoothieware-microros/src/SmoothieV2/Firmware
INCLUDES := -Itests/v2_support -Isim/v2/hal -Isim/v2 -I$(V2)/src/robot -I$(V2)/src -I$(V2)/src/libs -I$(V2)/Hal/src
CXXFLAGS := -std=gnu++14 -O1 -g -ffunction-sections -fdata-sections -w $(INCLUDES)
all:
	mkdir -p output
	$(CXX) $(CXXFLAGS) tests/v2_planner_test.cpp sim/v2/v2_step_timer.cpp sim/v2/v2_scheduler.cpp sim/v2/tmr_setup.cpp $(V2)/src/ConfigReader.cpp $(V2)/src/libs/StringUtils.cpp $(V2)/src/robot/Block.cpp $(V2)/src/robot/Planner.cpp $(V2)/src/robot/Conveyor.cpp $(V2)/src/robot/StepTicker.cpp $(V2)/src/robot/StepperMotor.cpp -Wl,--gc-sections -o output/v2-planner-test
	output/v2-planner-test
