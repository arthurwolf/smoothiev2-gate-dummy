CXX ?= g++
SOURCE ?= ../smoothieware-microros/src/SmoothieV2/Firmware
V2 := output/firmware
INCLUDES := -Isim -Itests/v2_support -Isim/v2/hal -Isim/v2 -I$(V2)/src -I$(V2)/src/libs -I$(V2)/src/robot -I$(V2)/src/robot/arm_solutions -I$(V2)/src/modules/tools/temperaturecontrol -I$(V2)/Hal/src
CXXFLAGS := -std=gnu++14 -O1 -g -ffunction-sections -fdata-sections -fpermissive -w $(INCLUDES)
CORE := $(addprefix $(V2)/src/,ConfigReader.cpp Module.cpp Dispatcher.cpp GCode.cpp GCodeProcessor.cpp libs/StringUtils.cpp libs/Vector3.cpp libs/nist_float.cpp libs/OutputStream.cpp libs/xformatc.c robot/Block.cpp robot/Planner.cpp robot/Conveyor.cpp robot/StepTicker.cpp robot/StepperMotor.cpp robot/Robot.cpp)
SOLUTIONS := $(addprefix $(V2)/src/robot/arm_solutions/,CartesianSolution.cpp CoreXZSolution.cpp HBotSolution.cpp LinearDeltaSolution.cpp MorganSCARASolution.cpp RotaryDeltaSolution.cpp)
all:
	mkdir -p output
	python3 tools/stage_v2.py $(SOURCE) $(V2)
	python3 tools/stage_v2_patch.py $(V2)/src
	$(CXX) $(CXXFLAGS) tests/v2_robot_test.cpp sim/v2/v2_step_timer.cpp sim/v2/v2_scheduler.cpp sim/v2/tmr_setup.cpp sim/v2/sim_v2_capture.cpp sim/sim_output.cpp $(CORE) $(SOLUTIONS) -Wl,--gc-sections -o output/v2-robot-test
	timeout 10s output/v2-robot-test
	python3 -c 'import json; r=json.load(open("output/v2-result.json")); assert r["pulse_position"][:2] == [-160, 0]; assert r["final_steps"][:2] == [160, 0]; assert [b["step_counts"][0] for b in r["blocks"]] == [80, 40, 40, 80, 80]; assert [b["gcode_line"] for b in r["blocks"]] == [1, 3, 4, 5, 6]'
