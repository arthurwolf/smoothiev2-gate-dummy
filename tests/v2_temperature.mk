CXX ?= g++
V2 ?= ../smoothieware-microros/src/SmoothieV2/Firmware
I := -Isim -Isim/v2/rtos -Isim/v2/hal -Isim/v2 -I$(V2)/src -I$(V2)/src/libs -I$(V2)/src/robot -I$(V2)/src/robot/arm_solutions -I$(V2)/src/modules/tools/temperaturecontrol -I$(V2)/src/modules/tools/temperatureswitch -I$(V2)/src/modules/tools/switch -I$(V2)/src/modules/tools/laser -I$(V2)/Hal/src
CXXFLAGS := -std=gnu++14 -O1 -fpermissive -ffunction-sections -fdata-sections -Dinfinityf=__builtin_inff -w $(I)
CORE := $(addprefix $(V2)/src/,ConfigReader.cpp Module.cpp Dispatcher.cpp GCode.cpp GCodeProcessor.cpp libs/StringUtils.cpp libs/OutputStream.cpp libs/xformatc.c libs/nist_float.cpp libs/Vector3.cpp libs/FastTicker.cpp libs/SlowTicker.cpp libs/SigmaDeltaPwm.cpp robot/Block.cpp robot/Planner.cpp robot/Conveyor.cpp robot/StepTicker.cpp robot/StepperMotor.cpp robot/Robot.cpp)
SOLUTIONS := $(addprefix $(V2)/src/robot/arm_solutions/,CartesianSolution.cpp CoreXZSolution.cpp HBotSolution.cpp LinearDeltaSolution.cpp MorganSCARASolution.cpp RotaryDeltaSolution.cpp)
TEMP := $(addprefix $(V2)/src/modules/tools/temperaturecontrol/,TemperatureControl.cpp Thermistor.cpp max31855.cpp PID_Autotuner.cpp) $(V2)/src/modules/tools/switch/Switch.cpp $(V2)/src/modules/tools/temperatureswitch/TemperatureSwitch.cpp $(V2)/src/modules/tools/laser/Laser.cpp
all:
	mkdir -p output
	$(CXX) $(CXXFLAGS) tests/v2_temperature_test.cpp sim/v2/v2_step_timer.cpp sim/v2/v2_scheduler.cpp sim/v2/tmr_setup.cpp sim/v2/v2_freertos_timers.cpp $(CORE) $(SOLUTIONS) $(TEMP) -Wl,--gc-sections -o output/v2-temperature-test
	output/v2-temperature-test
