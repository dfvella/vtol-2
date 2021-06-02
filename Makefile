CXX = arduino
BOARD = arduino:avr:nano
PORT = /dev/ttyUSB0

# if USB port is found, then set flags so the board is flashed
FLAGS = --board $(BOARD)
FOUND = $(shell find /dev -wholename $(PORT))
FLAGS += $(if $(FOUND),--port $(PORT) --upload,--verify)


.PHONY: flight_controller
flight_controller:
	$(CXX) $(FLAGS) $@/$@.ino


TESTS = imu_calibration
TESTS += $(shell find . -name "*.ino" | grep "test" | cut -d'/' -f3 | sed 's/.ino//')

# note this will not resolve relative includes in header files
define get_depends =
	$(shell cat $@/$@.ino | grep "#include \"" | cut -d' ' -f2 | sed "s/\"//g")
	$(shell cat $@/$@.ino | grep "#include \"" | cut -d' ' -f2 | sed "s/\"//g;s/\.h/\.cpp/")
endef

# copy test dependencies into the sketch directory before build
.PHONY: $(TESTS)
$(TESTS):
	$(foreach file, $(get_depends), $(shell cp flight_controller/$(file) $@/))
	$(CXX) $(FLAGS) $@/$@.ino
