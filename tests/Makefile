CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -O2 -I../include
SUITES   := ac_auto_fan_test ac_control_test ac_modbus_test
# Build in-tree (gitignored), not /tmp: a fixed path in a world-writable directory
# collides between parallel runs and users.
BUILD    := build

.PHONY: test clean $(SUITES)
test: $(SUITES)

$(SUITES): | $(BUILD)
	$(CXX) $(CXXFLAGS) $@.cpp -o $(BUILD)/$@ && $(BUILD)/$@ && echo "$@: PASS"

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
