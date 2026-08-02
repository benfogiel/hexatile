# Host build of the topology regression test. `make` builds and runs it.
#
# -I. picks up the Arduino.h / avr/pgmspace.h stubs before any real ones;
# -I.. picks up the firmware headers. sim.cpp includes ../topology.cpp itself.

CXXFLAGS = -std=c++14 -O1 -Wall -Wextra -Wno-unused-parameter -I. -I..

hexsim: sim.cpp Arduino.h avr/pgmspace.h \
        ../topology.cpp ../topology.h ../config.h ../comms.h ../led_layout.h
	$(CXX) $(CXXFLAGS) sim.cpp -o $@

.PHONY: run clean
run: hexsim
	./hexsim

clean:
	rm -f hexsim
