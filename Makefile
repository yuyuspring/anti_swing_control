CXX := g++
CXXFLAGS := -std=gnu++17 -Wall -Wextra -O2
LDFLAGS := -lm
PYTHON ?= ./.venv/bin/python
TARGET := pend_observer_test
REPLAY_TARGET := replay_csv
SRCS := main.cpp pend_observer.cpp
REPLAY_SRCS := replay_csv.cpp pend_observer.cpp
CSV := simulation_results.csv
CSV_PITCH := simulation_results_pitch.csv
CSV_ROLL := simulation_results_roll.csv
CSV_MIXED := simulation_results_mixed.csv
PLOT := simulation_results.png
REPLAY_INPUT := crane_imu_obs_debug.csv
REPLAY_CSV := replay_validation.csv
REPLAY_PLOT := replay_validation.png

.PHONY: all run plot replay replay-plot clean

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) $(LDFLAGS) -o $(TARGET)

$(REPLAY_TARGET): $(REPLAY_SRCS)
	$(CXX) $(CXXFLAGS) $(REPLAY_SRCS) $(LDFLAGS) -o $(REPLAY_TARGET)

run: $(TARGET)
	./$(TARGET)

plot: $(TARGET) plot_results.py
	./$(TARGET)
	$(PYTHON) plot_results.py $(CSV) $(PLOT)

replay: $(REPLAY_TARGET)
	./$(REPLAY_TARGET) $(REPLAY_INPUT) $(REPLAY_CSV)

replay-plot: $(REPLAY_TARGET) plot_replay_results.py
	./$(REPLAY_TARGET) $(REPLAY_INPUT) $(REPLAY_CSV)
	$(PYTHON) plot_replay_results.py $(REPLAY_CSV) $(REPLAY_PLOT)

clean:
	rm -f $(TARGET) $(REPLAY_TARGET) $(CSV) $(CSV_PITCH) $(CSV_ROLL) $(CSV_MIXED) $(PLOT) $(REPLAY_CSV) $(REPLAY_PLOT)