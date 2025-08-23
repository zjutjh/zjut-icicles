CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

TARGET = mst_experiment
SRC = mst_experiment.cpp

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f $(TARGET) *.o 