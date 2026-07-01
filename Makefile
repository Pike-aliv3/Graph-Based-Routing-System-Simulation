CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2 -static

all: phase1 phase2 phase3

phase1: Phase-1/main.cpp Phase-1/QueryProcessor.cpp Common/Graph.cpp
	$(CXX) $(CXXFLAGS) -ICommon -IPhase-1 Phase-1/main.cpp Phase-1/QueryProcessor.cpp Common/Graph.cpp -o phase1

phase2: Phase-2/main.cpp Phase-2/QueryProcessor.cpp Common/Graph.cpp
	$(CXX) $(CXXFLAGS) -ICommon -IPhase-2 Phase-2/main.cpp Phase-2/QueryProcessor.cpp Common/Graph.cpp -o phase2

phase3: Phase-3/main.cpp
	$(CXX) $(CXXFLAGS) -ICommon Phase-3/main.cpp -o phase3

clean:
	rm -f phase1 phase2 phase3