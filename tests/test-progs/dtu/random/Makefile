.PHONY: all clean

CXXFLAGS := -std=c++11 -Wall --static -Os -I ~/gem5/util/m5 ~/gem5/util/m5/m5op_x86.S

all: pe0 pe1 pe2 pe3 pe4 pe5 pe6 pe7

pe%: main.cpp
	$(CXX) $(CXXFLAGS) -DPE_ID=$* $< -o $@

clean:
	rm -f pe*
