.PHONY: all clean

CXXFLAGS := -std=c++11 --static -Os -I ~/gem5/util/m5 ~/gem5/util/m5/m5op_x86.S

all: clean dummy master slave

clean:
	rm -f dummy
	rm -f master
	rm -f slave
