Sources
=======

 * Scratchpad implementation located at src/mem/ (scratchpad.cc, scratchpad.hh,
   Sratchpad.py)
 * Tcu implementation located at src/mem/tcu/
 * Tester CPU located at /src/cpu/testers/tcutest
 * Configuration script configs/example/tcutest.py

Build
=====

 * Dependencies are listed [here](http://www.m5sim.org/Dependencies).
 * The build system is described [here](http://www.m5sim.org/Build_System).
 * For now I simply use this target to build: scons build/ALPHA/gem5.debug

Run
===

For testing I use this comand:
```build/ALPHA/gem5.debug --debug-flags=TcuTest,Tcu,MemoryAccess,XBar configs/example/tcutest.py --num-pes=2 [--atomic]```

The test script creates a simple system consisting of num_pes (--num-pes)
processing elements. Each PE consists of a CPU, a Scratchpad-Memory, and a TCU.
These units are connected via a simple crossbar. All the PEs are also connected
via a crossbar.

For now the TCU simply forwards all requests from the CPU to the local
Scratchpad. The Tester-CPU simply performs a view read and write accesses
directly on the Scratchpad-Memory and indirectly via the TCU.
