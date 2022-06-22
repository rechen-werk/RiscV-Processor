#all:
#	ghdl -a -Wno-hide -Wno-library --std=08 -fsynopsys riscvsingle.vhdl
#	ghdl -e --std=08 -fsynopsys testbench
#	ghdl -r --std=08 -fsynopsys testbench --wave=trace.ghw --vcd=trace.vcd --ieee-asserts=disable || true

#clean:
#	rm -rf *.o *.cf testbench *.ghw


# TODO: change to more diverse Makefile; change -fsynopsys to --ieee=synopsys


MAINFILE = riscvsingle.vhdl

SIMDIR = sim
WORK = work

GHDL_CMD = ghdl
GHDL_FLAGS = --ieee=synopsys --workdir=$(SIMDIR) --work=$(WORK)
#GHDL_FLAGS = --std=93c --ieee=synopsys --workdir=$(SIMDIR) --work=$(WORK)

GHDL_SIM_OPT = --ieee-asserts=disable

WAVEFORM_VIEWER = gtkwave

.PHONY: syntax analyze elaborate run

all: syntax analyze elaborate run

syntax: 
	[ -d $(SIMDIR) ] || mkdir -p $(SIMDIR)
	@$(GHDL_CMD) -s $(GHDL_FLAGS) $(MAINFILE)

analyze: syntax
	@$(GHDL_CMD) -a $(GHDL_FLAGS) $(MAINFILE)

elaborate: analyze
	@$(GHDL_CMD) -a $(GHDL_FLAGS) $(MAINFILE)
	@$(GHDL_CMD) -e $(GHDL_FLAGS) testbench

#result.vcd: run
#result.ghw: run

run: elaborate
	@$(GHDL_CMD) -r $(GHDL_FLAGS) testbench --vcd=result.vcd --wave=result.ghw $(GHDL_SIM_OPT) 

view: result.vcd
	$(WAVEFORM_VIEWER) result.vcd

view_adv: result.ghw
	$(WAVEFORM_VIEWER) result.ghw

clean:
	rm -rf $(SIMDIR)
	rm -rf $(WORKDIR)
	rm -f  *.o
	rm -f  testbench
