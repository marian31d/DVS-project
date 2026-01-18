# ============================================================
# Makefile for VCS + Verdi (FSDB) + Coverage
# Targets:
#   make all        -> clean + comp + run + wave
#   make comp       -> compile (FSDB enabled + timescale)
#   make run        -> run sim, create novas.fsdb
#   make wave       -> open verdi waveform (novas.fsdb)
#   make cov_run    -> compile with coverage + run -> creates simv.vdb
#   make cov_view   -> open verdi coverage on simv.vdb
#   make clean      -> remove generated files
# ============================================================

VCS      = vcs
SIMV     = ./simv
VERDI    = verdi

# Your file list
FILELIST = build.cud

# ---- Compile flags (regular) ----
VCS_FLAGS = -f $(FILELIST) -sverilog -kdb +vcs+fsdbon -timescale=1ns/1ps

# ---- Coverage compile flags ----
# line/toggle/cond/branch/fsm coverage + create simv.vdb
# (You can remove +fsm if your flow doesn?t support it, but usually it does.)
COV_FLAGS = -cm line+tgl+cond+branch+fsm -cm_dir simv.vdb

# ---- Run flags ----
# -cm enables dumping coverage during run when compiled with -cm...
RUN_FLAGS =
RUN_COV_FLAGS = -cm line+tgl+cond+branch+fsm

# ---- Grid run command (you are using qrsh) ----
QRSH_RUN = qrsh -V -cwd -b y -q normal

.PHONY: all test clean comp run wave cov_run cov_view cov_clean sim

all: clean comp run wave

test: comp run

# -------------------- CLEAN --------------------
clean:
	\rm -rf simv* csrc* *.log *.fsdb *.rc *.key verdi_config_file verdiLog *.conf \
	       novas.fsdb DVEfiles ucli.key vcs.key log

cov_clean:
	\rm -rf simv.vdb urgReport verdiLog

# -------------------- COMPILE (NO COVERAGE) --------------------
comp:
	vcs \
	/data/synopsys/lib/SAED32_EDK/lib/stdcell_hvt/verilog/saed32nm_hvt.v \
	/data/synopsys/lib/SAED32_EDK/lib/stdcell_lvt/verilog/saed32nm_lvt.v \
	/data/synopsys/lib/SAED32_EDK/lib/stdcell_rvt/verilog/saed32nm.v \
	-f build.cud -sverilog -kdb +vcs+fsdbon+lint=TFIPC-L -sdf typ:vctest.dut:switch1.sdf

# -------------------- RUN (NO COVERAGE) --------------------
run:
	$(QRSH_RUN) $(SIMV) $(RUN_FLAGS) 2>&1 | tee log

# -------------------- WAVES (FSDB) --------------------
wave:
	$(VERDI) -ssf novas.fsdb &

# -------------------- COMPILE + RUN WITH COVERAGE --------------------
cov_run: cov_clean
	$(VCS) $(VCS_FLAGS) $(COV_FLAGS) -o simv
	$(QRSH_RUN) $(SIMV) $(RUN_COV_FLAGS) 2>&1 | tee log_cov

# -------------------- OPEN COVERAGE VIEW --------------------
cov_view:
	$(VERDI) -cov -covdir simv.vdb &

# optional shortcut
sim: comp run
