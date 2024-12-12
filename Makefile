#
# Top Level Mission Makefile
#
BUILDTYPE ?= debug
INSTALLPREFIX ?= exe
FSWBUILDDIR ?= $(CURDIR)/fsw/build
GSWBUILDDIR ?= $(CURDIR)/gsw/build
OPENC3BUILDDIR ?= $(CURDIR)/gsw/openc3-cosmos-nos3/build
SIMBUILDDIR ?= $(CURDIR)/sims/build

export CFS_APP_PATH = ../components
export MISSION_DEFS = ../cfg/build/
export MISSIONCONFIG = ../cfg/build/nos3

# The "prep" step requires extra options that are specified via enviroment variables.
# Certain special ones should be passed via cache (-D) options to CMake.
# These are only needed for the "prep" target but they are computed globally anyway.
PREP_OPTS :=

ifneq ($(INSTALLPREFIX),)
PREP_OPTS += -DCMAKE_INSTALL_PREFIX=$(INSTALLPREFIX)
endif

ifneq ($(VERBOSE),)
PREP_OPTS += --trace
endif

ifneq ($(BUILDTYPE),)
PREP_OPTS += -DCMAKE_BUILD_TYPE=$(BUILDTYPE)
endif

# The "LOCALTGTS" defines the top-level targets that are implemented in this makefile
# Any other target may also be given, in that case it will simply be passed through.
LOCALTGTS := all checkout clean clean-fsw clean-sim clean-gsw config debug fsw gsw launch log prep real-clean sim stop stop-gsw
OTHERTGTS := $(filter-out $(LOCALTGTS),$(MAKECMDGOALS))

# As this makefile does not build any real files, treat everything as a PHONY target
# This ensures that the rule gets executed even if a file by that name does exist
.PHONY: $(LOCALTGTS) $(OTHERTGTS)

#
# Commands
#
all:
	$(MAKE) config
	$(MAKE) fsw
	$(MAKE) sim
	$(MAKE) gsw

build-cryptolib:
	cd $(GSWBUILDDIR) 
	$(MAKE) --no-print-directory -C $(GSWBUILDDIR)

build-fsw:
	cd $(FSWBUILDDIR) 
	$(MAKE) --no-print-directory -C $(FSWBUILDDIR) mission-install

build-gsw:
	cd $(OPENC3BUILDDIR)
	$(MAKE) --no-print-directory -C $(OPENC3BUILDDIR)
	cd $(OPENC3BUILDDIR)/openc3-cosmos-nos3 && ~/.nos3/cosmos/openc3.sh cli rake build VERSION=1.0
	
build-sim:
	cd $(SIMBUILDDIR) 
	$(MAKE) --no-print-directory -C $(SIMBUILDDIR) install

build-test:
	cd $(FSWBUILDDIR)
	$(MAKE) --no-print-directory -C $(FSWBUILDDIR) mission-install

cmake:
	$(MAKE) config
	./scripts/docker_cmake_fsw.sh
	$(MAKE) cmake-gsw
	./scripts/docker_cmake_sim.sh
	./scripts/docker_cmake_cryptolib.sh

cmake-cryptolib:
	mkdir -p $(GSWBUILDDIR)
	cd $(GSWBUILDDIR) && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $(PREP_OPTS) -DSUPPORT=1 ../../components/cryptolib

cmake-fsw:
	mkdir -p $(FSWBUILDDIR)
	cd $(FSWBUILDDIR) && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $(PREP_OPTS) ../cfe

cmake-gsw:
	mkdir -p $(OPENC3BUILDDIR)	
	cd $(OPENC3BUILDDIR) && cmake -DBASE_DIR=$(CURDIR) -DGSW_DIR=$(CURDIR)/gsw/cosmos ..

cmake-sim:
	mkdir -p $(SIMBUILDDIR)
	cd $(SIMBUILDDIR) && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_INSTALL_PREFIX=$(SIMBUILDDIR) ..

cmake-test:
	mkdir -p $(FSWBUILDDIR)
	cd $(FSWBUILDDIR) && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $(PREP_OPTS) -DENABLE_UNIT_TESTS=true ../cfe
	
checkout:
	./scripts/docker_checkout.sh

clean:
	$(MAKE) clean-fsw
	$(MAKE) clean-sim
	$(MAKE) clean-gsw
	rm -rf cfg/build

clean-fsw:
	rm -rf cfg/build/nos3_defs
	rm -rf fsw/build

clean-sim:
	rm -rf sims/build

clean-gsw:
	rm -rf gsw/build
	rm -rf gsw/cosmos/build
	rm -rf gsw/openc3-cosmos-nos3/build
	rm -rf /tmp/nos3

config:
	./scripts/config.sh

debug:
	./scripts/docker_debug.sh

fsw: 
	./scripts/docker_build_fsw.sh

gsw: build-gsw
	./scripts/docker_build_cryptolib.sh
	# ./cfg/build/gsw_build.sh

launch:
	./scripts/docker_compose_launch2.sh

launch-gsw:
	docker compose -f ~/.nos3/cosmos/compose.yaml up -d
	cd $(OPENC3BUILDDIR)/openc3-cosmos-nos3 && ~/.nos3/cosmos/openc3.sh cliroot load openc3-cosmos-nos3-1.0.gem

log:
	./scripts/log.sh

prep:
	./scripts/prepare.sh

real-clean:
	$(MAKE) clean
	./scripts/real_clean.sh

sim:
	./scripts/docker_build_sim.sh

stop:
	docker network disconnect scripts_nos3_sc_1 cosmos-openc3-operator-1
	docker compose -f scripts/docker-compose.yml down
	# ./scripts/docker_stop.sh
	# ./scripts/stop.sh

stop-gsw:
	docker compose -f ~/.nos3/cosmos/compose.yaml down
	# ./scripts/stop_gsw.sh

test-fsw:
	cd $(FSWBUILDDIR)/amd64-posix/default_cpu1 && ctest -O ctest.log

igniter:
	./scripts/igniter_launch.sh
