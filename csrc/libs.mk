# Build the C a JuliaC application calls into as shared libraries: `make -f libs.mk`.
#
# Copied into the application by QuanserComponents.export_program_juliac, and run by
# build_program_juliac on the machine (or in the root file system) the binary is built for, so
# the libraries have the binary's architecture. The SDK layouts are detected as in `Makefile`.
# Unlike that file, a missing SDK is not an error: the libraries are then built with the
# callback backend only, so the binary loads and runs, but qube_hw_open(QUBE_HW_MODE_HIL)
# fails at runtime. That is what makes a build without the SDK installed testable.
QUANSER_DIR ?= /opt/quanser/hil_sdk
CC          ?= cc

ifneq ($(wildcard $(QUANSER_DIR)/include/hil.h),)
  HIL_CFLAGS := -DQUBE_HW_HAVE_HIL -I$(QUANSER_DIR)/include
  HIL_LIBS   := -L$(QUANSER_DIR)/lib -lhil -lquanser_runtime -lquanser_common -lrt -lpthread -ldl -lm
else ifneq ($(wildcard /usr/include/quanser/hil.h),)
  HIL_CFLAGS := -DQUBE_HW_HAVE_HIL -I/usr/include/quanser
  HIL_LIBS   := -lhil -lquanser_runtime -lquanser_common -lrt -lpthread -ldl -lm
else
  $(warning Quanser HIL SDK not found: libqube_hw is built without the HIL backend)
endif

CFLAGS += -I. -O2 -Wall -fPIC

libs: libqube_hw.so libqube_log.so libqube_traj.so

libqube_hw.so: qube_hw.c qube_hw.h
	$(CC) $(CFLAGS) $(HIL_CFLAGS) -shared qube_hw.c -o $@ $(HIL_LIBS)

libqube_log.so: qube_log.c qube_log.h
	$(CC) $(CFLAGS) -shared qube_log.c -o $@

libqube_traj.so: qube_traj.c qube_traj.h
	$(CC) $(CFLAGS) -shared qube_traj.c -o $@ -lm

.PHONY: libs clean
clean:
	rm -f libqube_hw.so libqube_log.so libqube_traj.so
