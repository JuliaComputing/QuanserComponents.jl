# Build the hardware control loop for an exported QUBE program: `make`.
#
# Copied verbatim by QuanserComponents.export_program_c — it needs nothing from Julia,
# since the SDK layout is detected below and everything program-specific arrives
# through the generated run_hardware_config.h.
#
# qube_hw.c is the hardware I/O the generated node calls into and qube_log.c is the
# logging it calls into; -DQUBE_HW_HAVE_HIL selects the Quanser HIL SDK backend (without
# it, only the callback backend is compiled in and qube_hw_open(QUBE_HW_MODE_HIL) fails
# at runtime).
#
# Two SDK layouts are supported and detected below, so the same sources build on an
# x86 host with the older HIL SDK and on a Raspberry Pi with the `quanser-sdk`
# packages. Override QUANSER_DIR to force the first.
QUANSER_DIR ?= /opt/quanser/hil_sdk
CC          ?= cc

ifneq ($(wildcard $(QUANSER_DIR)/include/hil.h),)
  QUANSER_CFLAGS  := -I$(QUANSER_DIR)/include
  QUANSER_LDFLAGS := -L$(QUANSER_DIR)/lib
else ifneq ($(wildcard /usr/include/quanser/hil.h),)
  QUANSER_CFLAGS  := -I/usr/include/quanser
  QUANSER_LDFLAGS :=
else
  $(error Quanser HIL SDK not found: no $(QUANSER_DIR)/include/hil.h and no /usr/include/quanser/hil.h)
endif

CFLAGS  += -I. $(QUANSER_CFLAGS) -O2 -Wall -DQUBE_HW_HAVE_HIL
LDFLAGS += $(QUANSER_LDFLAGS)
LIBS    += -lhil -lquanser_runtime -lquanser_common -lrt -lpthread -ldl -lm

run_hardware: run_hardware.c run_hardware_config.h top.c top.h qube_hw.c qube_hw.h qube_log.c qube_log.h synchjulia.h
	$(CC) $(CFLAGS) run_hardware.c top.c qube_hw.c qube_log.c -o $@ $(LDFLAGS) $(LIBS)

.PHONY: clean
clean:
	rm -f run_hardware *.csv
