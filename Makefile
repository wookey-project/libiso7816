###################################################################
# About the library name and path
###################################################################

# library name, without extension
LIB_NAME ?= libiso7816

# project root directory, relative to app dir
PROJ_FILES = ../../

# library name, with extension
LIB_FULL_NAME = $(LIB_NAME).a

# SDK helper Makefiles inclusion
-include $(PROJ_FILES)/m_config.mk
-include $(PROJ_FILES)/m_generic.mk

# use an app-specific build dir
APP_BUILD_DIR = $(BUILD_DIR)/libs/$(LIB_NAME)

###################################################################
# About the compilation flags
###################################################################

CFLAGS := $(LIBS_CFLAGS)
CFLAGS += -MMD -MP

#############################################################
# About library sources
#############################################################

SRC_DIR = .
SRC = $(wildcard $(SRC_DIR)/*.c)
OBJ = $(patsubst %.c,$(APP_BUILD_DIR)/%.o,$(SRC))
DEP = $(OBJ:.o=.d)

OUT_DIRS = $(dir $(OBJ))

# file to (dist)clean
# objects and compilation related
TODEL_CLEAN += $(OBJ)
# targets
TODEL_DISTCLEAN += $(APP_BUILD_DIR)

##########################################################
# generic targets of all libraries makefiles
##########################################################


.PHONY: app doc

default: all

all: $(APP_BUILD_DIR) lib

doc:
	$(Q)$(MAKE) BUILDDIR=../$(APP_BUILD_DIR)/doc  -C doc html latexpdf

show:
	@echo
	@echo "\tAPP_BUILD_DIR\t=> " $(APP_BUILD_DIR)
	@echo
	@echo "C sources files:"
	@echo "\tSRC_DIR\t\t=> " $(SRC_DIR)
	@echo "\tSRC\t\t=> " $(SRC)
	@echo "\tOBJ\t\t=> " $(OBJ)
	@echo

lib: $(APP_BUILD_DIR)/$(LIB_FULL_NAME)

#############################################################
# build targets (driver, core, SoC, Board... and local)
# App C sources files
$(APP_BUILD_DIR)/%.o: %.c
	$(call if_changed,cc_o_c)

# lib
$(APP_BUILD_DIR)/$(LIB_FULL_NAME): $(OBJ)
	$(call if_changed,mklib)
	$(call if_changed,ranlib)

$(APP_BUILD_DIR):
	$(call cmd,mkdir)

-include $(DEP)
-include $(TESTSDEP)

#####################################################################
# Frama-C
#####################################################################

# This variable is to be overriden by local shell environment variable to
# compile and use frama-C targets
# by default, FRAMAC target is deactivated, it can be activated by overriding
# the following variable value with 'y' in the environment.
FRAMAC_TARGET ?= n

ifeq (y,$(FRAMAC_TARGET))

# some FRAMAC arguments may vary depending on the FRAMA-C version (Calcium, Scandium...)
# Here we support both Calcium (20) and Scandium (21) releases
FRAMAC_VERSION=$(shell frama-c -version|cut -d'.' -f 1)
FRAMAC_RELEASE=$(shell frama-c -version|sed -re 's:^.*\((.*)\)$:\1:g')

#
# INFO: Using Frama-C, the overall flags are not directly used as they are targetting
# arm-none-eabi architecture which is not handled by framaC. Instead, we used
# a 32bits target with custom CFLAGS to handle Frama-C compilation step.
# As a consequence, include paths need to be set here as above CFLAGS are dissmissed.
# dir of drviso7816 API
DRVISO7816_API_DIR ?= $(PROJ_FILES)/drivers/socs/$(SOC)/drviso7816/api

LIBSMARTCARD_API_DIR ?= $(PROJ_FILES)/libs/smartcard/api

# This is the Wookey micro-libC API directory. This directory is used by all libraries and driver
# and defines all prototypes and C types used nearly everywhere in the Wookey project.
LIBSTD_API_DIR ?= $(PROJ_FILES)/libs/std/api

# This is the EwoK kernel exported headers directory. These headers are requested by the libstd
# itself and thus by upper layers, including drivers and libraries.
EWOK_API_DIR ?= $(PROJ_FILES)/kernel/src/C/exported

FRAMAC_RESULTSDIR := framac/results

SESSION     := $(FRAMAC_RESULTSDIR)/frama-c-rte-eva-wp-ref.session
LOGFILE     := $(FRAMAC_RESULTSDIR)/frama-c-rte-eva-wp-ref.log
EVA_SESSION := $(FRAMAC_RESULTSDIR)/frama-c-rte-eva.session
EVA_LOGFILE := $(FRAMAC_RESULTSDIR)/frama-c-rte-eva.log
EVAREPORT	:= $(FRAMAC_RESULTSDIR)/frama-c-rte-eva_report
TIMESTAMP   := $(FRAMAC_RESULTSDIR)/timestamp-calcium_wp-eva.txt
JOBS        := $(shell nproc)
# Does this flag could be overriden by env (i.e. using ?=)
TIMEOUT     := 15


$(FRAMAC_RESULTSDIR):
	$(call cmd,mkdir)

FRAMAC_GEN_FLAGS:=\
			-absolute-valid-range 0x40040000-0x40044000 \
			-no-frama-c-stdlib \
	        -warn-left-shift-negative \
	        -warn-right-shift-negative \
	        -warn-signed-downcast \
	        -warn-signed-overflow \
	        -warn-unsigned-downcast \
	        -warn-unsigned-overflow \
	        -warn-invalid-pointer \
			-kernel-msg-key pp \
			-cpp-extra-args="-nostdinc -I framac/include -I $(LIBSTD_API_DIR) -I $(LIBSMARTCARD_API_DIR) -I $(DRVISO7816_API_DIR) -I $(EWOK_API_DIR)"  \
		    -rte \
		    -instantiate

FRAMAC_EVA_FLAGS:=\
		    -eva \
		    -eva-show-perf \
		    -eva-slevel 500 \
		    -eva-domains symbolic-locations\
		    -eva-domains equality \
		    -eva-split-return auto \
		    -eva-partition-history 3 \
            -eva-use-spec platform_is_smartcard_inserted \
            -eva-use-spec platform_set_smartcard_rst \
            -eva-use-spec platform_set_smartcard_vcc \
            -eva-use-spec platform_smartcard_early_init \
            -eva-use-spec platform_smartcard_init \
            -eva-use-spec platform_SC_adapt_clocks \
            -eva-use-spec platform_SC_set_direct_conv \
            -eva-use-spec platform_SC_set_inverse_conv \
            -eva-use-spec platform_SC_flush \
            -eva-use-spec platform_SC_getc \
            -eva-use-spec platform_SC_putc \
            -eva-use-spec platform_get_microseconds_ticks \
            -eva-use-spec platform_SC_reinit_smartcard_contact \
            -eva-use-spec platform_SC_reinit_iso7816 \
            -eva-use-spec platform_smartcard_lost \
            -eva-use-spec platform_smartcard_reinit \
            -eva-use-spec platform_smartcard_register_user_handler_action \
            -eva-use-spec platform_smartcard_map \
            -eva-use-spec platform_smartcard_unmap \
            -eva-use-spec platform_smartcard_set_1ETU_guardtime \
		    -eva-log a:$(EVA_LOGFILE)\
			-eva-report-red-statuses $(EVAREPORT)

ifeq (22,$(FRAMAC_VERSION))
	FRAMAC_WP_SUPP_FLAGS= # -wp-check-memory-model, asserts are to be added first: invalid wp behavior by now
else
FRAMAC_WP_SUPP_FLAGS=
endif

FRAMAC_WP_PROVERS ?= alt-ergo,cvc4,z3

FRAMAC_WP_FLAGS:=\
	        -wp \
  			-wp-model "Typed+ref+int" \
			$(FRAMAC_WP_SUPP_FLAGS)\
  			-wp-literals \
  			-wp-prover $(FRAMAC_WP_PROVERS),tip \
			-wp-prop="-@lemma" \
			-wp-time-margin 25 \
   			-wp-timeout $(TIMEOUT) \
			-wp-smoke-tests \
			-wp-no-smoke-dead-code \
   			-wp-log a:$(LOGFILE)

FRAMAC_WP_LEMMAS_FLAGS:=\
			-wp-prop="@lemma" \
			-wp-auto="wp:split,wp:bitrange"

frama-c-parsing: $(FRAMAC_RESULTSDIR)
	frama-c framac/entrypoint.c *.c  \
		 -c11 \
		 -no-frama-c-stdlib \
		 -cpp-extra-args="-nostdinc -I framac/include -I $(LIBSTD_API_DIR) -I $(LIBSMARTCARD_API_DIR) -I $(DRVISO7816_API_DIR) -I $(EWOK_API_DIR)"

frama-c-eva: $(FRAMAC_RESULTSDIR)
	frama-c framac/entrypoint.c smartcard*.c \
		    $(FRAMAC_GEN_FLAGS) \
			$(FRAMAC_EVA_FLAGS) \
			-save $(EVA_SESSION) \
   			-time $(TIMESTAMP)

frama-c: $(FRAMAC_RESULTSDIR)
	frama-c framac/entrypoint.c smartcard*.c \
		    $(FRAMAC_GEN_FLAGS) \
			$(FRAMAC_EVA_FLAGS) \
   		    -then \
			$(FRAMAC_WP_FLAGS) \
   			-save $(SESSION) \
			-then \
			$(FRAMAC_WP_LEMMAS_FLAGS) \
			-time $(TIMESTAMP)

frama-c-instantiate: $(FRAMAC_RESULTSDIR)
	frama-c framac/entrypoint.c -c11 -machdep x86_32 \
			$(FRAMAC_GEN_FLAGS) \
			-instantiate

frama-c-gui:
	frama-c-gui -load $(SESSION)

endif
