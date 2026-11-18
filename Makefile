
SRC = src
TST_SRC = test
BUILD_DIR = build/$(CC)-$(OPT)
TEST_BUILD = $(BUILD_DIR)/$(TST_SRC)
ARM_BUILD = build/cortex-m4-$(OPT)

COMPILERS = gcc clang
OPT_LEVELS = O0 Os O2 O3
CFLAGS = -Wall -Wextra
CORTEXM4_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LIB_FILES = matf32 linsolve quadprog robotat_control robotat_robotics
TEST_FILES = matf32_test linsolve_test quadprog_qp_test quadprog_sqp_test control_pid_test control_lti_sys_test control_nonlin_sys_test control_kalman_filter_test control_mpc_test robotics_test

AR := ar
AR_ARM = arm-none-eabi-ar
CC := gcc
CC_ARM = arm-none-eabi-gcc
OPT := O2
TARGET := host

OBJS = $(LIB_FILES:%=$(BUILD_DIR)/%.o)
ARM_OBJS = $(LIB_FILES:%=$(ARM_BUILD)/%.o)
TEST_OBJS = $(TEST_FILES:%=$(TEST_BUILD)/%.o)
TEST_BINS = $(TEST_FILES:%=$(TEST_BUILD)/%)
LIB = $(BUILD_DIR)/librobotat.a
ARM_LIB = $(ARM_BUILD)/librobotat.a

static_library: $(LIB)
static_test: $(TEST_BINS)
static_arm_library: $(ARM_LIB)

all:
	@mkdir -p logs
	@for cc in $(COMPILERS); do \
		for o in $(OPT_LEVELS); do \
			echo "== $$cc $$o"; \
			$(MAKE) -k static_library static_test CC=$$cc OPT=$$o > logs/$$cc-$$o.log 2>&1 \
			&& echo "   OK" || echo "   FAILED (see logs/$$cc-$$o.log)"; \
		done; \
	done

	@for o in $(OPT_LEVELS); do \
		echo "== cortexm4 $$o"; \
		$(MAKE) -k static_arm_library OPT=$$o > logs/cortexm4-$$o.log 2>&1 \
		&& echo "   OK" || echo "   FAILED (see logs/cortexm4-$$o.log)"; \
	done;

$(LIB): $(OBJS)
	$(AR) rcs $@ $^

$(ARM_LIB): $(ARM_OBJS)
	$(AR_ARM) rcs $@ $^

$(ARM_BUILD)/%.o: $(SRC)/%.c
	@mkdir -p $(@D)
	$(CC_ARM) $(CFLAGS) $(CORTEXM4_FLAGS) -$(OPT) -c $< -o $@

$(BUILD_DIR)/%.o: $(SRC)/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -$(OPT) -c $< -o $@

$(TEST_BUILD)/%: $(TEST_BUILD)/%.o $(LIB)
	$(CC) $< $(LIB) -lm -o $@

$(TEST_BUILD)/%.o: $(TST_SRC)/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -$(OPT) -I$(SRC) -c $< -o $@

clean: 
	rm -rf build

.PHONY: static_library static_test static_arm_library clean all