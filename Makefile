
SRC = src
TST_SRC = test
BUILD_DIR = build/$(CC)-$(OPT)
TEST_BUILD = $(BUILD_DIR)/$(TST_SRC)

COMPILERS = gcc clang
OPT_LEVELS = O0 Os O2 O3
TARGET_LIST = host cortexm4
CFLAGS = -Wall -Wextra
CORTEXM4_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LIB_FILES = matf32 linsolve quadprog robotat_control robotat_robotics
TEST_FILES = matf32_test linsolve_test quadprog_qp_test quadprog_sqp_test

AR := ar
CC := gcc
OPT := O2
TARGET := host

OBJS = $(LIB_FILES:%=$(BUILD_DIR)/%.o)
TEST_OBJS = $(TEST_FILES:%=$(TEST_BUILD)/%.o)
TEST_BINS = $(TEST_FILES:%=$(TEST_BUILD)/%)
LIB = $(BUILD_DIR)/librobotat.a

static_library: $(LIB)
static_test: $(TEST_BINS)

matrix:
	@mkdir -p logs
	@for cc in $(COMPILERS); do \
		for o in $(OPT_LEVELS); do \
			echo "== $$cc $$o"; \
			$(MAKE) -k static_library static_test CC=$$cc OPT=$$o > logs/$$cc-$$o.log 2>&1 \
			&& echo "   ok" || echo "   FAILED (see logs/$$cc-$$o.log)"; \
		done; \
	done

$(LIB): $(OBJS)
	$(AR) rcs $@ $^

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

.PHONY: static_library static_test clean matrix