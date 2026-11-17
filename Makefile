
SRC = src
BUILD_DIR = build/$(CC)-$(OPT)

COMPILERS = gcc clang
OPT_LEVELS = O0 Os O2 O3
TARGET_LIST = host cortexm4
CFLAGS = -Wall -Wextra
CORTEXM4_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LIB_FILES = matf32 linsolve quadprog robotat_control robotat_robotics

AR := ar
CC := gcc
OPT := O2
TARGET := host

OBJS = $(LIB_FILES:%=$(BUILD_DIR)/%.o)
LIB = $(BUILD_DIR)/librobotat.a

static_library: $(LIB)

matrix:
	@mkdir -p logs
	@for cc in $(COMPILERS); do \
		for o in $(OPT_LEVELS); do \
			echo "== $$cc $$o"; \
			$(MAKE) static_library CC=$$cc OPT=$$o > logs/$$cc-$$o.log 2>&1 \
			&& echo "   ok" || echo "   FAILED (see logs/$$cc-$$o.log)"; \
		done; \
	done

$(LIB): $(OBJS)
	$(AR) rcs $@ $^

$(BUILD_DIR)/%.o: $(SRC)/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -$(OPT) -c $< -o $@

clean: 
	rm -rf build

.PHONY: static_library clean matrix