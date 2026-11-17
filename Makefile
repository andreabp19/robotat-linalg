
SRC = src
BUILD_DIR = build/$(CC)-$(OPT)

COMPILERS = gcc clang
OPT_LEVELS = O0 Os O2 O3
TARGET_LIST = host cortexm4
CFLAGS = -Wall -Wextra
CORTEXM4_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LIB_FILES = matf32 linsolve quadprog robotat_control robotat_robotics

CC := gcc
OPT := O2
FILE := matf32
TARGET := host

OBJS = $(LIB_FILES:%=$(BUILD_DIR)/%.o)

static_library: $(OBJS)

$(BUILD_DIR)/%.o: $(SRC)/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -$(OPT) -c $< -o $@

.PHONY: static_library