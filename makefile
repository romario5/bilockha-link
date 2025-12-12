CC = /home/roman/Downloads/luckfox-pico-main/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc
CFLAGS = -Wno-int-conversion -Wno-pointer-to-int-cast -Wno-int-to-pointer-cast -Wno-sign-compare -Wextra -O2 -MMD -MP 
LDFLAGS := -L$(TOOLCHAIN)/sysroot/usr/lib



# Custom includes
CFLAGS += -I src/hal
CFLAGS += -I src/sx128x
CFLAGS += -I$(TOOLCHAIN)/sysroot/usr/include


# Directories
SRC_DIR = src
OBJ_DIR := obj
BLD_DIR := build
HAL_DIR = $(SRC_DIR)/hal
SX128X_DIR = $(SRC_DIR)/sx128x

# Automatically find all .c files in src and its subdirectories
SRC_FILES = $(shell find $(SRC_DIR) -name "*.c")

# Generate object files corresponding to the .c files
OBJ_FILES = $(SRC_FILES:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Target Executable
TARGET = $(BLD_DIR)/vtx

# Default rule
all: hello clear init $(TARGET) finish

# Rule to create object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link object files into the final executable
$(TARGET): $(OBJ_FILES)
	$(CC) $(OBJ_FILES) $(LDFLAGS) -o $(TARGET)

hello:
	@echo "┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓"
	@echo "┃ Welcome to BILOCHKA LINK ┃"
	@echo "┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛"

clear:
	rm -rf $(OBJ_DIR)
	rm -rf $(BLD_DIR)

init: 
	mkdir $(OBJ_DIR)
	mkdir $(OBJ_DIR)/hal
	mkdir $(OBJ_DIR)/sx128x
	mkdir $(BLD_DIR)

finish:
	rm -rf $(OBJ_DIR)
	@echo "Build success. Enjoy!"
	
