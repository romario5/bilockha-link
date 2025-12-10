CC = /mnt/d/Drones/BilochkaLink/luckfox-pico-sdk/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc
CFLAGS = -Wall -Wextra -O2 -MMD -MP -I.
LDFLAGS = -lgpiod

CFLAGS += -I$(TOOLCHAIN)/sysroot/usr/include
LDFLAGS += -L$(TOOLCHAIN)/sysroot/usr/lib

LIBOBJ = sx1280.o
TARGET = vtx

OBJ = $(SRC:.c=.o)
DEP = $(OBJ:.o=.d)

all: $(TARGET)

$(TARGET): vtx.c sx1280.o
	$(CC) $(CFLAGS) vtx.c sx1280.o -o $(TARGET) $(LDFLAGS)
	
-include $(DEP)
	

clean:
	rm -f $(TARGET) $(LIBOBJ)
