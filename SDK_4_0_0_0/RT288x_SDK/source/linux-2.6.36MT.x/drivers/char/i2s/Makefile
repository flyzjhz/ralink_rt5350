obj-$(CONFIG_RALINK_I2S)  += ralink_i2s.o

ralink_i2s-objs       		:= i2s_ctrl.o i2s_debug.o

ifeq ($(CONFIG_RALINK_RT3052),y)
ralink_i2s-objs += wm8751.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8751
endif

ifeq ($(CONFIG_RALINK_RT2883),y)
ralink_i2s-objs += wm8751.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8751
endif

ifeq ($(CONFIG_RALINK_RT3350),y)
ralink_i2s-objs += wm8751.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8751
endif

ifeq ($(CONFIG_RALINK_RT3883),y)
ralink_i2s-objs += wm8750.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8750
endif

ifeq ($(CONFIG_RALINK_RT3352),y)
ralink_i2s-objs += wm8750.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8750
endif

ifeq ($(CONFIG_RALINK_RT5350),y)
ralink_i2s-objs += wm8750.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8750
endif

ifeq ($(CONFIG_RALINK_RT6855),y)
ralink_i2s-objs += wm8750.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8750
endif

ifeq ($(CONFIG_RALINK_RT63365),y)
ralink_i2s-objs += wm8750.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8750
endif

ifeq ($(CONFIG_RALINK_RT6352),y)
ralink_i2s-objs += wm8750.o
EXTRA_CFLAGS   +=-DCONFIG_I2S_WM8750
endif

