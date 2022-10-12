###############################################################################
#
# Copyright (C) 2022 THALES DIS AIS Deutschland GmbH <CinterionWMSupport@thalesgroup.com>
# Company name change from Gemalto M2M GmbH to THALES DIS AIS Deutschland GmbH
# Copyright (C) 2013 Gemalto M2M GmbH
#
# All Rights Reserved.
#
# Gemalto provides this source code under the GPL v2 License.
# The GPL v2 license is available at
#
# https://opensource.org/licenses/gpl-license.php
#
###############################################################################

###############################################################################
# User parameters - Start
###############################################################################

# Use multiplex protocol in host or in client mode. This provides not the
# possibility for building the driver as client driver just by modifying this
# flag. It is only for simpler reuse of some source files.
client_mode         := 0

# Enable or disable trace output
trace_enabled       := 0

# Enable or disable power management
power_mgmt_enabled  := 0


###############################################################################
# User parameters - End
###############################################################################

ifeq ($(CONFIG_LINMUX_CLIENT_MODE),y)
client_mode         := 1
endif

ifeq ($(CONFIG_LINMUX_TRACE_ENABLED),y)
trace_enabled       := 1
endif

ifeq ($(CONFIG_LINMUX_POWER_MGMT_ENABLED),y)
power_mgmt_enabled  := 1
endif

# Make section for kernel integration

ccflags-y += -DTTY_DEVNUM=$(CONFIG_LINMUX_TTY_MAJOR_DEVNUM)
ccflags-y += -DFS_DEVNUM=$(CONFIG_LINMUX_FS_MAJOR_DEVNUM)
ccflags-y += -DFS_DEVNAME='$(CONFIG_LINMUX_FS_DEVICE_NAME)'
ccflags-y += -DCLIENT_MODE=$(client_mode)
ccflags-y += -DTRACE_ENABLED=$(trace_enabled)
ccflags-y += -DPWRMGMT_ENABLED=$(power_mgmt_enabled)

obj-y += linmuxtty.o
obj-y += linmuxcfg.o
obj-y += linmuxpwr.o
obj-y += os_wrap.o
obj-y += ringbuffer.o
obj-y += baseport.o
obj-y += mux_interface.o
obj-y += muxchannel.o
obj-y += mux_msgqueue.o
obj-y += muxdbg.o
obj-y += ddmpadp.o
obj-y += ddmpfra.o
obj-y += ddmpfunc.o
obj-y += ddmphdlc.o
obj-y += ddmpiov2.o
obj-y += ddmpmem.o
obj-y += ddmppkt.o
obj-y += ddmpstat.o
obj-y += ddmptype.o

