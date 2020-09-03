#
# Makefile for building the SGI L1 USB driver loadable module
#
# NOTE:  the body  this Makefile is for the 2.6 kernel driver build;
# the 2.4 version of the Makefile is included at the end
#

KVER=$(shell uname -r | cut -d'.' -f1-2)

ifeq ($(KVER),2.6)

#
# 2.6 kernel driver build Makefile
#


############################################################################
# This portion of the Makefile is used when the make is executed
# from within the kernel source tree   (when this Makefile is re-read
# after the outer make invocation is called)
############################################################################
ifneq ($(KERNELRELEASE),)

obj-m       := sgil1.o 


############################################################################
# The remainder of the makefile is executed
# from outside the kernel source tree
############################################################################

else

# use current kernel if none specified
ifndef KERNEL_VERSION
KERNEL_VERSION=$(shell uname -r)
endif



all : build install

#
# kernel module build defines/targets
#

KDIR        := /lib/modules/$(KERNEL_VERSION)/build
PWD         := $(shell pwd)

build : sgil1.ko

sgil1.ko sgil1.o : sgil1.c
	$(MAKE) -C $(KDIR) V=1 SUBDIRS=$(PWD) modules

#
# kernel module install defines/targets
#
# module install location and install tool
KERNEL_MOD_DIR=/lib/modules/$(KERNEL_VERSION)
INSTALLDIR=$(KERNEL_MOD_DIR)/kernel/drivers/usb/misc
INSTALL=`which install`

install : package-install

package-install : sgil1.ko
	@if [ `id -ur` != 0 ]; then \
		echo "Please run \"make install\" as root."; \
	elif [ ! -d $(INSTALLDIR) ]; then \
		echo "The install directory ($(INSTALLDIR)) does not exist!"; \
	else \
		$(INSTALL) -m 0664 -o root -g root sgil1.ko $(INSTALLDIR)/sgil1.ko && \
		echo "sgil1.o driver installed to $(INSTALLDIR)/sgil1.o" && \
		$(INSTALL) -m 0775 -o root -g root sgil1.initscript /etc/init.d/sgil1 && \
		/sbin/chkconfig --add sgil1 && \
		echo "sgil1 initscript installed to /etc/init.d/sgil1"; \
		if [ "$(KERNEL_VERSION)" = `uname -r` ]; then \
			/sbin/depmod -a && \
			if [ -x /etc/init.d/sgil1 ]; then \
				/etc/init.d/sgil1 start; \
			else \
				echo "ERROR! could not execute the sgil1 initscript /etc/init.d/sgil1\n"; \
			fi; \
		fi; \
	fi

clean :
	$(MAKE) -C $(KDIR) V=1 SUBDIRS=$(PWD) $(@)

endif    # end of 2.6 kernel source tree


#
# Makefile support for earlier kernel builds (2.4) start here
#

else

  ifeq ($(KVER),2.4)
    include Makefile.24
  else
    $(error "Unsupported kernel version!  ($(KVER))");
  endif

endif    # end of 2.4 kernel build support
