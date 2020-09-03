#ifndef _SGIL1_H
#define _SGIL1_H

#include <linux/ioctl.h>
#include <linux/version.h>

/* current driver revision */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#define DRIVER_VERSION		"v4.0 (09/30/2004)"
#else
#define DRIVER_VERSION		"v3.0 (02/07/2003)"
#endif

#define DRIVER_AUTHOR		"Steve Hein <ssh@sgi.com> and Bob Cutler <rwc@sgi.com>"
#define DRIVER_DESC		"USB L3 driver for SGI L1 system controller"
#define SGIL1_VENDOR_ID		0x065E
#define SGIL1_PRODUCT_ID	0x1234

#define SGIL1_MAX_LEVEL		6

typedef struct {
	unsigned char bus;
	unsigned char level;
	unsigned char dev;
	unsigned char path[SGIL1_MAX_LEVEL];
} sgil1_cfg_t;

#define SGIL1_IOCTL_BASE	'S'

/* read/write endpoint ioctl's */
#define SGIL1_RESET_READ	_IO(SGIL1_IOCTL_BASE, 1)
#define SGIL1_RESET_WRITE	_IO(SGIL1_IOCTL_BASE, 2)
#define SGIL1_RESET_DEVICE	_IO(SGIL1_IOCTL_BASE, 3)
#define SGIL1_READ_CFG		_IOR(SGIL1_IOCTL_BASE, 4, sgil1_cfg_t)
#define SGIL1_RESET_PIPES	_IO(SGIL1_IOCTL_BASE, 5)

/* control endpoint ioctl's */
#define SGIL1_ST_READ_REV	_IOR(SGIL1_IOCTL_BASE, 6, int)
#define SGIL1_ST_READ_DEV_CFG	_IOWR(SGIL1_IOCTL_BASE, 7, sgil1_cfg_t)

#endif /* _SGIL1_H */
