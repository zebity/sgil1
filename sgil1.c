/*
 * SGI L1 USB driver
 *
 * This file includes the appropriate source file depending on
 * the kernel version the driver is being built for
 */

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,4,0)
#include "sgil1_54.c"
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#include "sgil1_26.c"
#else
#include "sgil1_24.c"
#endif
