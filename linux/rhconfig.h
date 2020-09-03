/*
 * Try to be a little smarter about which kernel are we currently running
 */

#ifndef __rh_config_h__
#define __rh_config_h__

/*
 * First, get the version string for the running kernel from
 * /boot/kernel.h - initscripts should create it for us
 */

#ifndef SKIP_BOOT_KERNEL_H
#include "/boot/kernel.h"
#endif

#if defined(__BOOT_KERNEL_SMP) && (__BOOT_KERNEL_SMP == 1)
#define __module__smp
#endif /* __BOOT_KERNEL_SMP */

#if defined(__BOOT_KERNEL_BOOT) && (__BOOT_KERNEL_BOOT == 1)
#define __module__BOOT
#endif /* __BOOT_KERNEL_BOOT */

#if defined(__BOOT_KERNEL_BOOTSMP) && (__BOOT_KERNEL_BOOTSMP == 1)
#define __module__BOOTsmp
#endif /* __BOOT_KERNEL_BOOTSMP */

#if defined(__BOOT_KERNEL_ENTERPRISE) && (__BOOT_KERNEL_ENTERPRISE == 1)
#define __module__enterprise
#endif /* __BOOT_KERNEL_ENTERPRISE */

#if !defined(__module__smp) && !defined(__module__BOOT) && !defined(__module__BOOTsmp) && !defined(__module__enterprise)
#define __module__up
#endif /* default (BOOT_KERNEL_UP) */

#ifdef __i386__
# ifdef __MODULE_KERNEL_i586
#  define __module__i586
#  ifdef __module__up
#   define __module__i586_up
#  endif
#  ifdef __module__smp
#   define __module__i586_smp
#  endif
#  ifdef __module__BOOT
#   define __module__i586_BOOT
#  endif
#  ifdef __module__BOOTsmp
#   define __module__i586_BOOTsmp
#  endif
#  ifdef __module__enterprise
#   define __module__i586_enterprise
#  endif
# elif defined(__MODULE_KERNEL_i686)
#  define __module__i686
#  ifdef __module__up
#   define __module__i686_up
#  endif
#  ifdef __module__smp
#   define __module__i686_smp
#  endif
#  ifdef __module__BOOT
#   define __module__i686_BOOT
#  endif
#  ifdef __module__BOOTsmp
#   define __module__i686_BOOTsmp
#  endif
#  ifdef __module__enterprise
#   define __module__i686_enterprise
#  endif
# elif defined(__MODULE_KERNEL_athlon)
#  define __module__athlon
#  ifdef __module__up
#   define __module__athlon_up
#  endif
#  ifdef __module__smp
#   define __module__athlon_smp
#  endif
#  ifdef __module__BOOT
#   define __module__athlon_BOOT
#  endif
#  ifdef __module__BOOTsmp
#   define __module__athlon_BOOTsmp
#  endif
#  ifdef __module__enterprise
#   define __module__athlon_enterprise
#  endif
# else
#  define __module__i386
#  ifdef __module__up
#   define __module__i386_up
#  endif
#  ifdef __module__smp
#   define __module__i386_smp
#  endif
#  ifdef __module__BOOT
#   define __module__i386_BOOT
#  endif
#  ifdef __module__BOOTsmp
#   define __module__i386_BOOTsmp
#  endif
#  ifdef __module__enterprise
#   define __module__i386_enterprise
#  endif
# endif
#endif

#ifdef __sparc__
# ifdef __arch64__
#  define __module__sparc64
#  ifdef __module__up
#   define __module__sparc64_up
#  endif
#  ifdef __module__smp
#   define __module__sparc64_smp
#  endif
#  ifdef __module__BOOT
#   define __module__sparc64_BOOT
#  endif
#  ifdef __module__BOOTsmp
#   define __module__sparc64_BOOTsmp
#  endif
#  ifdef __module__enterprise
#   define __module__sparc64_enterprise
#  endif
# else
#  define __module__sparc
#  ifdef __module__up
#   define __module__sparc_up
#  endif
#  ifdef __module__smp
#   define __module__sparc_smp
#  endif
#  ifdef __module__BOOT
#   define __module__sparc_BOOT
#  endif
#  ifdef __module__BOOTsmp
#   define __module__sparc_BOOTsmp
#  endif
#  ifdef __module__enterprise
#   define __module__sparc_enterprise
#  endif
# endif
#endif

#ifdef __alpha__
# define __module__alpha
# ifdef __module__up
#  define __module__alpha_up
# endif
# ifdef __module__smp
#  define __module__alpha_smp
# endif
# ifdef __module__BOOT
#  define __module__alpha_BOOT
# endif
# ifdef __module__BOOTsmp
#  define __module__alpha_BOOTsmp
# endif
# ifdef __module__enterprise
#  define __module__alpha_enterprise
# endif
#endif

#ifdef __ia64__
# define __module__ia64
# ifdef __module__up
#  define __module__ia64_up
# endif
# ifdef __module__smp
#  define __module__ia64_smp
# endif
# ifdef __module__BOOT
#  define __module__ia64_BOOT
# endif
# ifdef __module__BOOTsmp
#  define __module__ia64_BOOTsmp
# endif
# ifdef __module__enterprise
#  define __module__ia64_enterprise
# endif
#endif

#if defined(__module__smp) || defined(__module__BOOTsmp) || defined(__module__enterprise)
#define _ver_str(x) smp_ ## x
#else
#define _ver_str(x) x
#endif

#endif /* __rh_config_h__ */
