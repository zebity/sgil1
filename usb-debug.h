
#ifndef __USB_DEBUG_H__

#ifdef DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) printk(KERN_ERR __FILE__ ": " format "\n" , ## arg)
#define info(format, arg...) printk(KERN_INFO __FILE__ ": " format "\n" , ## arg)
#define warn(format, arg...) printk(KERN_WARNING __FILE__ ": " format "\n" , ## arg)

#define __USB_DEBUG_H__

#endif
