
/* Driver for SGI L1 controller
 * 
 * Copyright 1999 Bob Cutler (rwc@sgi.com)
 * Copyright 2000 Steve Hein (ssh@sgi.com)
 *
 * Distribute under GPL version 2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/random.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/lp.h>
#include <linux/spinlock.h>
#include <asm/semaphore.h>
#define DEBUG
#include <linux/usb.h>

#include "sgil1.h"
#define NEW_SGIL1

#define SGIL1_MAX		40
#define SGIL1_MINOR_START	(256 - (16*3))

#define SGIL1_MAX_SIZE		4096
#define SGIL1_WR_TIMEOUT	(3 * HZ)

#define SGIL1_IDLE		0
#define SGIL1_BUSY		1
#define SGIL1_DONE		2
#define SGIL1_ERROR		3
#define SGIL1_CLOSED		4

typedef struct {
	struct usb_device	*dev;
	sgil1_cfg_t		cfg;
	__u8			active;
	__u8			minor;
	
	unsigned int		rd_endpoint;
	unsigned int		rd_pipe;
	char			*rd_buf;
	int			rd_size;
	int			rd_count;
	int			rd_error;
	int			rd_state;
	wait_queue_head_t	rd_wait;
	struct semaphore	rd_sem;
	struct urb		*rd_urb;

	unsigned int		wr_endpoint;
	unsigned int		wr_pipe;
	char			*wr_buf;
	int			wr_size;
	int			wr_count;
        int                     wr_error;
	int			wr_state;
	wait_queue_head_t	wr_wait;
	struct semaphore	wr_sem;
	struct urb		*wr_urb;
} sgil1_t;

static sgil1_t sgil1_state[SGIL1_MAX];

static char *sgil1_rev = DRIVER_VERSION;


/*
 * Connection status device driver
 */

static int sgil1_st_open(struct inode * inode, struct file * file);

static int sgil1_st_release(struct inode * inode, struct file * file);

static ssize_t sgil1_st_read(struct file *file,
	char *buffer, size_t count, loff_t *ppos);

static unsigned int sgil1_st_poll(struct file *file, poll_table *wait);

static int sgil1_st_ioctl(struct inode *inode, struct file *file,
		          unsigned int cmd, unsigned long arg);

static struct file_operations sgil1_st_fops = {
//	owner:		THIS_MODULE,
	read:		sgil1_st_read,
	poll:		sgil1_st_poll,
	ioctl:		sgil1_st_ioctl,
	open:		sgil1_st_open,
	release:	sgil1_st_release
};

typedef struct _sgil1_st_state_t {
	__u8			active;
	__u8			new_status;
	wait_queue_head_t	status_wait;

	__u8			status[SGIL1_MAX];

	struct _sgil1_st_state_t *prev;
	struct _sgil1_st_state_t *next;
} sgil1_st_state_t;

static sgil1_st_state_t *sgil1_st_state_list = NULL;


static int sgil1_st_open(struct inode * inode, struct file * file)
{
        int index = MINOR(inode->i_rdev) - SGIL1_MINOR_START;
	sgil1_st_state_t *st;

	if (index != SGIL1_MAX)
		return -ENODEV;

	if (!(st = kmalloc(sizeof(*st), GFP_KERNEL)))
		return -ENOMEM;

	memset(st, 0, sizeof(*st));
	st->next = sgil1_st_state_list;
	if (st->next)
		st->next->prev = st;
	sgil1_st_state_list = st;
	
	++st->active;
	file->f_op = &sgil1_st_fops;
	file->private_data = st;

	init_waitqueue_head(&st->status_wait);

	MOD_INC_USE_COUNT;
	return 0;
}

static int sgil1_st_release(struct inode * inode, struct file * file)
{
	sgil1_st_state_t *st = (sgil1_st_state_t*) file->private_data;

	if (!st  ||  !st->active)
		return -EINVAL;

	if (st->prev)
		st->prev->next = st->next;
	if (st->next)
		st->next->prev = st->prev;
	if (sgil1_st_state_list == st)
		sgil1_st_state_list = st->next;

	kfree(st);

	MOD_DEC_USE_COUNT;
	return 0;
}

static ssize_t sgil1_st_read(struct file *file,
       char *buffer, size_t count, loff_t *ppos)
{
	sgil1_st_state_t *st = (sgil1_st_state_t*) file->private_data;

	int dmax;
	int i;

	if (!st  ||  !st->active)
		return -EINVAL;

	dmax = (count < SGIL1_MAX) ? count : SGIL1_MAX;

	for (i = 0; i < dmax; i++)
		st->status[i] = sgil1_state[i].dev ? 1 : 0;

	if (copy_to_user(buffer, st->status, dmax))
		return -EFAULT;

	st->new_status = 0;

	return(dmax);
}

static unsigned int sgil1_st_poll(struct file *file, poll_table *wait)
{
	sgil1_st_state_t *st = (sgil1_st_state_t*) file->private_data;

	if (!st  ||  !st->active)
		return 0;

	poll_wait(file, &st->status_wait, wait);

	if (st->new_status)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int sgil1_st_ioctl(struct inode *inode, struct file *file,
		          unsigned int cmd, unsigned long arg)
{
	sgil1_st_state_t *st = (sgil1_st_state_t*) file->private_data;
	sgil1_cfg_t cfg;
	sgil1_t *devst;

	int rvalue;

	if (!st  ||  !st->active)
		return -EINVAL;

	switch (cmd) {

		case SGIL1_ST_READ_REV:
			rvalue = copy_to_user((void *) arg, sgil1_rev,
						sizeof(sgil1_rev));
			break;

		case SGIL1_ST_READ_DEV_CFG:

			if (copy_from_user(&cfg, (void*) arg, sizeof(cfg)))
				return -EFAULT;

			if (cfg.dev >= SGIL1_MAX  )
				return -EINVAL;

			devst = &sgil1_state[cfg.dev];
			if (!devst->dev)
				return -EINVAL;

			rvalue = copy_to_user((void *) arg, &devst->cfg,
						sizeof(devst->cfg));
			break;

		default:
			return -ENOIOCTLCMD;
	}

	if (rvalue)
		wait_ms(500);

	return rvalue;
}

static void sgil1_st_update(void)
{
	sgil1_st_state_t *st;

	for (st = sgil1_st_state_list; st; st = st->next) {
		st->new_status = 1;
		if (st->active)
			wake_up_interruptible(&st->status_wait);
	}
}


/*
 * Actual device driver
 */

static int sgil1_find_index(void)
{
	int i;

	for (i = 0; i < SGIL1_MAX; i++) {
		if (!sgil1_state[i].dev && !sgil1_state[i].active)
			return (i);
	}

	return (-1);
}

static void sgil1_lock(sgil1_t *sgil1)
{
	down(&sgil1->rd_sem);
	down(&sgil1->wr_sem);
}

static void sgil1_unlock(sgil1_t *sgil1)
{
        up(&sgil1->rd_sem);
        up(&sgil1->wr_sem);
}

static void sgil1_unlink_all(sgil1_t *sgil1)
{
	int state;

	state = sgil1->rd_state;
	sgil1->rd_state = SGIL1_CLOSED;
	sgil1->rd_error = -ENODEV;

	if (state == SGIL1_BUSY) {
		usb_unlink_urb(sgil1->rd_urb);
		wake_up_interruptible(&sgil1->rd_wait);
	}

	state = sgil1->wr_state;
	sgil1->wr_state = SGIL1_CLOSED;
	sgil1->wr_error = -ENODEV;

	if (state == SGIL1_BUSY) {
		usb_unlink_urb(sgil1->wr_urb);
		wake_up_interruptible(&sgil1->wr_wait);
	}
}

static void sgil1_read_irq(struct urb *urb)
{
	sgil1_t *sgil1 = (sgil1_t *)(urb->context);

	sgil1->rd_count = urb->actual_length;
	sgil1->rd_error = urb->status;
	sgil1->rd_state = SGIL1_DONE;

	if (sgil1->rd_error)
	    dbg("sgil1_read_irq: read error: %d minor: %d",
		    sgil1->rd_error, sgil1->minor);

	wake_up_interruptible(&sgil1->rd_wait);
}

static void sgil1_write_irq(struct urb *urb)
{
	sgil1_t *sgil1 = (sgil1_t *)(urb->context);

	sgil1->wr_error = urb->status;

	if (urb->actual_length != sgil1->wr_count) {
		dbg("short write, minor: %d, act: %d, exp: %d", sgil1->minor,
			urb->actual_length, sgil1->wr_count);

		if (!sgil1->wr_error)
			sgil1->wr_error = -EIO;
	}

	sgil1->wr_state = sgil1->wr_error ? SGIL1_ERROR : SGIL1_DONE;
	wake_up_interruptible(&sgil1->wr_wait);
}

static int sgil1_reset_pipe(struct usb_device *dev, unsigned int pipe, int stall)
{
	int endp = usb_pipeendpoint(pipe) | (usb_pipein(pipe) << 7);
	int rvalue;

	rvalue = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				 USB_REQ_SET_FEATURE, USB_RECIP_ENDPOINT, 0,
				 endp, NULL, 0, HZ * 3);

	if (stall)
		return rvalue;

	wait_ms(20);

	return((rvalue) ? rvalue : usb_clear_halt(dev, pipe));
}

#ifdef NEW_SGIL1
#if 0
static int sgil1_reset_device(sgil1_t *sgil1, int stall, char *func, char *action)
{
	int rvalue, try;
	char *stallmsg = (stall) ? "stall" : "unstall";

	/*
	 * try to stall/unstall both pipes.  if either one fails, reset
	 * the device, then retry the stall/unstall
	 */
	for (try = 0; try < 2; try++) {

		rvalue = 0;

		dbg("%s: %s write pipe, minor: %d", func, stallmsg, sgil1->minor);
		if ((rvalue = sgil1_reset_pipe(sgil1->dev, sgil1->wr_pipe, stall)) < 0)
			err("SGI L1 %s write failed on %s, minor: %d, error: %d",
				stallmsg, action, sgil1->minor, rvalue);

		dbg("%s: %s read pipe, minor: %d", func, stallmsg, sgil1->minor);
		if (!rvalue && (rvalue = sgil1_reset_pipe(sgil1->dev, sgil1->rd_pipe, stall)) < 0)
			err("SGI L1 %s read failed on %s, minor: %d, error: %d",
				stallmsg, action, sgil1->minor, rvalue);

		if (rvalue) {
			dbg("%s: reset device after %s failure, minor: %d",
				func, stallmsg, sgil1->minor);
			if ((rvalue = usb_reset_device(sgil1->dev)) != 0) {
				err("SGI L1 device reset failed on %s, minor: %d, error: %d",
					action, sgil1->minor, rvalue);
			}
		} else
			break;

		/* if we have an ENODEV error, then leave now */
		if (rvalue == -ENODEV)
			return -ENODEV;
	}

	if (rvalue) {
		err("SGI L1 device reset (%s) failed after multiple tries, minor: %d", action, sgil1->minor);
		return rvalue;
	}

	return 0;
}
#endif
#endif   /* NEW_SGIL1 */

static int sgil1_open(struct inode *inode, struct file *file)
{
        int index = MINOR(inode->i_rdev) - SGIL1_MINOR_START;
	sgil1_t *sgil1;

	if ((index < 0) || (index > SGIL1_MAX)) {
		return -EINVAL;
	}

	if (index == SGIL1_MAX)
		return(sgil1_st_open(inode, file));

	sgil1 = &sgil1_state[index];

	sgil1_lock(sgil1);

	if (sgil1->active) {
		sgil1_unlock(sgil1);
		return -EBUSY;
	}

	if (!sgil1->dev) {
		sgil1_unlock(sgil1);
		return -ENODEV;
	}

	++sgil1->active;

	if (!(sgil1->rd_buf = (char *)__get_free_page(GFP_KERNEL))) {
		sgil1->active = 0;
		sgil1_unlock(sgil1);
		return -ENOMEM;
	}

	if (!(sgil1->wr_buf = (char *)__get_free_page(GFP_KERNEL))) {
		free_page((unsigned long)sgil1->rd_buf);
		sgil1->active = 0;
		sgil1_unlock(sgil1);
		return -ENOMEM;
	}

	memset(sgil1->rd_urb, 0, sizeof(struct urb));
	memset(sgil1->wr_urb, 0, sizeof(struct urb));

	file->private_data = sgil1;
	sgil1->rd_state = SGIL1_IDLE;
#ifdef NEW_SGIL1
	sgil1->rd_error = 0;
#endif
	sgil1->wr_state = SGIL1_IDLE;
#ifdef NEW_SGIL1
	sgil1->wr_error = 0;
#endif
	init_waitqueue_head(&sgil1->rd_wait);
	init_waitqueue_head(&sgil1->wr_wait);

#ifdef NEW_SGIL1
#if 0
	/* un-stall the endpoint to release the other end */
	dbg("sgil1_open: unstall, minor: %d", sgil1->minor);
	if ((rvalue = sgil1_reset_device(sgil1, 0, "sgil1_open", "open")) < 0) {
		free_page((unsigned long)sgil1->rd_buf);
		free_page((unsigned long)sgil1->wr_buf);
		sgil1->active = 0;
		sgil1_unlock(sgil1);
		return rvalue;
	}
#endif
#endif

	sgil1_unlock(sgil1);

	dbg("sgil1_open, minor: %d index %d", sgil1->minor, index);
	MOD_INC_USE_COUNT;
	return 0;
}

static int sgil1_release(struct inode * inode, struct file * file)
{
	sgil1_t *sgil1 = file->private_data;

	if (!sgil1 || !sgil1->active)
		return -EINVAL;

	file->private_data = NULL;

	sgil1_lock(sgil1);

	if (!sgil1->active) {
		sgil1_unlock(sgil1);
		return -EINVAL;
	}

#ifdef NEW_SGIL1
	/* if the device is still there, stall the endpoint */
	if (sgil1->dev) {
		int rvalue;

		dbg("sgil1_release: reset device, minor: %d", sgil1->minor);
		if ((rvalue = usb_reset_device(sgil1->dev)) != 0) {
			err("SGI L1 device reset failed on close, minor: %d, error: %d",
				sgil1->minor, rvalue);
		}
	}
#endif

	sgil1_unlink_all(sgil1);
	free_page((unsigned long)sgil1->rd_buf);
	free_page((unsigned long)sgil1->wr_buf);
	sgil1->active = 0;

	sgil1_unlock(sgil1);

	dbg("sgil1_close, minor: %d", sgil1->minor);
	MOD_DEC_USE_COUNT;
	return 0;
}

static ssize_t sgil1_write(struct file *file,
       const char *buffer, size_t count, loff_t *ppos)
{
	sgil1_t *sgil1 = file->private_data;
	int error;

	if (!sgil1 || !sgil1->active || (count > sgil1->wr_size) || (count < 2))
		return -EINVAL;

	if (sgil1->wr_state == SGIL1_BUSY) {
		DECLARE_WAITQUEUE(wait, current);
		int timeout = SGIL1_WR_TIMEOUT;

		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&sgil1->wr_wait, &wait);

		while (timeout && (sgil1->wr_state == SGIL1_BUSY)) {
			if (signal_pending(current)) {
				remove_wait_queue(&sgil1->wr_wait, &wait);
				set_current_state(TASK_RUNNING);
				return -ERESTARTSYS;
			}

			timeout = schedule_timeout(timeout);
		}

		remove_wait_queue(&sgil1->wr_wait, &wait);
		set_current_state(TASK_RUNNING);
	}

	switch (sgil1->wr_state) {
		case SGIL1_IDLE:
		case SGIL1_DONE:
			break;

		case SGIL1_BUSY:
			dbg("write timeout, minor: %d", sgil1->minor);
			usb_unlink_urb(sgil1->wr_urb);
			return -ETIMEDOUT;

		case SGIL1_ERROR:
			dbg("last write error: %d minor: %d", sgil1->wr_error,
				sgil1->minor);
			return(sgil1->wr_error);

		default:
			return -ENODEV;
	}

	down(&sgil1->wr_sem);

	if (!sgil1->dev) {
		up(&sgil1->wr_sem);
		return -ENODEV;
	}

	if (copy_from_user(sgil1->wr_buf, buffer, count)) {
		up(&sgil1->wr_sem);
		return -EFAULT;
	}

        *((unsigned short *)(sgil1->wr_buf)) = htons((unsigned short) count);
	FILL_BULK_URB(sgil1->wr_urb, sgil1->dev, sgil1->wr_pipe,
                        sgil1->wr_buf, count, sgil1_write_irq, sgil1);
	sgil1->wr_count = count;
	sgil1->wr_error = 0;
	sgil1->wr_state = SGIL1_BUSY;
        error = usb_submit_urb(sgil1->wr_urb);

	up(&sgil1->wr_sem);

	if (error) {
		sgil1->wr_state = SGIL1_IDLE;
		dbg("write error: %d minor: %d", error, sgil1->minor);
		return(error);
	}

	return(count);
}

static ssize_t sgil1_read(struct file *file,
       char *buffer, size_t count, loff_t *ppos)
{
	sgil1_t *sgil1 = file->private_data;
	int read_count;

	if (!sgil1 || !sgil1->active)
		return -EINVAL;

	if (!sgil1->dev)
		return -ENODEV;

	if (sgil1->rd_state != SGIL1_DONE) {
		dbg("read not done,  minor: %d state: %d",
			sgil1->minor, sgil1->rd_state);
		return 0;
	}

	if (sgil1->rd_error) {
		int rderr = sgil1->rd_error;
		dbg("read error: %d minor: %d", sgil1->rd_error, sgil1->minor);
		sgil1->rd_state = SGIL1_IDLE;
		sgil1->rd_error = 0;
		return(rderr);
	}

	if ((read_count = sgil1->rd_count) > 0) {
		if (sgil1->rd_count > count)
			return -EINVAL;

		if (copy_to_user(buffer, sgil1->rd_buf, read_count))
			return -EFAULT;
	}

	sgil1->rd_state = SGIL1_IDLE;

	return(read_count);
}

static unsigned int sgil1_poll(struct file *file, poll_table *wait)
{
	sgil1_t *sgil1 = file->private_data;
	int error;

	if (!sgil1 || !sgil1->active)
		return POLLHUP;

	down(&sgil1->rd_sem);

	if (!sgil1->dev) {
		up(&sgil1->rd_sem);
		return POLLHUP;
	}

	if (sgil1->rd_state == SGIL1_IDLE) {
		FILL_BULK_URB(sgil1->rd_urb, sgil1->dev, sgil1->rd_pipe,
			sgil1->rd_buf, sgil1->rd_size, sgil1_read_irq, sgil1);
		sgil1->rd_state = SGIL1_BUSY;

		if ((error = usb_submit_urb(sgil1->rd_urb))) {
			sgil1->rd_state = SGIL1_IDLE;
			sgil1->rd_error = error;
			dbg("sgil1_poll: usb_submit_urb (read) error: %d minor: %d",
				sgil1->rd_error, sgil1->minor);
			up(&sgil1->rd_sem);
			return POLLIN | POLLRDNORM;
		}
	}

	up(&sgil1->rd_sem);

	poll_wait(file, &sgil1->rd_wait, wait);

	if (!sgil1->dev || !sgil1->active)
		return POLLHUP;

	if (sgil1->rd_state == SGIL1_DONE)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int sgil1_ioctl(struct inode *inode, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	sgil1_t *sgil1 = file->private_data;
	int rvalue;

	if (!sgil1 || !sgil1->active)
		return -EINVAL;

	sgil1_lock(sgil1);

        if (!sgil1->dev) {
		sgil1_unlock(sgil1);
		return -ENODEV;
	}

	switch (cmd) {

		case SGIL1_RESET_PIPES:
		case SGIL1_RESET_WRITE:
			rvalue = sgil1_reset_pipe(sgil1->dev, sgil1->wr_pipe, 0);
			dbg("SGIL1_RESET_WRITE %d.%d, %d",
				sgil1->dev->bus->busnum, sgil1->dev->devnum,
				rvalue);
			if (cmd == SGIL1_RESET_WRITE)
				break;
			/* SGIL1_RESET_PIPES falls through.... */

		case SGIL1_RESET_READ:
			rvalue = sgil1_reset_pipe(sgil1->dev, sgil1->rd_pipe, 0);
			dbg("SGIL1_RESET_READ %d.%d, %d",
				sgil1->dev->bus->busnum, sgil1->dev->devnum,
				rvalue);
			break;

		case SGIL1_RESET_DEVICE:
			if ((rvalue = usb_reset_device(sgil1->dev)) != 0)
				sgil1->rd_state = SGIL1_ERROR;

			dbg("SGIL1_RESET_DEVICE %d.%d, %d",
				sgil1->dev->bus->busnum, sgil1->dev->devnum,
				rvalue);
			break;

		case SGIL1_READ_CFG:
			rvalue = copy_to_user((void *) arg, &sgil1->cfg,
						sizeof(sgil1_cfg_t));
			break;

		default:
			return -ENOIOCTLCMD;
	}

	sgil1_unlock(sgil1);

	if (rvalue)
		wait_ms(500);

	return rvalue;
}

#ifdef MODULE_DEVICE_TABLE
static void *sgil1_probe(struct usb_device *dev, unsigned int ifnum,
                         const struct usb_device_id *id)
#else
static void *sgil1_probe(struct usb_device *dev, unsigned int ifnum)
#endif
{
	struct usb_interface_descriptor *interface;
	struct usb_endpoint_descriptor *endpoint;
	sgil1_t *sgil1;
	sgil1_cfg_t *cfg;
	struct usb_device *cdev;
	struct usb_device *pdev;
	int index;

	/*
	 * Determine if this device is an SGI L1 controller
	 */
	if ((dev->descriptor.idVendor != 0x065E) ||
	    (dev->descriptor.idProduct != 0x1234) ||
	    (dev->descriptor.bNumConfigurations != 1) ||
	    (dev->actconfig->bNumInterfaces != 1))
		return NULL;

	if (ifnum != 0)
		return NULL;

	interface = &dev->actconfig->interface[ifnum].altsetting[0];

	if (interface->bNumEndpoints != 2)
		return NULL;

	endpoint = &interface->endpoint[0];

	if (((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
	     != USB_ENDPOINT_XFER_BULK) ||
	    ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
	     != USB_DIR_OUT))
		return NULL;

	endpoint = &interface->endpoint[1];

	if (((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
	     != USB_ENDPOINT_XFER_BULK) ||
	    ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
	     != USB_DIR_IN))
		return NULL;

	/*
	 * Find an inactive driver state structure for this device
	 */
	if ((index = sgil1_find_index()) < 0)
		return NULL;

	sgil1 = &sgil1_state[index];
	memset(sgil1, 0, sizeof(sgil1_t));

	/*
	 * Save the USB configuration for this device
	 */
	cfg = &sgil1->cfg;
	cfg->bus = dev->bus->busnum;
	cfg->dev = dev->devnum;
	cdev = dev;
	pdev = dev->parent;

	while (pdev && (cfg->level < SGIL1_MAX_LEVEL)) {
		int port;
		int i;

		for (port = 0; port < pdev->maxchild; port++)
			if (pdev->children[port] == cdev)
				break;

		if (port >= pdev->maxchild)
			port = -1;

		for (i = cfg->level; i > 0; i--)
			cfg->path[i] = cfg->path[i - 1];

		++cfg->level;
		cfg->path[0] = port + 1;
		cdev = pdev;
		pdev = pdev->parent;
	}

	/*
	 * Configure this device
	 */
	if (!(sgil1->rd_urb = usb_alloc_urb(0)))
		return NULL;

	if (!(sgil1->wr_urb = usb_alloc_urb(0))) {
		usb_free_urb(sgil1->rd_urb);
		sgil1->rd_urb = NULL;
		return NULL;
	}

	init_MUTEX(&sgil1->rd_sem);
	init_MUTEX(&sgil1->wr_sem);
	sgil1->dev = dev;
	sgil1->minor = SGIL1_MINOR_START + index;

	sgil1->rd_size = (SGIL1_MAX_SIZE > PAGE_SIZE) ?
						PAGE_SIZE : SGIL1_MAX_SIZE;
	sgil1->wr_size = (SGIL1_MAX_SIZE > PAGE_SIZE) ?
						PAGE_SIZE : SGIL1_MAX_SIZE;

	sgil1->rd_endpoint  = interface->endpoint[1].bEndpointAddress &
				     USB_ENDPOINT_NUMBER_MASK;
	sgil1->wr_endpoint = interface->endpoint[0].bEndpointAddress &
				     USB_ENDPOINT_NUMBER_MASK;

	sgil1->rd_pipe = usb_rcvbulkpipe(sgil1->dev, sgil1->rd_endpoint);
	sgil1->wr_pipe = usb_sndbulkpipe(sgil1->dev, sgil1->wr_endpoint);

#ifdef NEW_SGIL1
#if 0
	int rvalue;

	dbg("sgil1_probe: reset device, minor: %d", sgil1->minor);
	if ((rvalue = usb_reset_device(sgil1->dev)) != 0) {
		err("SGI L1 device reset failed on close, minor: %d, error: %d",
			sgil1->minor, rvalue);
	}

	/* stall the endpoint to hold the other end */
	dbg("sgil1_probe: stall, minor: %d", sgil1->minor);
	if ((rvalue = sgil1_reset_device(sgil1, 1, "sgil1_probe", "probe")) < 0) {
		usb_free_urb(sgil1->rd_urb);
		sgil1->rd_urb = NULL;
		usb_free_urb(sgil1->wr_urb);
		sgil1->wr_urb = NULL;
		return NULL;
	}
#endif
#endif

	info("SGI L1 connected, minor: %d device: %d.%d",
		sgil1->minor, dev->bus->busnum, dev->devnum);

	sgil1_st_update();

	return sgil1;
}

static void sgil1_disconnect(struct usb_device *dev, void *ptr)
{
	sgil1_t *sgil1 = (sgil1_t *) ptr;

	if (sgil1) {
		sgil1_lock(sgil1);

		if (!sgil1->dev) {
			err("SGI L1 already disconnected, minor: %d",
				sgil1->minor);
			sgil1_unlock(sgil1);
			return;
		}

		sgil1->dev = 0;

		info("SGI L1 disconnected, minor: %d device: %d.%d",
			sgil1->minor, dev->bus->busnum, dev->devnum);

		sgil1_unlink_all(sgil1);

		usb_free_urb(sgil1->rd_urb);
		sgil1->rd_urb = NULL;

		usb_free_urb(sgil1->wr_urb);
		sgil1->wr_urb = NULL;

		sgil1_unlock(sgil1);

		sgil1_st_update();
	}
}

static struct file_operations sgil1_fops = {
	read:		sgil1_read,
	write:		sgil1_write,
	poll:		sgil1_poll,
	ioctl:		sgil1_ioctl,
	open:		sgil1_open,
	release:	sgil1_release
};

#ifdef MODULE_DEVICE_TABLE
static struct usb_device_id sgil1_table [] = {
	{ USB_DEVICE(SGIL1_VENDOR_ID, SGIL1_PRODUCT_ID) },
	{ }			/* Terminating entry */
};
#endif

static struct usb_driver sgil1_driver_a = {
	//owner:		THIS_MODULE,
	name:		"sgil1",
	probe:		sgil1_probe,
	disconnect:	sgil1_disconnect,
	fops:		&sgil1_fops,
	minor:		SGIL1_MINOR_START + 0,
#ifdef MODULE_DEVICE_TABLE
	id_table:	sgil1_table
#endif
};

static struct usb_driver sgil1_driver_b = {
	//owner:		THIS_MODULE,
	name:           "sgil1",
	probe:          sgil1_probe,
	disconnect:     sgil1_disconnect,
	fops:           &sgil1_fops,
	minor:          SGIL1_MINOR_START + 16,
#ifdef MODULE_DEVICE_TABLE
	id_table:       sgil1_table
#endif
};

static struct usb_driver sgil1_driver_c = {
	//owner:		THIS_MODULE,
	name:           "sgil1",
	probe:          sgil1_probe,
	disconnect:     sgil1_disconnect,
	fops:           &sgil1_fops,
	minor:          SGIL1_MINOR_START + 32,
#ifdef MODULE_DEVICE_TABLE
	id_table:       sgil1_table
#endif
};

static int __init sgil1_init(void)
{
	if (usb_register(&sgil1_driver_a) < 0)
		return -1;

	if (usb_register(&sgil1_driver_b) < 0) {
	        usb_deregister(&sgil1_driver_a);
		return -1;
        }

	if (usb_register(&sgil1_driver_c) < 0) {
	        usb_deregister(&sgil1_driver_a);
	        usb_deregister(&sgil1_driver_b);
		return -1;
        }

	info(DRIVER_VERSION ":" DRIVER_DESC);

	return 0;
}

static void __exit sgil1_exit(void)
{
	usb_deregister(&sgil1_driver_a);
	usb_deregister(&sgil1_driver_b);
	usb_deregister(&sgil1_driver_c);
}

module_init(sgil1_init);
module_exit(sgil1_exit);

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
