
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
#include <linux/lp.h>
#include <linux/spinlock.h>
#include <linux/pagemap.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#define DEBUG
#include <linux/usb.h>

#include <linux/sched/signal.h>
#include "usb-debug.h"


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
/* before 2.6.7, <linux/usb.h> contained its own wait_ms() delay function */
#define msleep(x)	wait_ms(x)
#endif

#include "sgil1.h"

#define SGIL1_MAX		40
#define SGIL1_MINOR_START	(256 - (16*3))

#define SGIL1_ST_MINOR		249

#define SGIL1_MAX_SIZE		4096
#define SGIL1_WR_TIMEOUT	(3 * HZ)

#define SGIL1_IDLE		0
#define SGIL1_BUSY		1
#define SGIL1_DONE		2
#define SGIL1_ERROR		3
#define SGIL1_CLOSED		4

#define SGIL1_LOCK_RD		1
#define SGIL1_LOCK_WR		2
#define SGIL1_LOCK_ALL		(SGIL1_LOCK_RD|SGIL1_LOCK_WR)

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

/* prevent races between open() and disconnect() */
static DEFINE_SEMAPHORE (disconnect_sem);


/*
 * local functions
 */
static int sgil1_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void sgil1_disconnect(struct usb_interface *interface);


/****************************************************************************
 * Connection status device driver
 ****************************************************************************/

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
	sgil1_st_state_t *st;

	if (!(st = kvmalloc(sizeof(*st), GFP_KERNEL)))
		return -ENOMEM;

	memset(st, 0, sizeof(*st));
	st->next = sgil1_st_state_list;
	if (st->next)
		st->next->prev = st;
	sgil1_st_state_list = st;
	
	++st->active;
	file->private_data = st;

	init_waitqueue_head(&st->status_wait);

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

	kvfree(st);

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

/* static int sgil1_st_ioctl(struct inode *inode, struct file *file, */
static long sgil1_st_ioctl(struct file *file,
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
		msleep(500);

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


/****************************************************************************
 * Actual (data transfer) device driver
 ****************************************************************************/


static void sgil1_lock(sgil1_t *sgil1, int which, char *who)
{
	if (which & SGIL1_LOCK_RD)
		down(&(sgil1->rd_sem));
	if (which & SGIL1_LOCK_WR)
		down(&(sgil1->wr_sem));
}

static void sgil1_unlock(sgil1_t *sgil1, int which, char *who)
{
	if (which & SGIL1_LOCK_RD)
		up(&(sgil1->rd_sem));
	if (which & SGIL1_LOCK_WR)
		up(&(sgil1->wr_sem));
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

static void sgil1_delete(sgil1_t *sgil1)
{
	if (sgil1->rd_buf) {
		usb_free_coherent(sgil1->dev, sgil1->rd_size,
				sgil1->rd_buf, sgil1->rd_urb->transfer_dma);
	}

	if (sgil1->wr_buf) {
		usb_free_coherent(sgil1->dev, sgil1->wr_size,
				sgil1->wr_buf, sgil1->wr_urb->transfer_dma);
	}

	if (sgil1->rd_urb) {
		usb_free_urb(sgil1->rd_urb);
		sgil1->rd_urb = NULL;
	}

	if (sgil1->wr_urb) {
		usb_free_urb(sgil1->wr_urb);
		sgil1->wr_urb = NULL;
	}

	sgil1->dev = 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static void sgil1_read_irq(struct urb *urb, struct pt_regs *regs)
#else
static void sgil1_read_irq(struct urb *urb)
#endif
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static void sgil1_write_irq(struct urb *urb, struct pt_regs *regs)
#else
static void sgil1_write_irq(struct urb *urb)
#endif
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
	int endp = usb_pipeendpoint(pipe);
	int rvalue;

	if (usb_pipein(pipe))
		endp |= USB_DIR_IN;

	rvalue = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				 USB_REQ_SET_FEATURE, USB_RECIP_ENDPOINT, 0,
				 endp, NULL, 0, HZ * 3);

	if (stall)
		return rvalue;

	msleep(20);

	return((rvalue) ? rvalue : usb_clear_halt(dev, pipe));
}

static int sgil1_open(struct inode *inode, struct file *file)
{
        int index = MINOR(inode->i_rdev) - SGIL1_MINOR_START;
	int rval;
	sgil1_t *sgil1;

	if ((index < 0) || (index >= SGIL1_MAX)) {
		return -EINVAL;
	}

        /* prevent disconnects */
        down(&disconnect_sem);

	sgil1 = &sgil1_state[index];

	sgil1_lock(sgil1, SGIL1_LOCK_ALL, "open");

	if (sgil1->active) {
		rval = -EBUSY;
		goto error;
	}

	if (!sgil1->dev) {
		rval = -ENODEV;
		goto error;
	}

	++sgil1->active;

	file->private_data = sgil1;
	sgil1->rd_state = SGIL1_IDLE;
	sgil1->rd_error = 0;
	sgil1->wr_state = SGIL1_IDLE;
	sgil1->wr_error = 0;
	init_waitqueue_head(&sgil1->rd_wait);
	init_waitqueue_head(&sgil1->wr_wait);

	sgil1_unlock(sgil1, SGIL1_LOCK_ALL, "open");

	dbg("sgil1_open, minor: %d index %d", sgil1->minor, index);
	up(&disconnect_sem);
	return 0;

error:
	sgil1_delete(sgil1);
	sgil1_unlock(sgil1, SGIL1_LOCK_ALL, "open err");
	up(&disconnect_sem);
	return rval;
}

static int sgil1_release(struct inode * inode, struct file * file)
{
	sgil1_t *sgil1 = file->private_data;

	if (!sgil1 || !sgil1->active)
		return -EINVAL;

	file->private_data = NULL;

	sgil1_lock(sgil1, SGIL1_LOCK_ALL, "release");

	if (!sgil1->active) {
		sgil1_unlock(sgil1, SGIL1_LOCK_ALL, "release notopen");
		return -EINVAL;
	}
	/* if the device is still there, stall the endpoint */
	if (sgil1->dev) {
		int rvalue;

		dbg("sgil1_release: reset device, minor: %d", sgil1->minor);
		if ((rvalue = usb_reset_device(sgil1->dev)) != 0) {
			err("SGI L1 device reset failed on close, minor: %d, error: %d",
				sgil1->minor, rvalue);
		}
	}

	/* stop all transactions in progress */
	sgil1_unlink_all(sgil1);

	/* if the device is disconnected, clean up (delete) URBs */
	if (!sgil1->dev)
		sgil1_delete(sgil1);

	sgil1->active = 0;

	sgil1_unlock(sgil1, SGIL1_LOCK_ALL, "release done");

	dbg("sgil1_close, minor: %d", sgil1->minor);

	return 0;
}

static ssize_t sgil1_write(struct file *file,
       const char *buffer, size_t count, loff_t *ppos)
{
	sgil1_t *sgil1 = file->private_data;
	int rval = 0;
	int error;

	if (!sgil1 || !sgil1->active || (count > sgil1->wr_size) || (count < 2))
		return -EINVAL;

	sgil1_lock(sgil1, SGIL1_LOCK_WR, "write");

	if (!sgil1->dev) {
		rval = -ENODEV;
		goto exit;
	}

	if (sgil1->wr_state == SGIL1_BUSY) {
		DECLARE_WAITQUEUE(wait, current);
		int timeout = SGIL1_WR_TIMEOUT;

		set_current_state(TASK_INTERRUPTIBLE);
		add_wait_queue(&sgil1->wr_wait, &wait);

		while (timeout && (sgil1->wr_state == SGIL1_BUSY)) {
			if (signal_pending(current)) {
				remove_wait_queue(&sgil1->wr_wait, &wait);
				set_current_state(TASK_RUNNING);
				rval = -ERESTARTSYS;
				goto exit;
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
			rval = -ETIMEDOUT;
			goto exit;

		case SGIL1_ERROR:
			dbg("last write error: %d minor: %d", sgil1->wr_error,
				sgil1->minor);
			rval = sgil1->wr_error;
			goto exit;

		default:
			rval = -ENODEV;
			goto exit;
	}

	if (!sgil1->dev) {
		rval = -ENODEV;
		goto exit;
	}

	if (copy_from_user(sgil1->wr_buf, buffer, count)) {
		rval = -EFAULT;
		goto exit;
	}

        *((unsigned short *)(sgil1->wr_buf)) = htons((unsigned short) count);
	usb_fill_bulk_urb(sgil1->wr_urb, sgil1->dev, sgil1->wr_pipe,
                        sgil1->wr_buf, count, sgil1_write_irq, sgil1);
	sgil1->wr_count = count;
	sgil1->wr_error = 0;
	sgil1->wr_state = SGIL1_BUSY;
        error = usb_submit_urb(sgil1->wr_urb, GFP_KERNEL);

	if (error) {
		sgil1->wr_state = SGIL1_IDLE;
		dbg("write error: %d minor: %d", error, sgil1->minor);
		sgil1_unlock(sgil1, SGIL1_LOCK_WR, "write err");
		rval = error;
		/* fall through to exit */
	}

exit:
	sgil1_unlock(sgil1, SGIL1_LOCK_WR, "write done");

	return((rval) ? rval : count);
}

static ssize_t sgil1_read(struct file *file,
       char *buffer, size_t count, loff_t *ppos)
{
	sgil1_t *sgil1 = file->private_data;
	int read_count;
	int rval = 0;

	if (!sgil1 || !sgil1->active)
		return -EINVAL;

	sgil1_lock(sgil1, SGIL1_LOCK_RD, "read");

	if (!sgil1->dev) {
		rval = -ENODEV;
		goto exit;
	}

	if (sgil1->rd_state != SGIL1_DONE) {
		dbg("read not done,  minor: %d state: %d",
			sgil1->minor, sgil1->rd_state);
		rval = 0;
		goto exit;
	}

	if (sgil1->rd_error) {
		int rderr = sgil1->rd_error;
		dbg("read error: %d minor: %d", sgil1->rd_error, sgil1->minor);
		sgil1->rd_state = SGIL1_IDLE;
		sgil1->rd_error = 0;
		rval = rderr;
		goto exit;
	}
	if ((read_count = sgil1->rd_count) > 0) {
		if (sgil1->rd_count > count) {
			rval = -EINVAL;
			goto exit;
		}
		rval = read_count;

		if (copy_to_user(buffer, sgil1->rd_buf, read_count)) {
			rval = -EFAULT;
			goto exit;
		}
	}

	sgil1->rd_state = SGIL1_IDLE;

exit:
	sgil1_unlock(sgil1, SGIL1_LOCK_RD, "read");

	return(rval);
}


static unsigned int sgil1_poll(struct file *file, poll_table *wait)
{
	sgil1_t *sgil1 = file->private_data;
	int error;

	if (!sgil1 || !sgil1->active)
		return POLLHUP;

	sgil1_lock(sgil1, SGIL1_LOCK_RD, "poll");

	if (!sgil1->dev) {
		sgil1_unlock(sgil1, SGIL1_LOCK_RD, "poll nodev");
		return POLLHUP;
	}

	if (sgil1->rd_state == SGIL1_IDLE) {
		usb_fill_bulk_urb(sgil1->rd_urb, sgil1->dev, sgil1->rd_pipe,
			sgil1->rd_buf, sgil1->rd_size, sgil1_read_irq, sgil1);
		sgil1->rd_state = SGIL1_BUSY;

		sgil1->rd_urb->status = 0;
		if ((error = usb_submit_urb(sgil1->rd_urb, GFP_KERNEL))) {
			sgil1->rd_state = SGIL1_IDLE;
			sgil1->rd_error = error;
			dbg("sgil1_poll: usb_submit_urb (read) error: %d minor: %d",
				sgil1->rd_error, sgil1->minor);
			sgil1_unlock(sgil1, SGIL1_LOCK_RD, "poll submit fail");
			return POLLIN | POLLRDNORM;
		}
	}

	sgil1_unlock(sgil1, SGIL1_LOCK_RD, "poll submit OK");

	poll_wait(file, &sgil1->rd_wait, wait);

	if (!sgil1->dev || !sgil1->active)
		return POLLHUP;

	if (sgil1->rd_state == SGIL1_DONE) {
		return POLLIN | POLLRDNORM;
	}

	return 0;
}

/* static int sgil1_ioctl(struct inode *inode, struct file *file, */
static long sgil1_ioctl(struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	sgil1_t *sgil1 = file->private_data;
	int rvalue;

	if (!sgil1 || !sgil1->active)
		return -EINVAL;

	sgil1_lock(sgil1, SGIL1_LOCK_ALL, "ioctl");

        if (!sgil1->dev) {
		sgil1_unlock(sgil1, SGIL1_LOCK_ALL, "ioctl nodev");
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

	sgil1_unlock(sgil1, SGIL1_LOCK_ALL, "ioctl done");

	if (rvalue)
		msleep(500);

	return rvalue;
}

/****************************************************************************
 * Support (probe/disconnect/init) routines
 ****************************************************************************/

/*
 * USB core driver structures
 */

static struct file_operations sgil1_fops = {
	.owner =	THIS_MODULE,
	.read =		sgil1_read,
	.write =	sgil1_write,
	.poll =		sgil1_poll,
	/* .ioctl =	sgil1_ioctl, */
	.unlocked_ioctl =	sgil1_ioctl,
	.open =		sgil1_open,
	.release =	sgil1_release
};

static struct usb_device_id sgil1_table [] = {
	{ USB_DEVICE(SGIL1_VENDOR_ID, SGIL1_PRODUCT_ID) },
	{ }			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, sgil1_table);

static struct usb_class_driver sgil1_class = {
	.name =		"usb/sgil1_%d",
	.fops =		&sgil1_fops,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
	.mode =		S_IFCHR|S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH,
#endif
	.minor_base =	SGIL1_MINOR_START,
};

static struct usb_driver sgil1_driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
	.owner =	THIS_MODULE,
#endif
	.name =		"sgil1",
	.probe =	sgil1_probe,
	.disconnect =	sgil1_disconnect,
	.id_table =	sgil1_table
};

/*
 * Driver for the SGI L1 connection status pseudo-device
 */

static struct file_operations sgil1_st_fops = {
	.owner =	THIS_MODULE,
	.read =		sgil1_st_read,
	.poll =		sgil1_st_poll,
	/* .ioctl =	sgil1_st_ioctl, */
	.unlocked_ioctl =	sgil1_st_ioctl,
	.open =		sgil1_st_open,
	.release =	sgil1_st_release
};

static struct miscdevice sgil1_st_dev = {
	.minor =	SGIL1_ST_MINOR,
	.name =		"sgil1_cs",
	.fops =		&sgil1_st_fops
};



static int sgil1_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_device *dev = interface_to_usbdev(interface);
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	sgil1_t sgil1_tmp;
	sgil1_t *sgil1 = &sgil1_tmp;
	sgil1_cfg_t *cfg;
	struct usb_device *cdev;
	struct usb_device *pdev;
	int rval = -ENOMEM;

	/*
	 * Determine if this device is an SGI L1 controller
	 */
	if ((dev->descriptor.idVendor != 0x065E) ||
	    (dev->descriptor.idProduct != 0x1234) ||
	    (dev->descriptor.bNumConfigurations != 1) ||
	    (dev->actconfig->desc.bNumInterfaces != 1))
		return -ENODEV;

	iface_desc = &interface->altsetting[0];

	if (iface_desc->desc.bNumEndpoints != 2)
		return -ENODEV;

	endpoint = &iface_desc->endpoint[0].desc;

	if (((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
	     != USB_ENDPOINT_XFER_BULK) ||
	    ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
	     != USB_DIR_OUT))
		return -ENODEV;

	endpoint = &iface_desc->endpoint[1].desc;

	if (((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
	     != USB_ENDPOINT_XFER_BULK) ||
	    ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
	     != USB_DIR_IN))
		return -ENODEV;

	/*
	 * Okay, this looks like a real L1 controller....
	 * use a temporary state structure, since we can't determine
	 * the real state slot until after the minor# is assigned by
	 * usb_register_dev()
	 */

	memset(sgil1, 0, sizeof(*sgil1));

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
			/* if (pdev->children[port] == cdev) */
			if (usb_hub_find_child(pdev,port) == cdev)
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

	if (!(sgil1->rd_urb = usb_alloc_urb(0, GFP_KERNEL)))
		return -ENOMEM;

	if (!(sgil1->wr_urb = usb_alloc_urb(0, GFP_KERNEL))) {
		usb_free_urb(sgil1->rd_urb);
		sgil1->rd_urb = NULL;
		return -ENOMEM;
	}

	sgil1->rd_urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
	sgil1->wr_urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;

	/* NOTE: semaphores (rd_sem, wr_sem) are initialized below, AFTER the
	   state data is copied from the temp space to the global table */

	sgil1->dev = dev;

	sgil1->rd_size = (SGIL1_MAX_SIZE > PAGE_SIZE) ?
						PAGE_SIZE : SGIL1_MAX_SIZE;
	sgil1->wr_size = (SGIL1_MAX_SIZE > PAGE_SIZE) ?
						PAGE_SIZE : SGIL1_MAX_SIZE;

	sgil1->rd_endpoint  = iface_desc->endpoint[1].desc.bEndpointAddress &
				     USB_ENDPOINT_NUMBER_MASK;
	sgil1->wr_endpoint = iface_desc->endpoint[0].desc.bEndpointAddress &
				     USB_ENDPOINT_NUMBER_MASK;

	sgil1->rd_pipe = usb_rcvbulkpipe(sgil1->dev, sgil1->rd_endpoint);
	sgil1->wr_pipe = usb_sndbulkpipe(sgil1->dev, sgil1->wr_endpoint);

	if (!(sgil1->rd_buf = usb_alloc_coherent(sgil1->dev, sgil1->rd_size,
				GFP_KERNEL, &sgil1->rd_urb->transfer_dma))) {
		sgil1->active = 0;
		rval = -ENOMEM;
		goto error;
	}

	if (!(sgil1->wr_buf = usb_alloc_coherent(sgil1->dev, sgil1->wr_size,
				GFP_KERNEL, &sgil1->wr_urb->transfer_dma))) {
		sgil1->active = 0;
		rval = -ENOMEM;
		goto error;
	}

	/* register the driver (claim a minor device#) */
	if ((rval = usb_register_dev(interface, &sgil1_class)) != 0) {
		err ("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* we now know the minor# for this device, so we can find the
	   correct state structure slot */
	sgil1->minor = interface->minor;
	memcpy(&sgil1_state[sgil1->minor - SGIL1_MINOR_START], sgil1, sizeof(*sgil1));

	/* point the the global copy of the interface data */
	sgil1 = &sgil1_state[sgil1->minor - SGIL1_MINOR_START];
	usb_set_intfdata(interface, sgil1);

	/* the semaphores MUST be initialized *AFTER* copying the dev state
	   data from the temporary buffer to the new one; for some reason,
	   initializing the semaphores and then copying them to a new address
	   causes kernel crashes when locking/unlocking */
	sema_init(&sgil1->rd_sem, 1);
	sema_init(&sgil1->wr_sem, 1);

	info("SGI L1 connected, minor: %d device: %d.%d",
		sgil1->minor, dev->bus->busnum, dev->devnum);

	sgil1_st_update();

	return 0;

error:	
	sgil1_delete(sgil1);

	return rval;
}

static void sgil1_disconnect(struct usb_interface *interface)
{
	struct usb_device *dev = interface_to_usbdev(interface);
	sgil1_t *sgil1;

	/* prevent races with open() */
	down (&disconnect_sem);

	sgil1 = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	if (sgil1) {

		if (!sgil1->dev) {
			err("SGI L1 already disconnected, minor: %d",
				sgil1->minor);
			goto error;
		}

		sgil1_lock(sgil1, SGIL1_LOCK_ALL, "disconnect");

		/* give back the minor# */
		usb_deregister_dev(interface, &sgil1_class);

		info("SGI L1 disconnected, minor: %d device: %d.%d",
			sgil1->minor, dev->bus->busnum, dev->devnum);

		/* stop all transactions in progress */
		sgil1_unlink_all(sgil1);

		/* clean up (delete) URBs now if device is not open,
		   if still open clear out the device handle to tell the
		   release() function to cleanup */
		if (!sgil1->active)
			sgil1_delete(sgil1);
		else
			sgil1->dev = 0;

		sgil1_unlock(sgil1, SGIL1_LOCK_ALL, "disconnect");

		sgil1_st_update();
	}

error:
	up(&disconnect_sem);
}


static int __init sgil1_init(void)
{
	int rval;

	/* register the connection status device: this is not associated
	   with an actual hardware device, instead it allows apps to monitor
	   for new SGI L1 USB device connections */
	if ((rval = misc_register(&sgil1_st_dev)) != 0) {
		err("misc_register failed, error %d", rval);
		return rval;
	}

	/* register the handler for actual USB device drivers */
	if ((rval = usb_register(&sgil1_driver)) != 0) {
		err("usb_register failed, error %d", rval);
		misc_deregister(&sgil1_st_dev);
		return rval;
	}

	info(DRIVER_VERSION ":" DRIVER_DESC);

	return 0;
}

static void __exit sgil1_exit(void)
{
	misc_deregister(&sgil1_st_dev);
	usb_deregister(&sgil1_driver);
}

module_init(sgil1_init);
module_exit(sgil1_exit);

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE( "GPL" );
MODULE_ALIAS_MISCDEV( SGIL1_ST_MINOR );
