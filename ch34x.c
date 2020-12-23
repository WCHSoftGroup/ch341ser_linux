/*
 * ch34x.c
 *
 * Copyright (c) 2020 TECH39 <zhangj@wch.cn>
 *
 * USB to serial driver for USB to serial chip ch340, ch341, etc.
 *
 * Sponsored by SuSE
 *
 * System required:
 * Kernel version beyond 3.4.x
 * Update Log:
 * V1.0 - initial version
 * V1.01 - added supports for high baudrates
 * V1.10 - added supports for modem signal, flow control, etc.
 * V1.11 - fixed usb pid supports
 * V1.12 - fixed baud rate 0 bugs and add some debug messages 
 * V1.13 - special setting on usb packet upload timeout
 * V1.14 - changed read urb length to ep size
 * V1.15 - fixed hardware flowcontrol bugs
 * V1.16 - added supports for application to get uart state
 *		 - removed tty throttle methods
 *		 - submits urbs when uart open while not probe
 *		 - fixed tty kref errors in dtr_rts
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/idr.h>
#include <linux/list.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
#include <linux/sched/signal.h>
#endif

#include "ch34x.h"

#define DRIVER_AUTHOR "TECH39"
#define DRIVER_DESC "USB to serial driver for USB to serial chip ch340, ch341, etc."
#define VERSION_DESC "V1.16 On 2020.12.23"

static struct usb_driver ch34x_driver;
static struct tty_driver *ch34x_tty_driver;

static DEFINE_IDR(ch34x_minors);
static DEFINE_MUTEX(ch34x_minors_lock);


static void ch34x_tty_set_termios(struct tty_struct *tty,
				struct ktermios *termios_old);

/*
 * ch34x_minors accessors
 */

/*
 * Look up an ch34x structure by minor. If found and not disconnected, increment
 * its refcount and return it with its mutex held.
 */
static struct ch34x *ch34x_get_by_minor(unsigned int minor)
{
	struct ch34x *ch34x;

	mutex_lock(&ch34x_minors_lock);
	ch34x = idr_find(&ch34x_minors, minor);
	if (ch34x) {
		mutex_lock(&ch34x->mutex);
		if (ch34x->disconnected) {
			mutex_unlock(&ch34x->mutex);
			ch34x = NULL;
		} else {
			tty_port_get(&ch34x->port);
			mutex_unlock(&ch34x->mutex);
		}
	}
	mutex_unlock(&ch34x_minors_lock);
	return ch34x;
}

/*
 * Try to find an available minor number and if found, associate it with 'ch34x'.
 */
static int ch34x_alloc_minor(struct ch34x *ch34x)
{
	int minor;

	mutex_lock(&ch34x_minors_lock);
	minor = idr_alloc(&ch34x_minors, ch34x, 0, CH34X_TTY_MINORS, GFP_KERNEL);
	mutex_unlock(&ch34x_minors_lock);

	return minor;
}

/* Release the minor number associated with 'ch34x'.  */
static void ch34x_release_minor(struct ch34x *ch34x)
{
	mutex_lock(&ch34x_minors_lock);
	idr_remove(&ch34x_minors, ch34x->minor);
	mutex_unlock(&ch34x_minors_lock);
}

/*
 * Functions for CH34X control messages.
 */

static int ch34x_control_out(struct ch34x *ch34x, u8 request,
                u16 value, u16 index)
{	
    int retval;

	retval = usb_autopm_get_interface(ch34x->data);
	if (retval)
		return retval;
        
    retval = usb_control_msg(ch34x->dev, usb_sndctrlpipe(ch34x->dev, 0),
        request, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
        value, index, NULL, 0, DEFAULT_TIMEOUT);

    dev_vdbg(&ch34x->data->dev,
           "ch34x_control_out(%02x,%02x,%04x,%04x)\n",
            USB_DIR_OUT|0x40, request, value, index);

    usb_autopm_put_interface(ch34x->data);

	return retval < 0 ? retval : 0;
}

static int ch34x_control_in(struct ch34x *ch34x, 
                u8 request, u16 value, u16 index,
                char *buf, unsigned bufsize)
{	
    int retval;
	int i;

	retval = usb_autopm_get_interface(ch34x->data);
	if (retval)
		return retval;

	retval = usb_control_msg(ch34x->dev, usb_rcvctrlpipe(ch34x->dev, 0), request,
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
        value, index, buf, bufsize, DEFAULT_TIMEOUT);
    
    dev_vdbg(&ch34x->data->dev,
        "ch34x_control_in(%02x,%02x,%04x,%04x,%p,%u)\n",
        USB_DIR_IN | 0x40, (u8)request, (u16)value, (u16)index, buf,
        (int)bufsize);
	
	dev_vdbg(&ch34x->data->dev,
	        "ch34x_control_in result:");
	for (i = 0; i < retval; i++) {
	   dev_vdbg(&ch34x->data->dev,
	        "0x%.2x ", (u8)buf[i]);
	}

    usb_autopm_put_interface(ch34x->data);

	return retval;
}

static inline int ch34x_set_control(struct ch34x *ch34x, int control)
{
	u16 value = 0;

	value |= (u8)~control;

    return ch34x_control_out(ch34x, VENDOR_MODEM_OUT,
            value, 0x0000);
}

static inline int ch34x_set_line(struct ch34x *ch34x, struct usb_ch34x_line_coding *line)
{
	return 0;
}

static int ch34x_get_status(struct ch34x *ch34x)
{
	char *buffer;
	int retval;
	const unsigned size = 2;
	unsigned long flags;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	retval = ch34x_control_in(ch34x, VENDOR_READ, 0x0706, 0, buffer, size);
	if (retval < 0)
		goto out;

	/* setup the private status if available */
	if (retval > 0) {
		spin_lock_irqsave(&ch34x->read_lock, flags);
		ch34x->ctrlin = (~(*buffer)) & CH34X_MODEM_STAT;
		spin_unlock_irqrestore(&ch34x->read_lock, flags);
	} else
		retval = -EPROTO;

out:	
    kfree(buffer);
	return retval;
}

/* -------------------------------------------------------------------------- */

static int ch34x_configure(struct ch34x *ch34x)
{
	char *buffer;
	int r;
	const unsigned size = 2;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	/* expect two bytes 0x27 0x00 */
	r = ch34x_control_in(ch34x, VENDOR_VERSION, 0, 0, buffer, size);
	if (r < 0)
		goto out; 

	r = ch34x_control_out(ch34x, VENDOR_SERIAL_INIT, 0, 0);
	if (r < 0)
		goto out;

	r = ch34x_control_out(ch34x, VENDOR_WRITE, 0x1312, 0xd982);
	if (r < 0)
		goto out;
	r = ch34x_control_out(ch34x, VENDOR_WRITE, 0x0f2c, 0x0007);
	if (r < 0)
		goto out;
	r = ch34x_control_in(ch34x, VENDOR_READ, 0x2518, 0, buffer, size);
	if (r < 0)
		goto out;
	r = ch34x_get_status(ch34x);
	if (r < 0)
		goto out;
	r = ch34x_control_out(ch34x, VENDOR_WRITE, 0x2727, 0);
	if (r < 0)
		goto out;
	/*
	r = ch34x_control_out(ch34x, VENDOR_MODEM_OUT, 0x00ff, 0);
	if (r < 0)
		goto out;
	*/
	
out:	
	kfree(buffer);
	return r;
}

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int ch34x_wb_alloc(struct ch34x *ch34x)
{
	int i, wbn;
	struct ch34x_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &ch34x->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			return wbn;
		}
		wbn = (wbn + 1) % CH34X_NW;
		if (++i >= CH34X_NW)
			return -1;
	}
}

static int ch34x_wb_is_avail(struct ch34x *ch34x)
{
	int i, n;
	unsigned long flags;

	n = CH34X_NW;
	spin_lock_irqsave(&ch34x->write_lock, flags);
	for (i = 0; i < CH34X_NW; i++)
		n -= ch34x->wb[i].use;
	spin_unlock_irqrestore(&ch34x->write_lock, flags);
	return n;
}

/*
 * Finish write. Caller must hold ch34x->write_lock
 */
static void ch34x_write_done(struct ch34x *ch34x, struct ch34x_wb *wb)
{
	wb->use = 0;
	ch34x->transmitting--;
	usb_autopm_put_interface_async(ch34x->data);
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int ch34x_start_wb(struct ch34x *ch34x, struct ch34x_wb *wb)
{
	int rc;

	ch34x->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = ch34x->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dev_err(&ch34x->data->dev,
			"%s - usb_submit_urb(write bulk) failed: %d\n",
			__func__, rc);
		ch34x_write_done(ch34x, wb);
	}
	return rc;
}

static void ch34x_update_status(struct ch34x *ch34x,
					unsigned char *data, size_t len)
{
	unsigned long flags;
	u8 status;
	u8 difference;
	u8 type = data[0];

	if (len < 4)
		return;

	switch (type) {
	case CH34X_CTRL_TYPE_MODEM:
		status = ~data[2] & CH34X_MODEM_STAT;
	
		if (!ch34x->clocal && (ch34x->ctrlin & status & CH34X_CTRL_DCD)) {
			dev_dbg(&ch34x->data->dev, "%s - calling hangup\n",
					__func__);
			tty_port_tty_hangup(&ch34x->port, false);
		}
		
		spin_lock_irqsave(&ch34x->read_lock, flags);
		difference = status ^ ch34x->ctrlin;
		ch34x->ctrlin = status;
		ch34x->oldcount = ch34x->iocount;
		if (!difference) {
			spin_unlock_irqrestore(&ch34x->read_lock, flags);
			return;
		}		
		if (difference & CH34X_CTRL_CTS) {
			dev_vdbg(&ch34x->data->dev, "%s - cts change\n", __func__);
			ch34x->iocount.cts++;
		}
		if (difference & CH34X_CTRL_DSR) {
			dev_vdbg(&ch34x->data->dev, "%s - dsr change\n", __func__);
			ch34x->iocount.dsr++;
		}
		if (difference & CH34X_CTRL_RI) {
			dev_vdbg(&ch34x->data->dev, "%s - rng change\n", __func__);
			ch34x->iocount.rng++;
		}
		if (difference & CH34X_CTRL_DCD) {
			dev_vdbg(&ch34x->data->dev, "%s - dcd change\n", __func__);
			ch34x->iocount.dcd++;
		}
		spin_unlock_irqrestore(&ch34x->read_lock, flags);
		wake_up_interruptible(&ch34x->wioctl);
		break;
	case CH34X_CTRL_TYPE_OVERRUN:
		spin_lock_irqsave(&ch34x->read_lock, flags);
		ch34x->oldcount = ch34x->iocount;
		dev_err(&ch34x->data->dev, "%s - overrun error\n", __func__);
		ch34x->iocount.overrun++;
		spin_unlock_irqrestore(&ch34x->read_lock, flags);
		break;
	case CH34X_CTRL_TYPE_PARITY:
		spin_lock_irqsave(&ch34x->read_lock, flags);
		ch34x->oldcount = ch34x->iocount;
		dev_err(&ch34x->data->dev, "%s - parity error\n", __func__);
		ch34x->iocount.parity++;
		spin_unlock_irqrestore(&ch34x->read_lock, flags);		
		break;
	case CH34X_CTRL_TYPE_FRAMING:
		spin_lock_irqsave(&ch34x->read_lock, flags);
		ch34x->oldcount = ch34x->iocount;
		dev_err(&ch34x->data->dev, "%s - frame error\n", __func__);
		ch34x->iocount.frame++;
		spin_unlock_irqrestore(&ch34x->read_lock, flags);		
		break;
	default:
		dev_err(&ch34x->data->dev,
			"%s - unknown status received:"
			"len:%d, data0:0x%x, data1:0x%x\n",
			__func__,
			(int)len, data[0], data[1]);
		break;
	}
}

/*
 * Interrupt handlers for various CH34X device responses
 */

/* control interface reports status changes with "interrupt" transfers */
static void ch34x_ctrl_irq(struct urb *urb)
{
	struct ch34x *ch34x = urb->context;
	unsigned char *data = urb->transfer_buffer;
	unsigned int len = urb->actual_length;
	int status = urb->status;
	int retval;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&ch34x->data->dev,
				"%s - urb shutting down with status: %d\n",
				__func__, status);
		return;
	default:
		dev_dbg(&ch34x->data->dev,
				"%s - nonzero urb status received: %d\n",
				__func__, status);
		goto exit;
	}

	usb_mark_last_busy(ch34x->dev);
	ch34x_update_status(ch34x, data, len);
exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval && retval != -EPERM)
		dev_err(&ch34x->data->dev, "%s - usb_submit_urb failed: %d\n",
							__func__, retval);
}

static int ch34x_submit_read_urb(struct ch34x *ch34x, int index, gfp_t mem_flags)
{
	int res;

	if (!test_and_clear_bit(index, &ch34x->read_urbs_free))
		return 0;

	dev_vdbg(&ch34x->data->dev, "%s - urb %d\n", __func__, index);

	res = usb_submit_urb(ch34x->read_urbs[index], mem_flags);
	if (res) {
		if (res != -EPERM) {
			dev_err(&ch34x->data->dev,
					"%s - usb_submit_urb failed: %d\n",
					__func__, res);
		}
		set_bit(index, &ch34x->read_urbs_free);
		return res;
	}

	return 0;
}

static int ch34x_submit_read_urbs(struct ch34x *ch34x, gfp_t mem_flags)
{
	int res;
	int i;

	for (i = 0; i < ch34x->rx_buflimit; ++i) {
		res = ch34x_submit_read_urb(ch34x, i, mem_flags);
		if (res)
			return res;
	}

	return 0;
}

static void ch34x_process_read_urb(struct ch34x *ch34x, struct urb *urb)
{
	if (!urb->actual_length)
		return;

	tty_insert_flip_string(&ch34x->port, urb->transfer_buffer,
			urb->actual_length);
	tty_flip_buffer_push(&ch34x->port);
}

static void ch34x_read_bulk_callback(struct urb *urb)
{
	struct ch34x_rb *rb = urb->context;
	struct ch34x *ch34x = rb->instance;
	int status = urb->status;

	dev_vdbg(&ch34x->data->dev, "%s - urb %d, len %d\n", __func__,
					rb->index, urb->actual_length);

	if (!ch34x->dev) {
		set_bit(rb->index, &ch34x->read_urbs_free);
		dev_dbg(&ch34x->data->dev, "%s - disconnected\n", __func__);
		return;
	}
	if (status) {
		set_bit(rb->index, &ch34x->read_urbs_free);
		dev_dbg(&ch34x->data->dev, "%s - non-zero urb status: %d\n",
							__func__, status);
		return;
	}
	usb_mark_last_busy(ch34x->dev);
	ch34x_process_read_urb(ch34x, urb);
	set_bit(rb->index, &ch34x->read_urbs_free);
	ch34x_submit_read_urb(ch34x, rb->index, GFP_ATOMIC);
}

/* data interface wrote those outgoing bytes */
static void ch34x_write_bulk(struct urb *urb)
{
	struct ch34x_wb *wb = urb->context;
	struct ch34x *ch34x = wb->instance;
	unsigned long flags;
	int status = urb->status;
	
	dev_vdbg(&ch34x->data->dev, "%s, len %d\n", __func__, urb->actual_length);
	if (status || (urb->actual_length != urb->transfer_buffer_length))
		dev_vdbg(&ch34x->data->dev, "%s - len %d/%d, status %d\n",
			__func__,
			urb->actual_length,
			urb->transfer_buffer_length,
			status);

	spin_lock_irqsave(&ch34x->write_lock, flags);
	ch34x_write_done(ch34x, wb);
	spin_unlock_irqrestore(&ch34x->write_lock, flags);
	schedule_work(&ch34x->work);
}

static void ch34x_softint(struct work_struct *work)
{
	struct ch34x *ch34x = container_of(work, struct ch34x, work);

	dev_vdbg(&ch34x->data->dev, "%s\n", __func__);

	tty_port_tty_wakeup(&ch34x->port);
}

/*
 * TTY handlers
 */

static int ch34x_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct ch34x *ch34x;
	int retval;

	dev_dbg(tty->dev, "%s\n", __func__);

	ch34x = ch34x_get_by_minor(tty->index);
	if (!ch34x)
		return -ENODEV;

	retval = tty_standard_install(driver, tty);
	if (retval)
		goto error_init_termios;

	tty->driver_data = ch34x;

	return 0;

error_init_termios:
	tty_port_put(&ch34x->port);
	return retval;
}

static int ch34x_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct ch34x *ch34x = tty->driver_data;

	dev_dbg(tty->dev, "%s\n", __func__);

	return tty_port_open(&ch34x->port, tty, filp);
}

static void ch34x_port_dtr_rts(struct tty_port *port, int raise)
{
	struct ch34x *ch34x = container_of(port, struct ch34x, port);
	int res;

	dev_dbg(&ch34x->data->dev, "%s\n", __func__);

	if (raise)
		ch34x->ctrlout |= CH34X_CTRL_DTR | CH34X_CTRL_RTS;
	else
		ch34x->ctrlout &= ~(CH34X_CTRL_DTR | CH34X_CTRL_RTS);
	if (ch34x->hardflow)
		ch34x->ctrlout |= CH34X_CTRL_RTS;
	res = ch34x_set_control(ch34x, ch34x->ctrlout);
	if (res)
		dev_err(&ch34x->data->dev, "failed to set dtr/rts\n");
}

static int ch34x_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct ch34x *ch34x = container_of(port, struct ch34x, port);
	int retval = -ENODEV;
	int i;

	dev_dbg(&ch34x->data->dev, "%s\n", __func__);

	mutex_lock(&ch34x->mutex);
	if (ch34x->disconnected)
		goto disconnected;

	retval = usb_autopm_get_interface(ch34x->data);
	if (retval)
		goto error_get_interface;

	/*
	 * FIXME: Why do we need this? Allocating 64K of physically contiguous
	 * memory is really nasty...
	 */
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	ch34x->data->needs_remote_wakeup = 1;
	retval = ch34x_configure(ch34x);
	if (retval)
		goto error_configure;

	ch34x_tty_set_termios(tty, NULL);

	retval = usb_submit_urb(ch34x->ctrlurb, GFP_KERNEL);
	if (retval) {
		dev_err(&ch34x->data->dev,
			"%s - usb_submit_urb(ctrl cmd) failed\n", __func__);
		goto error_submit_urb;
	}

	retval = ch34x_submit_read_urbs(ch34x, GFP_KERNEL);
	if (retval)
		goto error_submit_read_urbs;
	usb_autopm_put_interface(ch34x->data);

	mutex_unlock(&ch34x->mutex);

	return 0;

error_submit_read_urbs:
	for (i = 0; i < ch34x->rx_buflimit; i++)
		usb_kill_urb(ch34x->read_urbs[i]);
error_submit_urb:
	usb_kill_urb(ch34x->ctrlurb);
error_configure:
	usb_autopm_put_interface(ch34x->data);
error_get_interface:
disconnected:
	mutex_unlock(&ch34x->mutex);

	return usb_translate_errors(retval);
}

static void ch34x_port_destruct(struct tty_port *port)
{
	struct ch34x *ch34x = container_of(port, struct ch34x, port);

	dev_dbg(&ch34x->data->dev, "%s\n", __func__);

	ch34x_release_minor(ch34x);
	usb_put_intf(ch34x->data);
	kfree(ch34x);
}

static void ch34x_port_shutdown(struct tty_port *port)
{
	struct ch34x *ch34x = container_of(port, struct ch34x, port);
	struct urb *urb;
	struct ch34x_wb *wb;
	int i;
	
	dev_dbg(&ch34x->data->dev, "%s\n", __func__);

	usb_autopm_get_interface_no_resume(ch34x->data);
	ch34x->data->needs_remote_wakeup = 0;
	usb_autopm_put_interface(ch34x->data);

	for (;;) {
		urb = usb_get_from_anchor(&ch34x->delayed);
		if (!urb)
			break;
		wb = urb->context;
		wb->use = 0;
		usb_autopm_put_interface_async(ch34x->data);
	}

	usb_kill_urb(ch34x->ctrlurb);
	for (i = 0; i < CH34X_NW; i++)
		usb_kill_urb(ch34x->wb[i].urb);
	for (i = 0; i < ch34x->rx_buflimit; i++)
		usb_kill_urb(ch34x->read_urbs[i]);
}

static void ch34x_tty_cleanup(struct tty_struct *tty)
{
	struct ch34x *ch34x = tty->driver_data;
	dev_dbg(&ch34x->data->dev, "%s\n", __func__);
	tty_port_put(&ch34x->port);
}

static void ch34x_tty_hangup(struct tty_struct *tty)
{
	struct ch34x *ch34x = tty->driver_data;
	dev_dbg(&ch34x->data->dev, "%s\n", __func__);
	tty_port_hangup(&ch34x->port);
}

static void ch34x_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct ch34x *ch34x = tty->driver_data;
	dev_dbg(&ch34x->data->dev, "%s\n", __func__);
	tty_port_close(&ch34x->port, tty, filp);
}

static int ch34x_tty_write(struct tty_struct *tty,
					const unsigned char *buf, int count)
{
	struct ch34x *ch34x = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct ch34x_wb *wb;

	if (!count)
		return 0;

	dev_vdbg(&ch34x->data->dev, "%s - count %d\n", __func__, count);

	spin_lock_irqsave(&ch34x->write_lock, flags);
	wbn = ch34x_wb_alloc(ch34x);
	if (wbn < 0) {
		spin_unlock_irqrestore(&ch34x->write_lock, flags);
		return 0;
	}
	wb = &ch34x->wb[wbn];

	if (!ch34x->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&ch34x->write_lock, flags);
		return -ENODEV;
	}

	count = (count > ch34x->writesize) ? ch34x->writesize : count;

	memcpy(wb->buf, buf, count);
	wb->len = count;

	stat = usb_autopm_get_interface_async(ch34x->data);
	if (stat) {
		wb->use = 0;
		spin_unlock_irqrestore(&ch34x->write_lock, flags);
		return stat;
	}

	if (ch34x->susp_count) {
		usb_anchor_urb(wb->urb, &ch34x->delayed);
		spin_unlock_irqrestore(&ch34x->write_lock, flags);
		return count;
	}

	stat = ch34x_start_wb(ch34x, wb);
	spin_unlock_irqrestore(&ch34x->write_lock, flags);

	if (stat < 0)
		return stat;
	return count;
}

static int ch34x_tty_write_room(struct tty_struct *tty)
{
	struct ch34x *ch34x = tty->driver_data;
	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	return ch34x_wb_is_avail(ch34x) ? ch34x->writesize : 0;
}

static int ch34x_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct ch34x *ch34x = tty->driver_data;
	/*
	 * if the device was unplugged then any remaining characters fell out
	 * of the connector ;)
	 */
	if (ch34x->disconnected)
		return 0;
	/*
	 * This is inaccurate (overcounts), but it works.
	 */
	return (CH34X_NW - ch34x_wb_is_avail(ch34x)) * ch34x->writesize;
}

static int ch34x_tty_break_ctl(struct tty_struct *tty, int state)
{
	struct ch34x *ch34x = tty->driver_data;
	int retval;
	uint16_t reg_contents;
	uint8_t *break_reg;
	const u16 ch34x_break_reg = 
		((u16) CH34X_REG_LCR << 8) | CH34X_REG_BREAK;

	dev_dbg(&ch34x->data->dev, "%s\n", __func__);

	break_reg = kmalloc(2, GFP_KERNEL);
	if (!break_reg)
		return -1;

	retval = ch34x_control_in(ch34x, VENDOR_READ, ch34x_break_reg, 0, break_reg, 2);
	if (retval < 0) {
		dev_err(&ch34x->data->dev, "%s - ch34x_control_in error (%d)\n",
				__func__, retval);
		goto out;
	}
	dev_dbg(&ch34x->data->dev, "%s - initial ch34x break register contents - reg1: %x, reg2: %x\n",
		__func__, break_reg[0], break_reg[1]);
	if (state != 0) {
		dev_dbg(&ch34x->data->dev, "%s - Enter break state requested\n", __func__);
		break_reg[0] &= ~CH34X_NBREAK_BITS;
		break_reg[1] &= ~CH34X_LCR_ENABLE_TX;
	} else {
		dev_dbg(&ch34x->data->dev, "%s - Leave break state requested\n", __func__);
		break_reg[0] |= CH34X_NBREAK_BITS;
		break_reg[1] |= CH34X_LCR_ENABLE_TX;
	}
	dev_dbg(&ch34x->data->dev, "%s - New ch341 break register contents - reg1: %x, reg2: %x\n",
		__func__, break_reg[0], break_reg[1]);
	reg_contents = get_unaligned_le16(break_reg);

	retval = ch34x_control_out(ch34x, VENDOR_WRITE, ch34x_break_reg,
			reg_contents);
	if (retval < 0)
		dev_err(&ch34x->data->dev, "%s - USB control write error (%d)\n",
				__func__, retval);
out:
	kfree(break_reg);

	return retval;
}

static int ch34x_tty_tiocmget(struct tty_struct *tty)
{
	struct ch34x *ch34x = tty->driver_data;
	unsigned long flags;
	unsigned int result;

	dev_dbg(&ch34x->data->dev, "%s\n", __func__);

	spin_lock_irqsave(&ch34x->read_lock, flags);
	result = (ch34x->ctrlout & CH34X_CTRL_DTR ? TIOCM_DTR : 0) |
	       (ch34x->ctrlout & CH34X_CTRL_RTS ? TIOCM_RTS : 0) |
		   (ch34x->ctrlin  & CH34X_CTRL_CTS ? TIOCM_CTS : 0) | 
	       (ch34x->ctrlin  & CH34X_CTRL_DSR ? TIOCM_DSR : 0) |
	       (ch34x->ctrlin  & CH34X_CTRL_RI  ? TIOCM_RI  : 0) |
	       (ch34x->ctrlin  & CH34X_CTRL_DCD ? TIOCM_CD  : 0);
	spin_unlock_irqrestore(&ch34x->read_lock, flags);
	dev_dbg(&ch34x->data->dev, "%s - result = %x\n", __func__, result);
		   
	return result;
}

static int ch34x_tty_tiocmset(struct tty_struct *tty,
			    unsigned int set, unsigned int clear)
{
	struct ch34x *ch34x = tty->driver_data;
	unsigned int newctrl;

	dev_dbg(&ch34x->data->dev, "%s\n", __func__);
	dev_dbg(&ch34x->data->dev, "TIOCM_DTR:0x%4x, TIOCM_RTS:0x%4x\n", TIOCM_DTR, TIOCM_RTS);
	dev_dbg(&ch34x->data->dev, "set:0x%4x, clear:0x%4x\n", set, clear);
	
	newctrl = ch34x->ctrlout;
	set = (set & TIOCM_DTR ? CH34X_CTRL_DTR : 0) |
					(set & TIOCM_RTS ? CH34X_CTRL_RTS : 0);
	clear = (clear & TIOCM_DTR ? CH34X_CTRL_DTR : 0) |
					(clear & TIOCM_RTS ? CH34X_CTRL_RTS : 0);
	newctrl = (newctrl & ~clear) | set;
	if (C_CRTSCTS(tty))
		newctrl |= CH34X_CTRL_RTS;

	if (ch34x->ctrlout == newctrl)
		return 0;
	return ch34x_set_control(ch34x, ch34x->ctrlout = newctrl);
}

static int get_serial_info(struct ch34x *ch34x, struct serial_struct __user *info)
{
	struct serial_struct tmp;

	if (!info)
		return -EINVAL;

	memset(&tmp, 0, sizeof(tmp));
	tmp.flags = ASYNC_LOW_LATENCY;
	tmp.xmit_fifo_size = ch34x->writesize;
	tmp.baud_base = le32_to_cpu(ch34x->line.dwDTERate);
	tmp.close_delay = ch34x->port.close_delay / 10;
	tmp.closing_wait = ch34x->port.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
				ASYNC_CLOSING_WAIT_NONE :
				ch34x->port.closing_wait / 10;

	if (copy_to_user(info, &tmp, sizeof(tmp)))
		return -EFAULT;
	else
		return 0;
}
				
static int set_serial_info(struct ch34x *ch34x,
				struct serial_struct __user *newinfo)
{
	struct serial_struct new_serial;
	unsigned int closing_wait, close_delay;
	int retval = 0;

	if (copy_from_user(&new_serial, newinfo, sizeof(new_serial)))
		return -EFAULT;

	close_delay = new_serial.close_delay * 10;
	closing_wait = new_serial.closing_wait == ASYNC_CLOSING_WAIT_NONE ?
			ASYNC_CLOSING_WAIT_NONE : new_serial.closing_wait * 10;

	mutex_lock(&ch34x->port.mutex);

	if (!capable(CAP_SYS_ADMIN)) {
		if ((close_delay != ch34x->port.close_delay) ||
			(closing_wait != ch34x->port.closing_wait))
			retval = -EPERM;
		else
			retval = -EOPNOTSUPP;
	} else {
		ch34x->port.close_delay  = close_delay;
		ch34x->port.closing_wait = closing_wait;
	}

	mutex_unlock(&ch34x->port.mutex);
	return retval;
}
				
static int wait_serial_change(struct ch34x *ch34x, unsigned long arg)
{
	int rv = 0;
	DECLARE_WAITQUEUE(wait, current);
	struct async_icount old, new;

	do {
		spin_lock_irq(&ch34x->read_lock);
		old = ch34x->oldcount;
		new = ch34x->iocount;
		ch34x->oldcount = new;
		spin_unlock_irq(&ch34x->read_lock);

		if ((arg & TIOCM_CTS) &&
			old.cts != new.cts)
			break;
		if ((arg & TIOCM_DSR) &&
			old.dsr != new.dsr)
			break;
		if ((arg & TIOCM_RI) &&
			old.rng != new.rng)
			break;
		if ((arg & TIOCM_CD)  &&
			old.dcd != new.dcd)
			break;
		
		add_wait_queue(&ch34x->wioctl, &wait);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		remove_wait_queue(&ch34x->wioctl, &wait);
		if (ch34x->disconnected) {
			rv = -ENODEV;
		} else {
			if (signal_pending(current))
				rv = -ERESTARTSYS;
		}
	} while (!rv);

	return rv;
}
				
static int get_serial_usage(struct ch34x *ch34x,
				struct serial_icounter_struct __user *count)
{
	struct serial_icounter_struct icount;
	int rv = 0;

	memset(&icount, 0, sizeof(icount));
	icount.cts = ch34x->iocount.cts;
	icount.dsr = ch34x->iocount.dsr;
	icount.rng = ch34x->iocount.rng;
	icount.dcd = ch34x->iocount.dcd;
	icount.frame = ch34x->iocount.frame;
	icount.overrun = ch34x->iocount.overrun;
	icount.parity = ch34x->iocount.parity;
	
	if (copy_to_user(count, &icount, sizeof(icount)) > 0)
		rv = -EFAULT;

	return rv;
}


static int ch34x_tty_ioctl(struct tty_struct *tty,
					unsigned int cmd, unsigned long arg)
{
	struct ch34x *ch34x = tty->driver_data;
	int rv = -ENOIOCTLCMD;

	dev_dbg(&ch34x->data->dev, "%s\n", __func__);
	
	switch (cmd) {
	case TIOCGSERIAL: /* gets serial port data */
		rv = get_serial_info(ch34x, (struct serial_struct __user *)arg);
		break;
	case TIOCSSERIAL:
		rv = set_serial_info(ch34x, (struct serial_struct __user *)arg);
		break;
	case TIOCMIWAIT:
		rv = usb_autopm_get_interface(ch34x->data);
		if (rv < 0) {
			rv = -EIO;
			break;
		}
		rv = wait_serial_change(ch34x, arg);
		usb_autopm_put_interface(ch34x->data);
		break;
	case TIOCGICOUNT:
		rv = get_serial_usage(ch34x, (struct serial_icounter_struct __user *)arg);
		break;
	}

	return rv;
}

static int ch34x_get_baud_rate(unsigned int baud_rate,
		unsigned char *factor, unsigned char *divisor)
{
	unsigned char a;
	unsigned char b;
	unsigned long c;

	switch (baud_rate) {
	case 921600: 
		a = 0xf3; 
		b = 7; 
		break; 
	case 307200:
		a = 0xd9; 
		b = 7; 
		break; 	
	default: 
		if (baud_rate > 6000000/255) { 
			b = 3;
			c = 6000000;
		} else if (baud_rate > 750000/255) {  
			b = 2;
			c = 750000;
		} else if (baud_rate > 93750/255) { 
			b = 1;
			c = 93750;
		} else {
			b = 0;
			c = 11719;
		}
		a = (unsigned char)(c / baud_rate);
		if (a == 0 || a == 0xFF) return -EINVAL;
		if ((c / a - baud_rate) > (baud_rate - c / (a + 1))) 
			a ++;
		a = 256 - a;
		break;
	}
	*factor = a;
	*divisor = b;
	
	return 0;
}

static void ch34x_tty_set_termios(struct tty_struct *tty,
						struct ktermios *termios_old)
{
	struct ch34x *ch34x = tty->driver_data;
	struct ktermios *termios = &tty->termios;
	struct usb_ch34x_line_coding newline;
	int newctrl = ch34x->ctrlout;

	unsigned char divisor = 0;
	unsigned char reg_count = 0;
	unsigned char factor = 0;
	unsigned char reg_value = 0;
	unsigned short value = 0;
	unsigned short index = 0;

	dev_dbg(tty->dev, "%s\n", __func__);
	
	if (termios_old &&
		!tty_termios_hw_change(&tty->termios, termios_old)) {
		dev_dbg(tty->dev, "%s - nothing to change\n", __func__);
		return;
	}

	newline.dwDTERate = tty_get_baud_rate(tty);
	
	if (newline.dwDTERate == 0)
			newline.dwDTERate = 9600;
    ch34x_get_baud_rate(newline.dwDTERate, &factor, &divisor);

	newline.bCharFormat = termios->c_cflag & CSTOPB ? 2 : 1;
	if (newline.bCharFormat == 2)
	    reg_value |= CH34X_LCR_STOP_BITS_2;
	    
	newline.bParityType = termios->c_cflag & PARENB ?
				(termios->c_cflag & PARODD ? 1 : 2) +
				(termios->c_cflag & CMSPAR ? 2 : 0) : 0;

	switch (newline.bParityType) {
    case 0x01:
        reg_value |= CH34X_LCR_PAR_ODD;
        dev_dbg(&ch34x->data->dev, "parity = odd\n");
        break;
    case 0x02:
		reg_value |= CH34X_LCR_PAR_EVEN;
		dev_dbg(&ch34x->data->dev, "parity = even\n");
        break;
    case 0x03:
		reg_value |= CH34X_LCR_PAR_MARK;
		dev_dbg(&ch34x->data->dev, "parity = mark\n");
        break;
    case 0x04:
		reg_value |= CH34X_LCR_PAR_SPACE;
		dev_dbg(&ch34x->data->dev, "parity = space\n");
        break;
    default:
        dev_dbg(&ch34x->data->dev, "parity = none\n");
        break;
	}
		
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		newline.bDataBits = 5;
		reg_value |= CH34X_LCR_CS5;
		break;
	case CS6:
		newline.bDataBits = 6;
		reg_value |= CH34X_LCR_CS6;
		break;
	case CS7:
		newline.bDataBits = 7;
		reg_value |= CH34X_LCR_CS7;
		break;
	case CS8:
	default:
		newline.bDataBits = 8;
		reg_value |= CH34X_LCR_CS8;
		break;
	}
	
	/* FIXME: Needs to clear unsupported bits in the termios */
	ch34x->clocal = ((termios->c_cflag & CLOCAL) != 0);

	if (C_BAUD(tty) == B0) {
		newline.dwDTERate = ch34x->line.dwDTERate;
		newctrl &= ~CH34X_CTRL_DTR;
	} else if (termios_old && (termios_old->c_cflag & CBAUD) == B0) {
		newctrl |= CH34X_CTRL_DTR;
	}

	//enable SFR_UART RX and TX
	reg_value |= 0xc0;
	//enable SFR_UART Control register and timer
	reg_count |= 0x9c;

	value |= reg_count;
	value |= (unsigned short)reg_value << 8;
	index |= 0x80 | divisor;
	index |= (unsigned short)factor << 8;
	ch34x_control_out(ch34x, VENDOR_SERIAL_INIT, value, index);

	if (newctrl != ch34x->ctrlout)
		ch34x_set_control(ch34x, ch34x->ctrlout = newctrl);

	if (memcmp(&ch34x->line, &newline, sizeof newline)) {
		memcpy(&ch34x->line, &newline, sizeof newline);
		dev_dbg(&ch34x->data->dev, "%s - set line: %d %d %d %d\n",
			__func__,
			newline.dwDTERate,
			newline.bDataBits, newline.bCharFormat,
			newline.bParityType);
	}
	if (C_CRTSCTS(tty)) {
		ch34x_control_out(ch34x, VENDOR_WRITE, 0x2727, 0x0101);	
		newctrl |= CH34X_CTRL_RTS;
		ch34x_set_control(ch34x, ch34x->ctrlout = newctrl);
		ch34x->hardflow = true;
	}
	else {
		ch34x_control_out(ch34x, VENDOR_WRITE, 0x2727, 0x0000);
		ch34x->hardflow = false;
	}
}

static const struct tty_port_operations ch34x_port_ops = {
	.dtr_rts = ch34x_port_dtr_rts,
	.shutdown = ch34x_port_shutdown,
	.activate = ch34x_port_activate,
	.destruct = ch34x_port_destruct,
};

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void ch34x_write_buffers_free(struct ch34x *ch34x)
{
	int i;
	struct ch34x_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(ch34x->data);

	for (wb = &ch34x->wb[0], i = 0; i < CH34X_NW; i++, wb++)
		usb_free_coherent(usb_dev, ch34x->writesize, wb->buf, wb->dmah);
}

static void ch34x_read_buffers_free(struct ch34x *ch34x)
{
	struct usb_device *usb_dev = interface_to_usbdev(ch34x->data);
	int i;

	for (i = 0; i < ch34x->rx_buflimit; i++)
		usb_free_coherent(usb_dev, ch34x->readsize,
			  ch34x->read_buffers[i].base, ch34x->read_buffers[i].dma);
}

/* Little helper: write buffers allocate */
static int ch34x_write_buffers_alloc(struct ch34x *ch34x)
{
	int i;
	struct ch34x_wb *wb;

	for (wb = &ch34x->wb[0], i = 0; i < CH34X_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(ch34x->dev, ch34x->writesize, GFP_KERNEL,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(ch34x->dev, ch34x->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int ch34x_probe(struct usb_interface *intf,
		     const struct usb_device_id *id)
{
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct ch34x *ch34x;
	int minor;
	int ctrlsize, readsize;
	u8 *buf;
	unsigned long quirks;
	int num_rx_buf = CH34X_NR;
	int i;
	struct device *tty_dev;
	int rv = -ENOMEM;

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;

	/* handle quirks deadly to normal probing*/

	data_interface = usb_ifnum_to_if(usb_dev, 0);
	
	if (intf != data_interface)
		return -ENODEV;

	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2 ||
	    data_interface->cur_altsetting->desc.bNumEndpoints == 0)
		return -EINVAL;

	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;
	epctrl = &data_interface->cur_altsetting->endpoint[2].desc;

	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		dev_dbg(&intf->dev,
			"The data interface has switched endpoints\n");
		swap(epread, epwrite);
	}

	dev_dbg(&intf->dev, "interface is valid\n");

	ch34x = kzalloc(sizeof(struct ch34x), GFP_KERNEL);
	if (ch34x == NULL)
		goto alloc_fail;

	minor = ch34x_alloc_minor(ch34x);
	if (minor < 0) {
		dev_err(&intf->dev, "no more free ch34x devices\n");
		kfree(ch34x);
		return -ENODEV;
	}

	ctrlsize = usb_endpoint_maxp(epctrl);
	readsize = usb_endpoint_maxp(epread) *
				(quirks == SINGLE_RX_URB ? 1 : 1);
	ch34x->writesize = usb_endpoint_maxp(epwrite) * 20;
	ch34x->data = data_interface;
	ch34x->minor = minor;
	ch34x->dev = usb_dev;
	ch34x->ctrlsize = ctrlsize;
	ch34x->readsize = readsize;
	ch34x->rx_buflimit = num_rx_buf;

	dev_dbg(&intf->dev, "epctrl: %d, epread: %d, epwrite: %d\n",
		usb_endpoint_maxp(epctrl), usb_endpoint_maxp(epread),
		usb_endpoint_maxp(epwrite));

	INIT_WORK(&ch34x->work, ch34x_softint);
	init_waitqueue_head(&ch34x->wioctl);
	spin_lock_init(&ch34x->write_lock);
	spin_lock_init(&ch34x->read_lock);
	mutex_init(&ch34x->mutex);
	ch34x->rx_endpoint = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	tty_port_init(&ch34x->port);
	ch34x->port.ops = &ch34x_port_ops;
	init_usb_anchor(&ch34x->delayed);
	ch34x->quirks = quirks;

	buf = usb_alloc_coherent(usb_dev, ctrlsize, GFP_KERNEL, &ch34x->ctrl_dma);
	if (!buf)
		goto alloc_fail2;
	ch34x->ctrl_buffer = buf;

	if (ch34x_write_buffers_alloc(ch34x) < 0)
		goto alloc_fail4;

	ch34x->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ch34x->ctrlurb)
		goto alloc_fail5;

	for (i = 0; i < num_rx_buf; i++) {
		struct ch34x_rb *rb = &(ch34x->read_buffers[i]);
		struct urb *urb;

		rb->base = usb_alloc_coherent(ch34x->dev, readsize, GFP_KERNEL,
								&rb->dma);
		if (!rb->base)
			goto alloc_fail6;
		rb->index = i;
		rb->instance = ch34x;

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			goto alloc_fail6;

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		urb->transfer_dma = rb->dma;
		usb_fill_bulk_urb(urb, ch34x->dev,
			ch34x->rx_endpoint,
			rb->base,
			ch34x->readsize,
			ch34x_read_bulk_callback, rb);

		ch34x->read_urbs[i] = urb;
		__set_bit(i, &ch34x->read_urbs_free);
	}
	for (i = 0; i < CH34X_NW; i++) {
		struct ch34x_wb *snd = &(ch34x->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL)
			goto alloc_fail7;

		usb_fill_bulk_urb(snd->urb, usb_dev,
			usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
			NULL, ch34x->writesize, ch34x_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = ch34x;
	}

	usb_set_intfdata(intf, ch34x);

	usb_fill_int_urb(ch34x->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 ch34x->ctrl_buffer, ctrlsize, ch34x_ctrl_irq, ch34x,
			 epctrl->bInterval ? epctrl->bInterval : 16);
	ch34x->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ch34x->ctrlurb->transfer_dma = ch34x->ctrl_dma;

	dev_info(&intf->dev, "ttyCH341USB%d: ch34x USB device\n", minor);

	usb_driver_claim_interface(&ch34x_driver, data_interface, ch34x);
	usb_set_intfdata(data_interface, ch34x);

	usb_get_intf(data_interface);
	tty_dev = tty_port_register_device(&ch34x->port, ch34x_tty_driver, minor,
			&data_interface->dev);
	if (IS_ERR(tty_dev)) {
		rv = PTR_ERR(tty_dev);
		goto alloc_fail7;
	}

	if (quirks & CLEAR_HALT_CONDITIONS) {
		usb_clear_halt(usb_dev, usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress));
		usb_clear_halt(usb_dev, usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress));
	}

	dev_dbg(&intf->dev, "ch34x_probe finished!\n");

	return 0;

alloc_fail7:
	usb_set_intfdata(intf, NULL);
	for (i = 0; i < CH34X_NW; i++)
		usb_free_urb(ch34x->wb[i].urb);
alloc_fail6:
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(ch34x->read_urbs[i]);
	ch34x_read_buffers_free(ch34x);
	usb_free_urb(ch34x->ctrlurb);
alloc_fail5:
	ch34x_write_buffers_free(ch34x);
alloc_fail4:
	usb_free_coherent(usb_dev, ctrlsize, ch34x->ctrl_buffer, ch34x->ctrl_dma);
alloc_fail2:
	ch34x_release_minor(ch34x);
	kfree(ch34x);
alloc_fail:
	return rv;
}

static void stop_data_traffic(struct ch34x *ch34x)
{
	int i;

	dev_dbg(&ch34x->data->dev, "%s\n", __func__);

	usb_kill_urb(ch34x->ctrlurb);
	for (i = 0; i < CH34X_NW; i++)
		usb_kill_urb(ch34x->wb[i].urb);
	for (i = 0; i < ch34x->rx_buflimit; i++)
		usb_kill_urb(ch34x->read_urbs[i]);

	cancel_work_sync(&ch34x->work);
}

static void ch34x_disconnect(struct usb_interface *intf)
{
	struct ch34x *ch34x = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct tty_struct *tty;
	int i;

	dev_dbg(&intf->dev, "%s\n", __func__);

	/* sibling interface is already cleaning up */
	if (!ch34x)
		return;

	mutex_lock(&ch34x->mutex);
	ch34x->disconnected = true;
	wake_up_all(&ch34x->wioctl);
	usb_set_intfdata(ch34x->data, NULL);
	mutex_unlock(&ch34x->mutex);

	tty = tty_port_tty_get(&ch34x->port);
	if (tty) {
		tty_vhangup(tty);
		tty_kref_put(tty);
	}

	stop_data_traffic(ch34x);

	tty_unregister_device(ch34x_tty_driver, ch34x->minor);

	usb_free_urb(ch34x->ctrlurb);
	for (i = 0; i < CH34X_NW; i++)
		usb_free_urb(ch34x->wb[i].urb);
	for (i = 0; i < ch34x->rx_buflimit; i++)
		usb_free_urb(ch34x->read_urbs[i]);
	ch34x_write_buffers_free(ch34x);
	usb_free_coherent(usb_dev, ch34x->ctrlsize, ch34x->ctrl_buffer, ch34x->ctrl_dma);
	ch34x_read_buffers_free(ch34x);

	usb_driver_release_interface(&ch34x_driver, ch34x->data);

	tty_port_put(&ch34x->port);
	dev_info(&intf->dev, "%s\n", "ch34x usb device disconnect.");
}

#ifdef CONFIG_PM
static int ch34x_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct ch34x *ch34x = usb_get_intfdata(intf);
	int cnt;

	dev_dbg(&intf->dev, "%s\n", __func__);
	spin_lock_irq(&ch34x->write_lock);
	if (PMSG_IS_AUTO(message)) {
		if (ch34x->transmitting) {
			spin_unlock_irq(&ch34x->write_lock);
			return -EBUSY;
		}
	}
	cnt = ch34x->susp_count++;
	spin_unlock_irq(&ch34x->write_lock);

	if (cnt)
		return 0;

	stop_data_traffic(ch34x);

	return 0;
}

static int ch34x_resume(struct usb_interface *intf)
{
	struct ch34x *ch34x = usb_get_intfdata(intf);
	struct urb *urb;
	int rv = 0;

	dev_dbg(&intf->dev, "%s\n", __func__);
	spin_lock_irq(&ch34x->write_lock);

	if (--ch34x->susp_count)
		goto out;

	if (test_bit(ASYNCB_INITIALIZED, &ch34x->port.flags)) {
		rv = usb_submit_urb(ch34x->ctrlurb, GFP_ATOMIC);

		for (;;) {
			urb = usb_get_from_anchor(&ch34x->delayed);
			if (!urb)
				break;

			ch34x_start_wb(ch34x, urb->context);
		}

		/*
		 * delayed error checking because we must
		 * do the write path at all cost
		 */
		if (rv < 0)
			goto out;

		rv = ch34x_submit_read_urbs(ch34x, GFP_ATOMIC);
	}
out:
	spin_unlock_irq(&ch34x->write_lock);

	return rv;
}

static int ch34x_reset_resume(struct usb_interface *intf)
{
	struct ch34x *ch34x = usb_get_intfdata(intf);

	dev_dbg(&intf->dev, "%s\n", __func__);
	if (test_bit(ASYNCB_INITIALIZED, &ch34x->port.flags))
		tty_port_tty_hangup(&ch34x->port, false);

	return ch34x_resume(intf);
}

#endif /* CONFIG_PM */

/*
 * USB driver structure.
 */

static const struct usb_device_id ch34x_ids[] = {
	/* quirky and broken devices */
	{ USB_DEVICE(0x1a86, 0x7523), /* ch340 chip */
	.driver_info = CLEAR_HALT_CONDITIONS, },

	{ USB_DEVICE(0x1a86, 0x7522), /* ch340K chip */
	.driver_info = CLEAR_HALT_CONDITIONS, },

	{ USB_DEVICE(0x1a86, 0x5523), /* ch341 chip */
	.driver_info = CLEAR_HALT_CONDITIONS, },
	
	{ USB_DEVICE(0x1a86, 0xE523), /* ch330 chip */
	.driver_info = CLEAR_HALT_CONDITIONS, },
	
	{ USB_DEVICE(0x4348, 0x5523), /* ch340 custom chip */
	.driver_info = CLEAR_HALT_CONDITIONS, },

	{ }
};

MODULE_DEVICE_TABLE(usb, ch34x_ids);

static struct usb_driver ch34x_driver = {
	.name =		"usb_ch34x",
	.probe =	ch34x_probe,
	.disconnect =	ch34x_disconnect,
#ifdef CONFIG_PM
	.suspend =	ch34x_suspend,
	.resume =	ch34x_resume,
	.reset_resume =	ch34x_reset_resume,
#endif
	.id_table =	ch34x_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
	.disable_hub_initiated_lpm = 1,
};

/*
 * TTY driver structures.
 */

static const struct tty_operations ch34x_ops = {
	.install =		ch34x_tty_install,
	.open =			ch34x_tty_open,
	.close =		ch34x_tty_close,
	.cleanup =		ch34x_tty_cleanup,
	.hangup =		ch34x_tty_hangup,
	.write =		ch34x_tty_write,
	.write_room =		ch34x_tty_write_room,
	.ioctl =		ch34x_tty_ioctl,
	.chars_in_buffer =	ch34x_tty_chars_in_buffer,
	.break_ctl =		ch34x_tty_break_ctl,
	.set_termios =		ch34x_tty_set_termios,
	.tiocmget =		ch34x_tty_tiocmget,
	.tiocmset =		ch34x_tty_tiocmset,
};

/*
 * Init / exit.
 */

static int __init ch34x_init(void)
{
	int retval;
	ch34x_tty_driver = alloc_tty_driver(CH34X_TTY_MINORS);
	if (!ch34x_tty_driver)
		return -ENOMEM;
	ch34x_tty_driver->driver_name = "ch34x_uart",
	ch34x_tty_driver->name = "ttyCH341USB",
	ch34x_tty_driver->major = CH34X_TTY_MAJOR,
	ch34x_tty_driver->minor_start = 0,
	ch34x_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	ch34x_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	ch34x_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	ch34x_tty_driver->init_termios = tty_std_termios;
	ch34x_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD |
								HUPCL | CLOCAL;
	tty_set_operations(ch34x_tty_driver, &ch34x_ops);

	retval = tty_register_driver(ch34x_tty_driver);
	if (retval) {
		put_tty_driver(ch34x_tty_driver);
		return retval;
	}

	retval = usb_register(&ch34x_driver);
	if (retval) {
		tty_unregister_driver(ch34x_tty_driver);
		put_tty_driver(ch34x_tty_driver);
		return retval;
	}

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");
	printk(KERN_INFO KBUILD_MODNAME ": " VERSION_DESC "\n");

	return 0;
}

static void __exit ch34x_exit(void)
{
	usb_deregister(&ch34x_driver);
	tty_unregister_driver(ch34x_tty_driver);
	put_tty_driver(ch34x_tty_driver);
	idr_destroy(&ch34x_minors);
	printk(KERN_INFO KBUILD_MODNAME ": " "ch34x driver exit.\n");
}

module_init(ch34x_init);
module_exit(ch34x_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(CH34X_TTY_MAJOR);


