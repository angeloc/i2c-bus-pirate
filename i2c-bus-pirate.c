// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2019 Angelo Compagnucci <angelo.compagnucci@gmail.com>
 *
 * Driver for the Bus Pirate
 * http://dangerousprototypes.com/docs/Bus_Pirate
 * These devices includeprovides an I2C adapter which can be controlled over a
 * ttyUSB serial port.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/string.h>

#define BUSPIRATE_BUFFER_SIZE			32

#define SERIO_BUSPIRATE				0x40

#define BUSPIRATE_STATE_NONE			0
#define BUSPIRATE_STATE_INIT			1
#define BUSPIRATE_STATE_I2C			2
#define BUSPIRATE_STATE_BBIO			3
#define BUSPIRATE_STATE_CMD			4
#define BUSPIRATE_STATE_CMDREPLY		5
#define BUSPIRATE_STATE_CONFIG			6
#define BUSPIRATE_STATE_CONFIG_UPDATED		7

#define BUSPIRATE_CMD_RESET			'#'
#define BUSPIRATE_CMD_ENTER			'\n'
#define BUSPIRATE_CMD_START			0x0
#define BUSPIRATE_CMD_I2C			0x2
#define BUSPIRATE_CMD_BB_RESET			0xF
#define BUSPIRATE_CMD_CONFIG			0x40

#define BUSPIRATE_I2C_START			0x2
#define BUSPIRATE_I2C_STOP			0x3
#define BUSPIRATE_I2C_READ			0x4
#define BUSPIRATE_I2C_ACK			0x6
#define BUSPIRATE_I2C_NACK			0x7
#define BUSPIRATE_I2C_WRITE_READ		0x8
#define BUSPIRATE_I2C_WRITE_BULK		0x10

#define BUSPIRATE_CONFIG_PSU_MASK		(1<<3)
#define BUSPIRATE_CONFIG_PULLUP_MASK		(1<<2)

#define WAIT_MSEC				50

static DECLARE_WAIT_QUEUE_HEAD(wq);

struct buspirate_data {
	struct i2c_adapter adapter;
	struct i2c_client *client;
	int state;
	u8 addr;		/* last used address */
	unsigned char buffer[BUSPIRATE_BUFFER_SIZE];
	unsigned int pos;	/* position inside the buffer */
	u8 config;
	u8 last_byte;
};

static int buspirate_mode_reset(struct serio *serio)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);
	int i;

	buspirate->state = BUSPIRATE_STATE_NONE;

	for (i = 0; i < 10; i++) {
		serio_write(serio, BUSPIRATE_CMD_ENTER);
		msleep(20);
	}

	serio_write(serio, BUSPIRATE_CMD_RESET);
	msleep(50);
	serio_write(serio, BUSPIRATE_CMD_ENTER);
	msleep(50);

	return 1;
}

static int buspirate_mode_binary(struct serio *serio)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);
	int i;

	if (buspirate->state != BUSPIRATE_STATE_NONE &&
		buspirate->state != BUSPIRATE_STATE_I2C)
		return -EINVAL;

	buspirate->state = BUSPIRATE_STATE_INIT;

	for (i = 0; i < 20; i++) {
		serio_write(serio, BUSPIRATE_CMD_START);
		wait_event_interruptible_timeout(wq,
				buspirate->state == BUSPIRATE_STATE_BBIO,
				msecs_to_jiffies(WAIT_MSEC));
		if (buspirate->state == BUSPIRATE_STATE_BBIO)
			break;
	}

	if (buspirate->state != BUSPIRATE_STATE_BBIO) {
		dev_err(&serio->dev,
			"%s failed (state=%d, pos=%d)\n", __func__,
			buspirate->state, buspirate->pos);
		return -EINVAL;
	}

	return 1;
}

static int buspirate_mode_i2c(struct serio *serio)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);
	int i;

	if (buspirate->state != BUSPIRATE_STATE_BBIO)
		return -EINVAL;

	for (i = 0; i < 20; i++) {
		serio_write(serio, BUSPIRATE_CMD_I2C);
		wait_event_interruptible_timeout(wq,
				buspirate->state == BUSPIRATE_STATE_I2C,
				msecs_to_jiffies(WAIT_MSEC));
		if (buspirate->state == BUSPIRATE_STATE_I2C)
			break;
	}

	if (buspirate->state != BUSPIRATE_STATE_I2C) {
		dev_err(&serio->dev, "%s failed (state=%d, pos=%d)\n", __func__,
			buspirate->state, buspirate->pos);
		return -EINVAL;
	}

	return 1;
}

static int buspirate_write_config(struct serio *serio, unsigned int val,
					unsigned int mask)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);

	if (buspirate->state != BUSPIRATE_STATE_I2C)
		return -EINVAL;

	buspirate->state = BUSPIRATE_STATE_CONFIG;
	buspirate->config = buspirate->config & ~mask;
	serio_write(serio, buspirate->config);
	wait_event_interruptible_timeout(wq,
			buspirate->state == BUSPIRATE_STATE_CONFIG_UPDATED,
			msecs_to_jiffies(WAIT_MSEC));

	if (buspirate->state != BUSPIRATE_STATE_CONFIG_UPDATED)
		goto error;

	buspirate->state = BUSPIRATE_STATE_CONFIG;
	buspirate->config = buspirate->config | val * mask;
	serio_write(serio, buspirate->config);
	wait_event_interruptible_timeout(wq,
			buspirate->state == BUSPIRATE_STATE_CONFIG_UPDATED,
			msecs_to_jiffies(WAIT_MSEC));

	if (buspirate->state != BUSPIRATE_STATE_CONFIG_UPDATED)
		goto error;

	buspirate->state = BUSPIRATE_STATE_I2C;

	return 1;

error:
	buspirate->state = BUSPIRATE_STATE_I2C;
	dev_err(&serio->dev,
		"%s failed (state=%d, pos=%d)\n", __func__, buspirate->state,
		buspirate->pos);
	return -EINVAL;
}

static int buspirate_i2c_cmd_check(struct serio *serio, u8 cmd, u8 addr,
					u16 len, u8 *buf)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);
	int i;

	if (buspirate->state != BUSPIRATE_STATE_I2C)
		return -EINVAL;

	if (cmd == BUSPIRATE_I2C_WRITE_BULK && len && buf)
		cmd |= len;

	if (cmd != BUSPIRATE_I2C_READ) {
		buspirate->state = BUSPIRATE_STATE_CMD;
		serio_write(serio, cmd);
		wait_event_interruptible_timeout(wq,
			buspirate->state == BUSPIRATE_STATE_CMDREPLY,
			msecs_to_jiffies(WAIT_MSEC));

		if (buspirate->state != BUSPIRATE_STATE_CMDREPLY ||
				buspirate->last_byte != 0x1)
			goto error;
	}

	if ((cmd & BUSPIRATE_I2C_WRITE_BULK) && addr) {
		buspirate->state = BUSPIRATE_STATE_CMD;
		serio_write(serio, addr);
		wait_event_interruptible_timeout(wq,
			buspirate->state == BUSPIRATE_STATE_CMDREPLY,
			msecs_to_jiffies(WAIT_MSEC));

		// checking last_byte is not needed here cause
		// the returned byte is the ACK/NACK from
		// the i2c protocol in this case
		if (buspirate->state != BUSPIRATE_STATE_CMDREPLY)
			goto error;
	}

	if ((cmd & BUSPIRATE_I2C_WRITE_BULK) && len && buf) {
		for (i = 0; i < len; i++) {
			buspirate->state = BUSPIRATE_STATE_CMD;
			serio_write(serio, buf[i]);
			wait_event_interruptible_timeout(wq,
				buspirate->state == BUSPIRATE_STATE_CMDREPLY,
				msecs_to_jiffies(WAIT_MSEC));

			// checking last_byte not needed here cause
			// the returned byte is the ACK/NACK from
			// the i2c protocol in this case
			if (buspirate->state != BUSPIRATE_STATE_CMDREPLY)
				goto error;
		}
		buspirate->state = BUSPIRATE_STATE_CMD;
	}

	if (cmd == BUSPIRATE_I2C_READ && len) {
		for (i = 0; i < len; i++) {
			buspirate->state = BUSPIRATE_STATE_CMD;
			serio_write(serio, BUSPIRATE_I2C_READ);
			wait_event_interruptible_timeout(wq,
				buspirate->state == BUSPIRATE_STATE_CMDREPLY,
				msecs_to_jiffies(WAIT_MSEC));

			// checking last_byte not needed here cause
			// the returned byte is the data byte
			if (buspirate->state != BUSPIRATE_STATE_CMDREPLY)
				goto error;

			buf[i] = buspirate->last_byte;
			buspirate->state = BUSPIRATE_STATE_CMD;

			if (i < len-1)
				serio_write(serio, BUSPIRATE_I2C_ACK);
			else
				serio_write(serio, BUSPIRATE_I2C_NACK);

			wait_event_interruptible_timeout(wq,
				buspirate->state == BUSPIRATE_STATE_CMDREPLY,
				msecs_to_jiffies(WAIT_MSEC));

			if (buspirate->state != BUSPIRATE_STATE_CMDREPLY ||
				buspirate->last_byte != 0x1) {
				goto error;
			}
		}
	}

	buspirate->state = BUSPIRATE_STATE_I2C;
	return 1;

error:
	buspirate->state = BUSPIRATE_STATE_I2C;
	dev_err(&serio->dev,
		"%s failed (state=%d, pos=%d, cmd=%x, last_byte=%d)\n",
		__func__, buspirate->state, buspirate->pos, cmd,
		buspirate->last_byte);
	return -EINVAL;
}

static int buspirate_i2c_write_data(struct serio *serio, u8 addr, u16 len,
					u8 *buf)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);

	if (buspirate->state != BUSPIRATE_STATE_I2C)
		return -EINVAL;

	if (!buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_START, 0, 0, 0))
		goto error;
	if (!buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_WRITE_BULK, addr,
					len, buf))
		goto error;
	if (!buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_STOP, 0, 0, 0))
		goto error;

	return 1;

error:
	buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_STOP, 0, 0, 0);
	return -EPROTO;
}

static int buspirate_i2c_read_data(struct serio *serio, u8 addr, u16 len,
					u8 *buf)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);

	if (buspirate->state != BUSPIRATE_STATE_I2C)
		return -EINVAL;

	if (!buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_START, 0, 0, 0))
		goto error;
	if (!buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_WRITE_BULK,
								addr, 0, 0))
		goto error;
	if (!buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_READ, addr, len, buf))
		goto error;
	if (!buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_STOP, 0, 0, 0))
		goto error;

	return 1;

error:
	buspirate_i2c_cmd_check(serio, BUSPIRATE_I2C_STOP, 0, 0, 0);
	return -EPROTO;
}

static int buspirate_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
				int num)
{
	struct serio *serio = adapter->algo_data;
	struct i2c_msg *pmsg;
	int i;

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		dev_dbg(&serio->dev, "%s (addr=%x, flags=%x, len=%d, buf=%.*s)",
			__func__, pmsg->addr, pmsg->flags, pmsg->len, pmsg->len,
			pmsg->buf);
		if (pmsg->flags & I2C_M_RD) {
			if (!buspirate_i2c_read_data(serio, (pmsg->addr<<1) + 1,
				pmsg->len, pmsg->buf))
				return -EREMOTEIO;
		} else {
			if (!buspirate_i2c_write_data(serio, pmsg->addr<<1,
				pmsg->len, pmsg->buf))
				return -EREMOTEIO;
		}
	}
	return i;
}

static u32 buspirate_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static ssize_t psu_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{

	struct i2c_adapter *adapter = to_i2c_adapter(dev);
	struct serio *serio = adapter->algo_data;

	unsigned int value;
	int ret;

	ret = kstrtouint(buf, 10, &value);

	switch (value) {
	case 0:
	case 1:
		buspirate_write_config(serio, value,
					BUSPIRATE_CONFIG_PSU_MASK);
	break;
	default:
		return -EINVAL;
	}

	return count;
}

static ssize_t psu_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct i2c_adapter *adapter = to_i2c_adapter(dev);
	struct serio *serio = adapter->algo_data;
	struct buspirate_data *buspirate = serio_get_drvdata(serio);

	return sprintf(buf, "%d\n",
			(buspirate->config & BUSPIRATE_CONFIG_PSU_MASK) >> 3);
}

static ssize_t pullup_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{

	struct i2c_adapter *adapter = to_i2c_adapter(dev);
	struct serio *serio = adapter->algo_data;

	unsigned int value;
	int ret;

	ret = kstrtouint(buf, 10, &value);

	switch (value) {
	case 0:
	case 1:
		buspirate_write_config(serio, value,
					BUSPIRATE_CONFIG_PULLUP_MASK);
	break;
	default:
		return -EINVAL;
	}

	return count;
}

static ssize_t pullup_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_adapter *adapter = to_i2c_adapter(dev);
	struct serio *serio = adapter->algo_data;
	struct buspirate_data *buspirate = serio_get_drvdata(serio);

	return sprintf(buf, "%d\n",
		(buspirate->config & BUSPIRATE_CONFIG_PULLUP_MASK) >> 2);
}

static DEVICE_ATTR_RW(psu);
static DEVICE_ATTR_RW(pullup);

static struct attribute *buspirate_attrs[] = {
	&dev_attr_psu.attr,
	&dev_attr_pullup.attr,
	NULL
};

ATTRIBUTE_GROUPS(buspirate);

static const struct i2c_algorithm buspirate_algorithm = {
	.master_xfer	= buspirate_xfer,
	.functionality	= buspirate_func,
};

static irqreturn_t buspirate_interrupt(struct serio *serio, unsigned char data,
				  unsigned int flags)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);

	if (buspirate->pos == BUSPIRATE_BUFFER_SIZE-1)
		buspirate->pos = 0;

	switch (buspirate->state) {
	case BUSPIRATE_STATE_INIT:
		buspirate->buffer[buspirate->pos++] = data;
		if (buspirate->pos >= 5 &&
			strncmp(&buspirate->buffer[buspirate->pos-5], "BBIO1",
								5) == 0) {
				buspirate->state = BUSPIRATE_STATE_BBIO;
				buspirate->pos = 0;
				wake_up_interruptible(&wq);
		}
		break;
	case BUSPIRATE_STATE_BBIO:
		buspirate->buffer[buspirate->pos++] = data;
		if (buspirate->pos >= 4 &&
			strncmp(&buspirate->buffer[buspirate->pos-4], "I2C1",
								4) == 0) {
				buspirate->state = BUSPIRATE_STATE_I2C;
				buspirate->pos = 0;
				wake_up_interruptible(&wq);
		}
		break;
	case BUSPIRATE_STATE_CONFIG:
		buspirate->buffer[buspirate->pos++] = data;
		if (buspirate->pos >= 1) {
			buspirate->state = BUSPIRATE_STATE_CONFIG_UPDATED;
			buspirate->pos = 0;
			wake_up_interruptible(&wq);
		}
		break;
	case BUSPIRATE_STATE_CMD:
		buspirate->buffer[buspirate->pos++] = data;
		if (buspirate->pos >= 1) {
			buspirate->last_byte =
					buspirate->buffer[buspirate->pos-1];
			buspirate->state = BUSPIRATE_STATE_CMDREPLY;
			buspirate->pos = 0;
			wake_up_interruptible(&wq);
		}
		break;
	}

	dev_dbg(&serio->dev, "%s: (state=%d, last=%x, pos=%d, buf=%s)\n",
		__func__, buspirate->state, buspirate->last_byte,
		buspirate->pos, buspirate->buffer);

	return IRQ_HANDLED;
}

static int buspirate_connect(struct serio *serio, struct serio_driver *drv)
{
	struct buspirate_data *buspirate;
	struct i2c_adapter *adapter;
	int err;

	buspirate = devm_kzalloc(&serio->dev, sizeof(struct buspirate_data),
			GFP_KERNEL);
	if (!buspirate)
		return -ENOMEM;

	buspirate->config = 0x40;

	err = serio_open(serio, drv);
	if (err)
		return -ENODEV;

	adapter = &buspirate->adapter;
	adapter->owner = THIS_MODULE;
	adapter->algo = &buspirate_algorithm;
	adapter->algo_data = serio;
	adapter->dev.parent = &serio->dev;
	adapter->dev.groups = buspirate_groups;
	strlcpy(adapter->name, "Bus Pirate adapter", sizeof(adapter->name));

	/* Bus pirate init sequence*/
	if (!buspirate_mode_reset(serio))
		return -ENODEV;
	if (!buspirate_mode_binary(serio))
		return -ENODEV;
	if (!buspirate_mode_i2c(serio))
		return -ENODEV;

	serio_set_drvdata(serio, buspirate);

	err = i2c_add_adapter(adapter);
	if (err)
		goto exit;

	dev_info(&serio->dev, "connected to the Bus Pirate!\n");

	return 0;

 exit:
	serio_close(serio);
	return err;
}

static void buspirate_disconnect(struct serio *serio)
{
	struct buspirate_data *buspirate = serio_get_drvdata(serio);

	if (buspirate->client)
		i2c_unregister_device(buspirate->client);
	i2c_del_adapter(&buspirate->adapter);
	serio_close(serio);
}

static struct serio_device_id buspirate_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_BUSPIRATE,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};
MODULE_DEVICE_TABLE(serio, buspirate_serio_ids);

static struct serio_driver buspirate_drv = {
	.driver		= {
		.name	= "buspirate",
	},
	.description	= "bus pirate driver",
	.id_table	= buspirate_serio_ids,
	.connect	= buspirate_connect,
	.disconnect	= buspirate_disconnect,
	.interrupt	= buspirate_interrupt,
};

module_serio_driver(buspirate_drv);

MODULE_AUTHOR("Angelo Compagnucci <angelo.compagnucci@gmail.com>");
MODULE_DESCRIPTION("bus pirate driver");
MODULE_LICENSE("GPL");
