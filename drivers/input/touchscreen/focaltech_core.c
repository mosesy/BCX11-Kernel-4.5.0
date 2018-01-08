/* 
 * drivers/input/touchscreen/FT5x26.c
 *
 * FocalTech FT5X26 TouchScreen driver. 
 *
 * Copyright (c) 2014  FocalTech  Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * Vresion   Purpose                                                Date
 *   1.1     Modify upgradefirmware func for shortransmit time. 2014/04/16
 *   1.2     Modify Compal Touch Module for FT5X26              2015/05/24
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/input.h>
//#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/acpi.h>
#include <linux/firmware.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/input/mt.h>
#include "focaltech_core.h"

#define TOUCH_MAX_POINT 10 

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];   /*x coordinate */
    u16 au16_y[CFG_MAX_TOUCH_POINTS];   /*y coordinate */
    u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];   /*touch event:
														                                                   0 -- down; 1-- contact; 2 -- contact */
    u8 au8_finger_id[CFG_MAX_TOUCH_POINTS]; /*touch ID */
    u16 pressure;
    u8 touch_point;

};

struct fts_ts_data {
	unsigned int irq;
	unsigned int reset;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client; 
	struct input_dev	*input_dev;
	struct ts_event		event;
#ifdef CONFIG_PM
	//struct early_suspend    early_suspend;
#endif
	struct work_struct 	pen_event_work;
	struct workqueue_struct *wq;
	struct fts_platform_data *pdata;
};

u8 au8_finger_id_old[CFG_MAX_TOUCH_POINTS] = { 0xf };

static struct fts_ts_data *fts_wq_data;
/*  
 *  *fts_i2c_Read-read data and write data by i2c
 *  *@client: handle of i2c
 *  *@writebuf: Data that will be written to the slave
 *  *@writelen: How many bytes to write
 *  *@readbuf: Where to store data read from slave
 *  *@readlen: How many bytes to read
 *  *   
 *  *Returns negative errno, else the number of messages executed
 *  *   
 *  *   
 *  */ 
int fts_i2c_Read(struct i2c_client *client, char *writebuf,
	int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

/*
 * 1. Open function for character device driver
 * 2. Return Status 
*/
static int fts_file_open(struct inode *inode , struct file *filp){
	printk("TS Debug FocalTech File Open\n");
	return 0;
}

/*
 * 1. Read function for character device driver.
 * 2. Let functions in user space to read and operate it.
*/
static ssize_t fts_file_read(struct file *filp , char *buf , size_t count , loff_t *f_pos){
	int ret=0;
	return ret;
}
/*
 * 1.Close Function for Character device driver
 * 2.Return Status
*/
static int fts_file_close (struct inode *inode , struct file *filp){
	printk("TS Debug -FocalTech File Close Module\n");
	return 0;
}

/*
 * 1. Write Funtion for character device driver
 * 2. Let functions in user space to write and operate it.
*/

static ssize_t fts_file_write( struct file *filp,const char *buf , size_t count, loff_t *f_pos){
	int ret;
	char *tmp;
	struct i2c_msg msg[1]; 
	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count,GFP_KERNEL);
	if(tmp==NULL) return -ENOMEM;
					       
    	if (copy_from_user(tmp, buf, count)) {
		return -EFAULT;
    	}

	msg[0].addr = fts_wq_data->client->addr;
	msg[0].flags = 0;
 	msg[0].len = count;
        msg[0].buf = tmp;
    	ret = i2c_transfer(fts_wq_data->client->adapter,msg,1);						       
        kfree(tmp);
	
        return ret;
}


/* File Operation*/
struct file_operations FocalTech_fops = {
	.owner = THIS_MODULE,
	.open = fts_file_open,
	.release = fts_file_close,
	.read = fts_file_read,
	.write = fts_file_write,
};

static void fts_report_value(struct fts_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i=0,j=0,k=0;
	int uppoint = 0;

	/*protocol b*/
	
	for (i = 0; i < event->touch_point; i++) {

		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i] == 0|| event->au8_touch_event[i] == 2)
		{
			input_mt_report_slot_state(data->input_dev,MT_TOOL_FINGER,true);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,event->pressure);
		
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,event->au16_y[i]);

		}
		else
		{
			uppoint++;
			input_mt_report_slot_state(data->input_dev,MT_TOOL_FINGER,false);
		}
	}

	for(i=0; i < event->touch_point; i++)
	{
		for(j=0; j < CFG_MAX_TOUCH_POINTS;j++)
		{
			if(au8_finger_id_old[j] == event->au8_finger_id[i])
				au8_finger_id_old[j]= 0xf;
		}
	}

	for(j=0; j < CFG_MAX_TOUCH_POINTS; j++)
	{
		if(au8_finger_id_old[j] != 0xf)
		{
			printk("[FocalTech] Report ID Error:%2x\n",au8_finger_id_old[j]);
			input_mt_slot(data->input_dev, au8_finger_id_old[j]);
			input_mt_report_slot_state(data->input_dev,MT_TOOL_FINGER,false);
		}
	}

	memset(au8_finger_id_old,0x0f,sizeof(au8_finger_id_old));
	
	for(i=0; i < CFG_MAX_TOUCH_POINTS; i++)
	{
		if(event->au8_touch_event[i] == 0 || event->au8_touch_event[i] == 2)
		{
			au8_finger_id_old[k] = event->au8_finger_id[k];
			k++;
		}
	}

	
	if(event->touch_point == uppoint)
	{
 	       for(i=0;i<CFG_MAX_TOUCH_POINTS;i++)
 	       {
 	              input_mt_slot(data->input_dev,i);
 	              input_mt_report_slot_state(data->input_dev,MT_TOOL_FINGER,false);
               }

		input_report_key(data->input_dev, BTN_TOUCH, 0);
	}
	else
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	input_sync(data->input_dev);
	
}	


static int fts_read_Touchdata(struct fts_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = fts_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	
	memset(event, 0xf, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS ; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
			(((s16) buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i])& 0xFF);
		event->au16_y[i] =
			(((s16) buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i]) & 0xFF);
		event->au8_touch_event[i] =
			buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
			(buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}

	//event->pressure = FT_PRESS;	
	event->pressure = 200;

	return 0;
}

static irqreturn_t fts_ts_interrupt(int irq, void *dev_id)
{
	struct fts_ts_data *fts_wq_data =	dev_id;
	int ret = 0;
	disable_irq_nosync(fts_wq_data->irq);
        ret = fts_read_Touchdata(fts_wq_data);
        if (ret == 0) {
	        fts_report_value(fts_wq_data);
        }
        else
	        printk("[FocalTech Debug] data package read error\n");

	
	enable_irq(fts_wq_data->irq);

	return IRQ_HANDLED;

}

void fts_free_fingers(struct fts_ts_data *data)
{
	int i=0;
	for(i=0;i<CFG_MAX_TOUCH_POINTS;i++)
	{
		input_mt_slot(data->input_dev,i);
		input_mt_report_slot_state(data->input_dev,MT_TOOL_FINGER,false);
	}
	input_sync(data->input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fts_ts_suspend(struct early_suspend *handler)
{
	struct fts_ts_data *ts = fts_wq_data;
	printk("==fts_ts_suspend=\n");
	//disable_irq(ts->irq);
	//msleep(20);
	fts_free_fingers(ts);
}

static void fts_ts_resume(struct early_suspend *handler)
{
	struct fts_ts_data *ts = fts_wq_data;
 	u8 auc_i2c_write_buf[3] = {0xEB,0xAA,0x09};
	u8 reg_val[3]= {0};
	printk("==fts_ts_resume=\n");
#if 0
	gpio_direction_output(ts->reset,0);
        msleep(20);
        gpio_set_value(ts->reset,1);
        msleep(20);
        gpio_direction_output(ts->reset,0);
	fts_i2c_Write(ts->client, auc_i2c_write_buf, 3);
	fts_i2c_Read(ts->client, auc_i2c_write_buf, 0, reg_val, 3);
	enable_irq(ts->irq);
#endif

}
#endif  //CONFIG_HAS_EARLYSUSPEND

static void resetTouch(int time)
{
	 gpio_set_value(FTS_RESET_GPIO,1);
	 msleep(time);
	 gpio_direction_output(FTS_RESET_GPIO,0);
	 msleep(time);
	 gpio_set_value(FTS_RESET_GPIO,1);
}


int HidI2c_To_StdI2c(struct i2c_client * client)
{
	int retry =0;
	u8 auc_i2c_write_buf[10] = {0};
	u8 reg_val[10] = {0};
	int iRet = 0;

	auc_i2c_write_buf[0] = 0xEB;
	auc_i2c_write_buf[1] = 0xAA;
	auc_i2c_write_buf[2] = 0x09;

	for(retry =0;retry < 5;retry++)
	{
    	reg_val[0] = reg_val[1] =  reg_val[2] = 0x00;
        iRet = fts_i2c_Write(client, auc_i2c_write_buf, 3);
	    msleep(50);
    	iRet = fts_i2c_Read(client, auc_i2c_write_buf, 0, reg_val, 3);
    	printk("Brandy HidtoI2C,REG1= 0x%x,REG2=0x%x,REG3=0x%x, iRet=%d\n",
		reg_val[0], reg_val[1], reg_val[2], iRet);	

		if (reg_val[0] == 0xEB && reg_val[1] == 0xAA && reg_val[2] == 0x08) 
		{
			dev_dbg(&client->dev, "HidI2c_To_StdI2c successful.\n");
			return 1;
		}
		else
		{
			pr_err("HidI2c_To_StdI2c error.\n");
			iRet = 0;
		}
	}
	return iRet;
}
static int fts_init_gpio_hw(struct fts_ts_data *fts_wq_data)
{
	int ret = 0;

	ret = gpio_request(FTS_RESET_PIN, FTS_RESET_PIN_NAME);
	if (ret) {
		pr_err("%s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTS_RESET_PIN_NAME, ret);
		return ret;
	}
		
	gpio_direction_output(FTS_RESET_PIN, 1);	//reset set high
	pr_info("[ftxxxx] gpio_request reset gpio Num: %d\n", FTS_RESET_PIN);

	ret = gpio_request(FTS_INT_PIN, FTS_INT_PIN_NAME);
	if (ret) {
		pr_err("%s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTS_INT_PIN_NAME, ret);
		return ret;
	}
	gpio_direction_input(FTS_INT_PIN);
	pr_info("[ftxxxx] gpio_request int gpio Num: %d\n", FTS_INT_PIN);

	return ret;
}


static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fts_platform_data *pdata = (struct fts_platform_data *)client->dev.platform_data;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	int flag=0;
	printk("[FocalTech]: fts_ts Probe Function Start\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	fts_wq_data = kzalloc(sizeof(*fts_wq_data), GFP_KERNEL);
	if (!fts_wq_data)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

#if 0
	fts_wq_data->irq = pdata->gpio_irq;
	fts_wq_data->client = client;
	fts_wq_data->pdata = pdata;
	fts_wq_data->x_max = pdata->screen_max_x - 1;
	fts_wq_data->y_max = pdata->screen_max_y - 1;
	fts_wq_data->reset = FTS_RESET_GPIO;
#endif
	
	fts_wq_data->client = client;
	fts_wq_data->x_max = TOUCH_MAX_X - 1;
	fts_wq_data->y_max = TOUCH_MAX_Y - 1;
	fts_wq_data->irq = FTS_INT_PIN;
	fts_wq_data->reset = FTS_RESET_GPIO; 
	client->irq = gpio_to_irq(fts_wq_data->irq);
	
	fts_init_gpio_hw(fts_wq_data);	
	
	fts_wq_data->client = client;
	i2c_set_clientdata(client, fts_wq_data);

    err = request_threaded_irq(client->irq, NULL, fts_ts_interrupt,IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,fts_wq_data);

	if (err < 0) {
		dev_err(&client->dev, "ft5x26_probe: request irq failed\n");
		printk("[FocalTech]: Request Irq Error\n");
		goto exit_irq_request_failed;
	}else{
		printk("[FocalTech]: Request IRQ PASS\n");
	}

	printk("[FocalTech]: input_allocate_device\n");
	disable_irq(client->irq);
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	fts_wq_data->input_dev = input_dev;

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS, 0);
	input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0,TOUCH_MAX_X , 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0,TOUCH_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name		= FTS_NAME;		
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"fts_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		printk("[FocalTech]: Input Register Error");
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	fts_wq_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	fts_wq_data->early_suspend.suspend = fts_ts_suspend;
	fts_wq_data->early_suspend.resume	= fts_ts_resume;
	register_early_suspend(&fts_wq_data->early_suspend);
#endif
	/*reset fts touch*/
	resetTouch(20);
	msleep(100);	
	HidI2c_To_StdI2c(client);
	msleep(20);

	/*get some register information */
	uc_reg_addr = FTS_REG_FW_VER;
	flag = fts_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk("[FocalTech] Firmware version = 0x%x\n", uc_reg_value);
	if(flag<0)
		printk("[fts] chip does not exits\n");

	uc_reg_addr = FTS_REG_POINT_RATE;
	fts_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk("[FocalTech] report rate is %dHz.\n", uc_reg_value * 10);

	uc_reg_addr = FTS_REG_THGROUP;
	fts_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk("[FocalTech] touch threshold is %d.\n", uc_reg_value * 4);

	uc_reg_addr = FTS_REG_VENDOR_ID;
	fts_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk("[Focal] VENDOR ID = 0x%x\n", uc_reg_value);
	enable_irq(client->irq);
    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, fts_wq_data);
exit_irq_request_failed:
	cancel_work_sync(&fts_wq_data->pen_event_work);
	destroy_workqueue(fts_wq_data->wq);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(fts_wq_data);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int fts_ts_remove(struct i2c_client *client)
{
	printk("[FocalTech]: ft5x26_remove=\n");
        struct fts_ts_data *fts_wq_data;
        fts_wq_data = i2c_get_clientdata(client);
        input_unregister_device(fts_wq_data->input_dev);
        free_irq(client->irq, fts_wq_data);
        kfree(fts_wq_data);
        i2c_set_clientdata(client, NULL);
	return 0;
}

#if CONFIG_ACPI
static struct acpi_device_id fts_acpi_match[] = {
    {"FTSC1000", 0},
    {},
};
MODULE_DEVICE_TABLE(acpi, fts_acpi_match);
#endif

static const struct i2c_device_id fts_ts_id[] = {
	{ FTS_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, fts_ts_id);

static struct i2c_driver fts_ts_driver = {
	.probe		= fts_ts_probe,
	.remove		= fts_ts_remove,
	.id_table	= fts_ts_id,
	.driver	= {
		.name	= FTS_NAME,
		.owner	= THIS_MODULE,
		.acpi_match_table = ACPI_PTR(fts_acpi_match),
	},
};
module_i2c_driver(fts_ts_driver);

#if 0
static int __init fts_ts_init(void)
{
	int ret;
		
	printk("[Focal][Touch] Android fts_ts_init !\n");
						
													
	ret = i2c_add_driver(&fts_ts_driver);
	if (ret) 
	{
		printk(KERN_WARNING "Adding focaltech driver failed errno = %d\n", ret);
	}else
	{
		pr_info("Successfully added driver %s\n",fts_ts_driver.driver.name);
	}

	return ret;
}

static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

struct fts_platform_data fts_data = 
{
	.gpio_irq = FTS_INT_PIN,		// IRQ port
	.irq_cfg = IRQF_TRIGGER_RISING,
	.gpio_reset = FTS_RESET_PIN,		// Reset support
	.screen_max_x = TOUCH_MAX_X,
	.screen_max_y = TOUCH_MAX_Y
};

static struct i2c_board_info __initdata fts_i2c_device = {
	I2C_BOARD_INFO(FTS_NAME, FTS_I2C_ADDR),
	.platform_data = &fts_data,
};

static int __init fts_platform_init(void)
{
	int i2c_busnum = 4;
	printk(KERN_INFO "%s:\n", __func__);
	fts_i2c_device.irq = gpio_to_irq(fts_data.gpio_irq);
	return i2c_register_board_info(i2c_busnum, &fts_i2c_device, 1);
}
fs_initcall(fts_platform_init);


module_init(fts_ts_init);
module_exit(fts_ts_exit);
#endif

MODULE_AUTHOR("<Brandy.Liu>");
MODULE_DESCRIPTION("FocalTech FT5x26 TouchScreen driver");
MODULE_LICENSE("GPL");
