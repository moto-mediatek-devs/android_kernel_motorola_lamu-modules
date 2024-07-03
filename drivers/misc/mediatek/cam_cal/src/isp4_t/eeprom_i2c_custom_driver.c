// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"
#include <linux/dma-mapping.h>
#if IS_ENABLED(CONFIG_COMPAT)
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define EEPROM_I2C_MSG_SIZE_READ 2

static DEFINE_SPINLOCK(g_spinLock);
static struct i2c_client *g_pstI2CclientG;

extern struct gc08a8_otp_t gc08a8_otp_info;
struct gc08a8_otp_t {
	u8  module_flag;
	u8  module_param[18];
	u8  moduleChksum;
	u8  awb_flag;
	u8  awb_param[12];
	u8  awbChksum;
	u8  lsc_flag;
	u8  lsc_param[1868];
	u8  lscChksum;
};
/************************************************************
 * I2C read function (Custom)
 * Customer's driver can put on here
 * Below is an example
 ************************************************************/
 #define PAGE_SIZE_ 256
static int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int  i4RetValue = 0;
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

	spin_lock(&g_spinLock);
	g_pstI2CclientG->addr = (i2cId >> 1);
	spin_unlock(&g_spinLock);

	msg[0].addr = g_pstI2CclientG->addr;
	msg[0].flags = g_pstI2CclientG->flags & I2C_M_TEN;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = g_pstI2CclientG->addr;
	msg[1].flags = g_pstI2CclientG->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue = i2c_transfer(g_pstI2CclientG->adapter,
				msg,
				EEPROM_I2C_MSG_SIZE_READ);

	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
		pr_debug("I2C read failed!!\n");
		return -1;
	}
	return 0;
}

static int custom_read_region(u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	u8 *buff = data;
	u32 size_to_read = size;

	int ret = 0;

	while (size_to_read > 0) {
		u8 page = addr / PAGE_SIZE_;
		u8 offset = addr % PAGE_SIZE_;
		char *Buff = data;

		if (iReadRegI2C(&offset, 1, (u8 *)Buff, 1,
			i2c_id + (page << 1)) < 0) {
			pr_debug("fail addr=0x%x 0x%x, P=%d, offset=0x%x",
				addr, *Buff, page, offset);
			break;
		}
		addr++;
		buff++;
		size_to_read--;
		ret++;
	}
	pr_debug("addr =%x size %d data read = %d\n", addr, size, ret);
	return ret;
}



unsigned int Custom_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	g_pstI2CclientG = client;
	if (custom_read_region(addr, data, g_pstI2CclientG->addr, size) == 0)
		return size;
	else
		return 0;
}

/*begin 20240702 add for otp check*/
unsigned int gc08a8_dd_p329_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{
    int i=0;

    pr_err("[yy]addr =%x size %d\n", addr, size);
	pr_err("module id = 0x%x", gc08a8_otp_info.module_param[0]);
	pr_err("Year = 0x%x", gc08a8_otp_info.module_param[1]);

	if (addr == 0x6)
    {
        *(u32 *)data = 0x00000088;
		pr_err("gc08a8_read_region [yy]addr =%x data = %x  size = %x\n", addr, *(u32 *)data, size);
    }

    else if (addr == 0x1)
    {
        if ((gc08a8_otp_info.lsc_flag == 0x01) || (gc08a8_otp_info.lsc_flag == 0x04))
        {
            pr_err("[yy]lsc_flag valid\n");
            for(i=0; i<size; i++){
                data[i] = gc08a8_otp_info.lsc_param[i];
            }
        }
    }
	/*
    else if (addr == 0x2)
    {
        if ((gc08a8_otp_info.awb_flag == 0x01) || (gc08a8_otp_info.awb_flag == 0x04))
        {
            *data = 1;
        }
    }
	*/
    else if (addr == 0x3)
    {
        for(i=0; i<size; i++){
            data[i] = gc08a8_otp_info.awb_param[i];
            pr_err("[yy]gc08a8_read_region awb data[%d] =%x \n", i, data[i]);
        }
    }
    else
    {
        pr_err("add = 0x%x, size = %d ,read error !!!\n",addr,size);
    }

    return size;
}

/*end 20240702 add for otp check*/
