/*
 *
 * Copyright 2011 Texas Instruments, Inc.
 * Author: Archit Taneja <archit@ti.com>
 *
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
 *
 * This program iss free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if 0

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>

#include "dsi85.h"

#define DSI85DEBUG

static struct i2c_board_info dsi85_i2c_board_info =
{
	I2C_BOARD_INFO("dsi85_i2c_driver", 0x2d /*0x2c*/),
};

struct dsi85_i2c_data
{
	struct mutex xfer_lock;
};



static struct omap_video_timings dsi85_timings =
{
	.x_res = 1024,
	.y_res = 600,
	.pixel_clock = 50000,
	.hfp = 36,  //42 128 38 - 100 1 107  chido es 36 128  45
	.hsw = 128,
	.hbp = 45,
	.vfp = 1,
	.vsw = 4,
	.vbp = 9,
};

static const struct omap_dss_dsi_videomode_data vm_data =
{
	.hsa = 1,
	.hfp = 20,   // Set to 4/3 of the DISPC porch value.
	.hbp = 20,   // Set to 4/3 of the DISPC porch value.
	.vsa = 4,
	.vfp = 6,
	.vbp = 4,

	.blanking_mode = 1,
	.hsa_blanking_mode = 1,
	.hbp_blanking_mode = 1,
	.hfp_blanking_mode = 1,

	.vp_de_pol = true,
	.vp_vsync_pol = false,
	.vp_hsync_pol = false,
	.vp_hsync_end = false,
	.vp_vsync_end = false,

	.window_sync = 4,
	.ddr_clk_always_on = true,
};

struct dsi85_data
{
	struct mutex lock;

	struct omap_dss_device* dssdev;
	struct backlight_device* bldev;
	struct dentry* debug_dir;
	bool enabled;
	u8 rotate;
	bool mirror;
	bool use_dsi_bl;
	unsigned int bl;
	unsigned long hw_guard_end;   /* next value of jiffies when we can
                * issue the next sleep in/out command
                */
	unsigned long hw_guard_wait;  /* max guard time in jiffies */

	int config_channel;
	int pixel_channel;

	struct omap_video_timings* timings;

	struct panel_dsi85_data* pdata;
	struct i2c_client* dsi85_i2c_client;
};

struct dsi85_reg
{
	/* Address and register value */
	u8 data[10];
	int len;
};


static void dsi85_get_timings(struct omap_dss_device* dssdev,
                              struct omap_video_timings* timings)
{
	*timings = dssdev->panel.timings;
}

static void dsi85_set_timings(struct omap_dss_device* dssdev,
                              struct omap_video_timings* timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int dsi85_check_timings(struct omap_dss_device* dssdev,
                               struct omap_video_timings* timings)
{
	return 0;
}

static void dsi85_get_resolution(struct omap_dss_device* dssdev,
                                 u16* xres, u16* yres)
{
	struct dsi85_data* dsi85_d = dev_get_drvdata(&dssdev->dev);

	if (dsi85_d->rotate == 0 || dsi85_d->rotate == 2)
	{
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	}
	else
	{
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static int dsi85_probe(struct omap_dss_device* dssdev)
{
	struct i2c_adapter* adapter;
	struct i2c_client* dsi85_i2c_client;
	int ret = 0;
	struct dsi85_data* dsi85_d = NULL;
	dev_dbg(&dssdev->dev, "dsi85_probe\n");

	if (dssdev->data == NULL)
	{
		dev_err(&dssdev->dev, "no platform data!\n");
		return -EINVAL;
	}

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = dsi85_timings;
	dssdev->panel.dsi_vm_data = vm_data;
	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;
	dsi85_d = kzalloc(sizeof(*dsi85_d), GFP_KERNEL);

	if (!dsi85_d)
	{
		return -ENOMEM;
	}

	dsi85_d->dssdev = dssdev;
	dsi85_d->pdata = dssdev->data;

	if (!dsi85_d->pdata)
	{
		dev_err(&dssdev->dev, "Invalid platform data\n");
		ret = -EINVAL;
		goto err;
	}

	mutex_init(&dsi85_d->lock);
	dev_set_drvdata(&dssdev->dev, dsi85_d);
	dsi85_d->debug_dir = debugfs_create_dir("dsi85", NULL);

	if (!dsi85_d->debug_dir)
	{
		dev_err(&dssdev->dev, "failed to create debug dir\n");
		goto err_debugfs;
	}

	ret = omap_dsi_request_vc(dssdev, &dsi85_d->pixel_channel);

	if (ret)
	{
		dev_err(&dssdev->dev, "failed to request pixel update "
		        "channel\n");
		goto err_debugfs;
	}
	else
		printk(KERN_INFO "pixel channel setup for %d\n", \
		       dsi85_d->pixel_channel);

	printk(KERN_INFO "pixel channel %d\n", dsi85_d->pixel_channel);
	ret = omap_dsi_set_vc_id(dssdev, dsi85_d->pixel_channel, 0);

	if (ret)
	{
		dev_err(&dssdev->dev, "failed to set VC_ID for pixel data"
		        " virtual channel\n");
		goto err_pixel_vc;
	}

	adapter = i2c_get_adapter(2);

	if (!adapter)
	{
		printk(KERN_INFO "DSI85-I2C: can't get i2c adapter 2\n");
//		ret = -ENODEV;
//		goto err_pixel_vc;
	}

	dsi85_i2c_client = i2c_new_device(adapter, &dsi85_i2c_board_info);

	if (!dsi85_i2c_client)
	{
		printk(KERN_INFO "DSI85-I2C: can't add i2c device\n");
//		ret = -ENODEV;
//		goto err_pixel_vc;
	}

	dsi85_d->dsi85_i2c_client = dsi85_i2c_client;
	ret = i2c_smbus_read_byte_data(dsi85_i2c_client, 0x00);
	printk(KERN_INFO "DSI85-I2C: reading 0x00 returned - 0x%x\n", ret);
	dev_dbg(&dssdev->dev, "dsi85_probe\n");
	return 0;
err_pixel_vc:
	omap_dsi_release_vc(dssdev, dsi85_d->pixel_channel);
err_debugfs:
	mutex_destroy(&dsi85_d->lock);
	gpio_free(dsi85_d->pdata->reset_gpio);
err:
	kfree(dsi85_d);
	return ret;
}

static void dsi85_remove(struct omap_dss_device* dssdev)
{
	struct dsi85_data* dsi85_d = dev_get_drvdata(&dssdev->dev);
	debugfs_remove_recursive(dsi85_d->debug_dir);
	mutex_destroy(&dsi85_d->lock);
	gpio_free(dsi85_d->pdata->reset_gpio);
	kfree(dsi85_d);
}

#ifdef DSI85DEBUG
static void dsi85_dumpconfig(struct omap_dss_device* dssdev)
{
	struct dsi85_data* dsi85_d = dev_get_drvdata(&dssdev->dev);
	struct i2c_client* dsi85_i2c_client = dsi85_d->dsi85_i2c_client;
	printk(KERN_INFO "luis' message");
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x00, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x00));
	/*
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x01, \
	i2c_smbus_read_byte_data(dsi85_i2c_client, 0x01));

	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x02, \
	i2c_smbus_read_byte_data(dsi85_i2c_client, 0x02));

	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x03, \
	i2c_smbus_read_byte_data(dsi85_i2c_client, 0x03));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x04, \
	i2c_smbus_read_byte_data(dsi85_i2c_client, 0x04));

	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x05, \
	i2c_smbus_read_byte_data(dsi85_i2c_client, 0x05));
	*/
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x0D, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x0D));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x0A, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x0A));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x0B, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x0B));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x10, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x10));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x18, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x18));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x1A, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x1A));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x20, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x20));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x21, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x21));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x22, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x22));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x23, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x23));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x24, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x24));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x25, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x25));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x26, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x26));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x27, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x27));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x28, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x28));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x29, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x29));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x2A, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x2A));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x2B, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x2B));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x2C, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x2C));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x2D, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x2D));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x2E, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x2E));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x2F, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x2F));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x30, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x30));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x31, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x31));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x32, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x32));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x33, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x33));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x34, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x34));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x35, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x35));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x36, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x32));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x37, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x33));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x38, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x34));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x39, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x35));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x3A, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x35));
	printk(KERN_INFO "DSI85-I2C: read 0x%02x  - 0x%02x\n", 0x3B, \
	       i2c_smbus_read_byte_data(dsi85_i2c_client, 0x3B));
}

#endif





#endif

#include <linux/module.h>
#include <linux/i2c.h>

/* DSI85 registers */
#define DSI85_SOFT_RESET		    0x09
#define DSI85_CORE_PLL			    0x0A
#define DSI85_PLL_DIV			    0x0B
#define DSI85_PLL_EN			    0x0D
#define DSI85_DSI_CFG			    0x10
#define DSI85_DSI_EQ			    0x11
#define DSI85_CHA_DSI_CLK_RNG       0x12
#define DSI85_CHB_DSI_CLK_RNG       0x13
#define DSI85_LVDS_MODE			    0x18
#define DSI85_LVDS_SIGN			    0x19
#define DSI85_LVDS_TERM			    0x1A
#define DSI85_CHA_LINE_LEN_LO		0x20
#define DSI85_CHA_LINE_LEN_HI		0x21
#define DSI85_CHB_LINE_LEN_LO		0x22
#define DSI85_CHB_LINE_LEN_HI		0x23
#define DSI85_CHA_VERT_LINES_LO		0x24
#define DSI85_CHA_VERT_LINES_HI		0x25
#define DSI85_CHB_VERT_LINES_LO		0x26
#define DSI85_CHB_VERT_LINES_HI		0x27
#define DSI85_CHA_SYNC_DELAY_LO		0x28
#define DSI85_CHA_SYNC_DELAY_HI		0x29
#define DSI85_CHB_SYNC_DELAY_LO		0x2A
#define DSI85_CHB_SYNC_DELAY_HI		0x2B
#define DSI85_CHA_HSYNC_WIDTH_LO	0x2C
#define DSI85_CHA_HSYNC_WIDTH_HI	0x2D
#define DSI85_CHB_HSYNC_WIDTH_LO	0x2E
#define DSI85_CHB_HSYNC_WIDTH_HI	0x2F
#define DSI85_CHA_VSYNC_WIDTH_LO	0x30
#define DSI85_CHA_VSYNC_WIDTH_HI	0x31
#define DSI85_CHB_VSYNC_WIDTH_LO	0x32
#define DSI85_CHB_VSYNC_WIDTH_HI	0x33
#define DSI85_CHA_HORZ_BACKPORCH	0x34
#define DSI85_CHB_HORZ_BACKPORCH	0x35
#define DSI85_CHA_VERT_BACKPORCH	0x36
#define DSI85_CHB_VERT_BACKPORCH	0x37
#define DSI85_CHA_HORZ_FRONTPORCH	0x38
#define DSI85_CHB_HORZ_FRONTPORCH	0x39
#define DSI85_CHA_VERT_FRONTPORCH	0x3A
#define DSI85_CHB_VERT_FRONTPORCH	0x3B

#define LVDS_CLK_FROM_DSI_CLK  1

struct dsi85_lvds_timings
{
	u16 hfp;
	u16 hsw;
	u16 hbp;
	u16 vfp;
	u16 vsw;
	u16 vbp;
};

static struct dsi85_lvds_timings lvds_timings =
{
	.hfp = 40,
	.hsw = 128,
	.hbp = 40,
	.vfp = 1,
	.vsw = 4,
	.vbp = 9,
};

#if 0
/**
 * dsi85_config - Configure dsi85
 *
 * Initial configuration for dsi85 configuration registers
 */
static void dsi85_config(struct dsi85_data* dsi85_d)
{
	printk(KERN_INFO "Now Configuring\n");
	struct i2c_client* dsi85_i2c_client = dsi85_d->dsi85_i2c_client;
	u8 val = 0;
	/* Soft reset and disable PLL */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_SOFT_RESET, 0x01);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x00);
#if LVDS_CLK_FROM_DSI_CLK
	val = 0x1;
#endif

	/* user external clock reference with no muliplier */
	if (dssdev->panel.timings.pixel_clock <= 37500)
	{
		// Do nothing.
	}
	else if (dssdev->panel.timings.pixel_clock <= 62500)
	{
		val |= (0x01 << 1);
	}
	else if (dssdev->panel.timings.pixel_clock <= 87500)
	{
		val |= (0x02 << 1);
	}
	else if (dssdev->panel.timings.pixel_clock <= 112500)
	{
		val |= (0x03 << 1);
	}
	else if (dssdev->panel.timings.pixel_clock <= 137500)
	{
		val |= (0x04 << 1);
	}
	else
	{
		val |= (0x05 << 1);
	}

	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CORE_PLL, val);
#if LVDS_CLK_FROM_DSI_CLK
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_DIV, 0x10);  // Divide DSI_CLK by 3.
#else
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_DIV, 0x00);  // Multiply REFCLK by 1.
#endif
	/* four DSI lanes with single channel*/
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_DSI_CFG, 0x20);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_DSI_EQ, 0x00);
	/* set DSI clock range */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_DSI_CLK_RNG, (dssdev->panel.timings.pixel_clock * 3 / 5000));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_DSI_CLK_RNG, (dssdev->panel.timings.pixel_clock * 3 / 5000));
	/* set LVDS for single channel, 24 bit mode, HS/VS low, DE high */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_MODE, 0x7F);
	/* set LVDS 200 Ohm termination and max differential swing voltage */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_SIGN, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_TERM, 0x00);
	/* x resolution high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_LINE_LEN_LO, \
	                          ((dssdev->panel.timings.x_res) & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_LINE_LEN_HI, \
	                          ((dssdev->panel.timings.x_res) & 0xFF00) >> 8);
	/* x resolution high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_LINE_LEN_LO, \
	                          (dssdev->panel.timings.x_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_LINE_LEN_HI, \
	                          (dssdev->panel.timings.x_res & 0xFF00) >> 8);
	/* y resolution high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_LINES_LO, \
	                          (dssdev->panel.timings.y_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_LINES_HI, \
	                          (dssdev->panel.timings.y_res & 0xFF00) >> 8);
	/* y resolution high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_LINES_LO, \
	                          (dssdev->panel.timings.y_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_LINES_HI, \
	                          (dssdev->panel.timings.y_res & 0xFF00) >> 8);
	/* SYNC delay high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, \
	                          DSI85_CHA_SYNC_DELAY_LO, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, \
	                          DSI85_CHA_SYNC_DELAY_HI, 0x02);
	/* SYNC delay high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, \
	                          DSI85_CHB_SYNC_DELAY_LO, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, \
	                          DSI85_CHB_SYNC_DELAY_HI, 0x02);
	/* HSYNC width high/low for channel A */
	/* i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_LO, \
	      (dssdev->panel.timings.hsw & 0x00FF));
	   i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_HI, \
	      (dssdev->panel.timings.hsw & 0xFF00)>>8); */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_LO, \
	                          (lvds_timings.hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_HI, \
	                          (lvds_timings.hsw & 0xFF00) >> 8);
	/* HSYNC width high/low for channel B */
	/* i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_LO, \
	      (dssdev->panel.timings.hsw & 0x00FF));
	   i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_HI, \
	      (dssdev->panel.timings.hsw & 0xFF00)>>8); */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_LO, \
	                          (lvds_timings.hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_HI, \
	                          (lvds_timings.hsw & 0xFF00) >> 8);
	/* VSYNC width high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VSYNC_WIDTH_LO, \
	                          (lvds_timings.vsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VSYNC_WIDTH_HI, \
	                          (lvds_timings.vsw & 0xFF00) >> 8);
	/* VSYNC width high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VSYNC_WIDTH_LO, \
	                          (lvds_timings.vsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VSYNC_WIDTH_HI, \
	                          (lvds_timings.vsw & 0xFF00) >> 8);
	/* Horizontal BackPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_BACKPORCH, \
	                          (lvds_timings.hbp & 0x00FF));
	/* Horizontal BackPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_BACKPORCH, \
	                          (lvds_timings.hbp & 0x00FF));
	/* Vertical BackPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_BACKPORCH, \
	                          (lvds_timings.vbp & 0x00FF));
	/* Vertical BackPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_BACKPORCH, \
	                          (lvds_timings.vbp & 0x00FF));
	/* Horizontal FrontPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_FRONTPORCH, \
	                          (lvds_timings.hfp & 0x00FF));
	/* Horizontal FrontPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_FRONTPORCH, \
	                          (lvds_timings.hfp & 0x00FF));
	/* Vertical FrontPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_FRONTPORCH, \
	                          (lvds_timings.vbp & 0x00FF));
	/* Vertical FrontPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_FRONTPORCH, \
	                          (lvds_timings.vbp & 0x00FF));
	/* Soft reset and enable PLL */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_SOFT_RESET, 0x01);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x01);
	return;
}
#endif

static void dsi85_test(struct i2c_client* dsi85_i2c_client, u32 pixel_clock, u16 x_res, u16 y_res)
{
	u8 val = 0;
	printk(KERN_INFO "Now Configuring Test Pattern\n");
	/* Soft reset and disable PLL */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_SOFT_RESET, 0x01);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x00);
#if LVDS_CLK_FROM_DSI_CLK
	val = 0x1;
#endif

	/* user external clock reference with no muliplier */
	if (pixel_clock <= 37500)
	{
		// Do nothing.
	}
	else if (pixel_clock <= 62500)
	{
		val |= (0x01 << 1);
	}
	else if (pixel_clock <= 87500)
	{
		val |= (0x02 << 1);
	}
	else if (pixel_clock <= 112500)
	{
		val |= (0x03 << 1);
	}
	else if (pixel_clock <= 137500)
	{
		val |= (0x04 << 1);
	}
	else
	{
		val |= (0x05 << 1);
	}

	//LADLDL
	printk(KERN_INFO "Pixel CLK value was %d\n", val);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CORE_PLL, val);
#if LVDS_CLK_FROM_DSI_CLK
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_DIV, 0x10);  // Divide DSI_CLK by 3.
#else
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_DIV, 0x00);  // Multiply REFCLK by 1.
#endif
	//LADLDL PLL enable after address A and B configured
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x01);
	/* four DSI lanes with single channel*/
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_DSI_CFG, 0x20);
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_DSI_EQ, 0x00);
	/* set DSI clock range */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_DSI_CLK_RNG, (pixel_clock * 3 / 5000));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_DSI_CLK_RNG, (pixel_clock * 3 / 5000));
	/* set LVDS for single channel, 24 bit mode, HS/VS low, DE high */
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_MODE, 0x7F);
	/*LADLD set LVDS for single channel, 24 bit mode, HS/VS low, DE high */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_MODE, 0x60);
	/* set LVDS 200 Ohm termination and max differential swing voltage */
	//LADLDL
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_SIGN, 0x00);
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_LVDS_TERM, 0x00);
	/* x resolution high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_LINE_LEN_LO, \
	                          ((x_res) & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_LINE_LEN_HI, \
	                          ((x_res) & 0xFF00) >> 8);
	/* x resolution high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_LINE_LEN_LO, \
	                          (x_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_LINE_LEN_HI, \
	                          (x_res & 0xFF00) >> 8);
	/* y resolution high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_LINES_LO, \
	                          (y_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_LINES_HI, \
	                          (y_res & 0xFF00) >> 8);
	/* y resolution high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_LINES_LO, \
	                          (y_res & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_LINES_HI, \
	                          (y_res & 0xFF00) >> 8);
	/* SYNC delay high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, \
	                          DSI85_CHA_SYNC_DELAY_LO, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, \
	                          DSI85_CHA_SYNC_DELAY_HI, 0x02);
	/* SYNC delay high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, \
	                          DSI85_CHB_SYNC_DELAY_LO, 0x00);
	i2c_smbus_write_byte_data(dsi85_i2c_client, \
	                          DSI85_CHB_SYNC_DELAY_HI, 0x02);
	/* HSYNC width high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_LO, \
	                          (lvds_timings.hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HSYNC_WIDTH_HI, \
	                          (lvds_timings.hsw & 0xFF00) >> 8);
	/* HSYNC width high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_LO, \
	                          (lvds_timings.hsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HSYNC_WIDTH_HI, \
	                          (lvds_timings.hsw & 0xFF00) >> 8);
	/* VSYNC width high/low for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VSYNC_WIDTH_LO, \
	                          (lvds_timings.vsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VSYNC_WIDTH_HI, \
	                          (lvds_timings.vsw & 0xFF00) >> 8);
	/* VSYNC width high/low for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VSYNC_WIDTH_LO, \
	                          (lvds_timings.vsw & 0x00FF));
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VSYNC_WIDTH_HI, \
	                          (lvds_timings.vsw & 0xFF00) >> 8);
	/* Horizontal BackPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_BACKPORCH, \
	                          (lvds_timings.hbp & 0x00FF));
	/* Horizontal BackPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_BACKPORCH, \
	                          (lvds_timings.hbp & 0x00FF));
	/* Vertical BackPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_BACKPORCH, \
	                          (lvds_timings.vbp & 0x00FF));
	/* Vertical BackPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_BACKPORCH, \
	                          (lvds_timings.vbp & 0x00FF));
	/* Horizontal FrontPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_HORZ_FRONTPORCH, \
	                          (lvds_timings.hfp & 0x00FF));
	/* Horizontal FrontPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_HORZ_FRONTPORCH, \
	                          (lvds_timings.hfp & 0x00FF));
	/* Vertical FrontPorch for channel A */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHA_VERT_FRONTPORCH, \
	                          (lvds_timings.vbp & 0x00FF));
	/* Vertical FrontPorch for channel B */
	i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_CHB_VERT_FRONTPORCH, \
	                          (lvds_timings.vbp & 0x00FF));
	//Test Pattern
	i2c_smbus_write_byte_data(dsi85_i2c_client, 0x3C, 0x11);
	//LADLDL
	/* Soft reset and enable PLL */
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_SOFT_RESET, 0x01);
	//i2c_smbus_write_byte_data(dsi85_i2c_client, DSI85_PLL_EN, 0x01);
	return;
}

static int dsi85_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	dsi85_test(client, 137500, 1920, 1080);
	return 0;
}

static int dsi85_remove(struct i2c_client* client)
{
	return 0;
}

static const struct i2c_device_id dsi85_id[] =
{
	{ "dsi85", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, dsi85_id);

#ifdef CONFIG_OF
static const struct of_device_id of_dsi85_match[] =
{
	{ .compatible = "ti,dsi85", },
	{},
};

MODULE_DEVICE_TABLE(of, of_dsi85_match);
#endif

static struct i2c_driver dsi85_driver =
{
	.driver = {
		.name = "dsi85_i2c_driver",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(of_dsi85_match),
#endif
	},
	.probe      = dsi85_probe,
	.remove     = dsi85_remove,
	.id_table   = dsi85_id,
};

module_i2c_driver(dsi85_driver);

MODULE_DESCRIPTION("Texas DSI85 dsi to lvds driver");
MODULE_AUTHOR("Moritz Bitsch");
MODULE_LICENSE("GPL");

