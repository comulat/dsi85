/*
 * Copyright 2017 Hella Gutmann Solutions GmbH
 * Author: Moritz Bitsch <moritz.bitsch@hella-gutmann.com>
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

#include <linux/module.h>
#include <linux/i2c.h>

const struct _cfg_reg
{
	uint8_t reg;
	uint8_t val;
} cfg_reg[] =
{
	{0x09, 0x01}, // soft reset
	{0x0D, 0x00}, // pll disable

	{0x09, 0x00},
	{0x0A, 0x05},
	{0x0B, 0x28},
	{0x0D, 0x00},
	{0x10, 0x26},
	{0x11, 0x00},
	{0x12, 0x5d},
	{0x13, 0x00},
	{0x18, 0x6c},
	{0x19, 0x00},
	{0x1A, 0x03},
	{0x1B, 0x00},
	{0x20, 0x80},
	{0x21, 0x07},
	{0x22, 0x00},
	{0x23, 0x00},
	{0x24, 0x00},
	{0x25, 0x00},
	{0x26, 0x00},
	{0x27, 0x00},
	{0x28, 0x21},
	{0x29, 0x00},
	{0x2A, 0x00},
	{0x2B, 0x00},
	{0x2C, 0x2c},
	{0x2D, 0x00},
	{0x2E, 0x00},
	{0x2F, 0x00},
	{0x30, 0x0f},
	{0x31, 0x00},
	{0x32, 0x00},
	{0x33, 0x00},
	{0x34, 0x30},
	{0x35, 0x00},
	{0x36, 0x00},
	{0x37, 0x00},
	{0x38, 0x00},
	{0x39, 0x00},
	{0x3A, 0x00},
	{0x3B, 0x00},
	{0x3C, 0x00},
	{0x3D, 0x00},
	{0x3E, 0x00},

	{0x0D, 0x01}, // pll enable
	{0x09, 0x01}, // soft reset
	{}
};

static int dsi85_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	int i;

	for (i = 0; cfg_reg[i].reg != 0x00; i++)
	{
		if (i2c_smbus_write_byte_data(client, cfg_reg[i].reg, cfg_reg[i].val) < 0)
		{
			return -1;
		}
	}

	return 0;
}

static int dsi85_remove(struct i2c_client* client)
{
	/* disable pll */
	i2c_smbus_write_byte_data(client, 0x0D, 0x00);
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

MODULE_DESCRIPTION("Ti DSI85 dsi to lvds converter driver");
MODULE_AUTHOR("Moritz Bitsch");
MODULE_LICENSE("GPL");
