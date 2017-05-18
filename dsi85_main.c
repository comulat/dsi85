/*
 * Copyright (C) 2017, Hella-Gutmann Solutions GmbH
 *
 * Partly based on panel-dsi85.c from:
 *
 * Copyright 2011 Texas Instruments, Inc.
 * Author: Archit Taneja <archit@ti.com>
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
 * 
 * Also partly based on the ptn3460 and adv7533 kernel drivers.
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
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_edid.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>

#include <video/mipi_display.h>

#include "dsi85_registers.h"

#define SN65DSI85_MODULE_NAME "bridge-sn65dsi85"

/* Config struct
 * (to be filled from DT. See dsi85.txt.)
 */
struct sn65dsi85_config {
	u32 lanes; 
	u32 lvds_channels;
	u32 sync_delay;
		
	u32 de_neg_polarity;
	u32 force_24bpp_mode;
	u32 force_24bpp_format1;
	u32 lvds_vocm;
	u32 lvds_vod_swing_cha;
	u32 lvds_vod_swing_chb;
	u32 lvds_even_odd_swap;
	u32 lvds_reverse;
	u32 lvds_term;
	u32 lvds_cm_adjust_cha;
	u32 lvds_cm_adjust_chb;
};

/*
 * Device state
 */
struct sn65dsi85_device {
	struct sn65dsi85_config *config;
	struct i2c_client *i2c;
	const struct i2c_device_id *i2c_id;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_panel *panel;
	struct device_node *dsi_host_node;
	struct mipi_dsi_device *dsi;
};

/* Registers of the DSI85, named like in the data sheet.
 * except for _LO/_HI pairs, which appear as one u16,
 * and TPG-only registers, which are omitted. */
struct sn65dsi85_regs {
	u8 pll_en_stat;
	u8 lvds_clk_range;
	u8 hs_clk_src;
	u8 dsi_clk_divider;
	u8 refclk_multiplier;
	
	u8 left_right_pixels;
	u8 dsi_channel_mode;
	u8 cha_dsi_lanes;
	u8 chb_dsi_lanes;
	u8 sot_err_tol_dis;
	u8 cha_dsi_data_eq;
	u8 chb_dsi_data_eq;
	u8 cha_dsi_clk_eq;
	u8 chb_dsi_clk_eq;
	
	u8 cha_dsi_clk_rng;
	u8 chb_dsi_clk_rng;
	
	u8 de_neg_polarity;
	u8 hs_neg_polarity;
	u8 vs_neg_polarity;
	u8 lvds_link_cfg;
	u8 cha_24bpp_mode;
	u8 chb_24bpp_mode;
	u8 cha_24bpp_format1;
	u8 chb_24bpp_format1;

	u8 cha_lvds_vocm;
	u8 chb_lvds_vocm;
	u8 cha_lvds_vod_swing;
	u8 chb_lvds_vod_swing;
	
	u8 even_odd_swap;
	u8 cha_reverse_lvds;
	u8 chb_reverse_lvds;
	u8 cha_lvds_term;
	u8 chb_lvds_term;

	u8 cha_lvds_cm_adjust;
	u8 chb_lvds_cm_adjust;
	
	u16 cha_active_line_length;
	u16 chb_active_line_length;
	
	u16 cha_sync_delay;
	u16 chb_sync_delay;
	
	u16 cha_hsync_pulse_width;
	u16 chb_hsync_pulse_width;
	
	u16 cha_vsync_pulse_width;
	u16 chb_vsync_pulse_width;
	
	u8 cha_horizontal_back_porch;
	u8 chb_horizontal_back_porch;
	
	/* omitted: TPG-related */
};

static inline struct sn65dsi85_device *bridge_to_sn65dsi85(struct drm_bridge *bridge)
{
	return container_of(bridge, struct sn65dsi85_device, bridge);
}
static inline struct sn65dsi85_device *connector_to_sn65dsi85(struct drm_connector *connector)
{
	return container_of(connector, struct sn65dsi85_device, connector);
}

static int dsi85_hardcoded_deinit(struct i2c_client* client)
{
	/* disable pll */
	i2c_smbus_write_byte_data(client, 0x0D, 0x00);
	return 0;
}

/*
 * Read configuration from metadata
 */
static struct sn65dsi85_config*
sn65dsi85_get_pdata(struct i2c_client *client)
{
	struct device_node *dsi85_node = client->dev.of_node;
	struct device_node *endpoint = NULL;
	struct sn65dsi85_config *pdata = NULL;
	
	DRM_DEV_DEBUG(&client->dev, "Consulting DT node %s", dsi85_node->full_name);

	endpoint = of_graph_get_next_endpoint(dsi85_node, NULL);
	if(!endpoint) {
		DRM_DEV_ERROR(&client->dev, "Cannot find OF endpoint\n");
		return NULL;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if(!pdata) {
		DRM_DEV_ERROR(&client->dev, "Failed to kzalloc the config object");
		goto out_put_endpoint;
	}

	/* read DT attributes into Config */
	if(of_property_read_u32(dsi85_node, "dsi-lanes", &pdata->lanes) < 0) {
		DRM_DEV_INFO(&client->dev, "DT: dsi-lanes property not found, using default\n");
		pdata->lanes = 4;
	}
	if(of_property_read_u32(dsi85_node, "lvds-channels", &pdata->lvds_channels) < 0) {
		DRM_DEV_INFO(&client->dev, "DT: lvds-channels property not found, using default\n");
		pdata->lvds_channels = 1;
	} else {
		if( pdata->lvds_channels < 1 || pdata->lvds_channels > 2 ) {
			DRM_DEV_ERROR(&client->dev, "DT: lvds-channels must be 1 or 2, not %u", pdata->lvds_channels);
			goto out_free;
		}
	}
	of_property_read_u32(dsi85_node, "sync-delay", &pdata->sync_delay);

	of_property_read_u32(dsi85_node, "de-neg-polarity", &pdata->de_neg_polarity);
	of_property_read_u32(dsi85_node, "force-24bpp-mode", &pdata->force_24bpp_mode);
	of_property_read_u32(dsi85_node, "force-24bpp-format1", &pdata->force_24bpp_format1);
	of_property_read_u32(dsi85_node, "lvds-vocm", &pdata->lvds_vocm);
	of_property_read_u32(dsi85_node, "lvds-vod-swing-cha", &pdata->lvds_vod_swing_cha);
	of_property_read_u32(dsi85_node, "lvds-vod-swing-chb", &pdata->lvds_vod_swing_chb);
	of_property_read_u32(dsi85_node, "lvds-even-odd-swap", &pdata->lvds_even_odd_swap);
	of_property_read_u32(dsi85_node, "lvds-reverse", &pdata->lvds_reverse);
	of_property_read_u32(dsi85_node, "lvds-term", &pdata->lvds_term);
	of_property_read_u32(dsi85_node, "lvds-cm-adjust-cha", &pdata->lvds_cm_adjust_cha);
	of_property_read_u32(dsi85_node, "lvds-cm-adjust-chb", &pdata->lvds_cm_adjust_chb);

	DRM_DEV_INFO(&client->dev, "DT attribute result: dsi-lanes=%d lvds-channels=%d sync-delay=%u",
		 pdata->lanes, pdata->lvds_channels, pdata->sync_delay);
	of_node_put(endpoint);
	return pdata;

out_free:			/* Sorry, DT node is invalid, bailing out. */
	devm_kfree(&client->dev, pdata);
	pdata = NULL;

out_put_endpoint:
	of_node_put(endpoint);
	return pdata;	

}

static enum drm_connector_status sn65dsi85_connector_detect(struct drm_connector *connector,
		bool force)
{
	DRM_DEV_DEBUG(connector->dev->dev, "%s entry", __func__);
	return connector_status_connected;
}

static void sn65dsi85_connector_destroy(struct drm_connector* connector)
{
	DRM_DEV_DEBUG(connector->dev->dev, "%s entry", __func__);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs sn65dsi85_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = sn65dsi85_connector_detect,
	.destroy = sn65dsi85_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_encoder *sn65dsi85_best_encoder(struct drm_connector* connector)
{
	struct sn65dsi85_device *self = connector_to_sn65dsi85(connector);
	/* The bridge's encoder is the best encoder */
	return self->bridge.encoder;
}

/* Get modes (connector_helper callback), delegated to Panel */
static int sn65dsi85_get_modes(struct drm_connector* connector)
{
	struct sn65dsi85_device *self;
	int num_modes = 0;
	DRM_DEV_DEBUG(connector->dev->dev, "%s entry connector=%p", __func__, connector);

	if(!connector) {
		pr_err("Null connector in sn65dsi85_get_modes");
		return num_modes;
	}
	self = connector_to_sn65dsi85(connector);

	if (self->panel && self->panel->funcs && self->panel->funcs->get_modes) {
		num_modes = self->panel->funcs->get_modes(self->panel);
		if (num_modes > 0) {
			DRM_DEV_DEBUG(connector->dev->dev, "%s got %d modes from panel, good enough", __func__, num_modes);
		}
		else {
			DRM_WARN("got 0 modes from panel");
		}
	}
	else {
		DRM_WARN("could not interrogate drm_panel for modes");
	}

	return num_modes;
}

static struct drm_connector_helper_funcs sn65dsi85_connector_helper_funcs = {
	.get_modes = sn65dsi85_get_modes,
	.best_encoder = sn65dsi85_best_encoder,
};

static void sn65dsi85_bridge_pre_enable(struct drm_bridge* bridge)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);
	int res;
	DRM_DEV_DEBUG(bridge->dev->dev, "%s entry", __func__);

	res = drm_panel_prepare(self->panel);
	if (res) {
		DRM_ERROR("failed to prepare my panel: %d\n", res);
		return;
	}

}
static void sn65dsi85_bridge_enable(struct drm_bridge* bridge)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);
	int res;
	DRM_DEV_DEBUG(bridge->dev->dev, "%s entry", __func__);

	DRM_DEV_INFO(bridge->dev->dev, "%s: time for PLL-Enable", __func__);
	i2c_smbus_write_byte_data(self->i2c, 0x0D, 0x01);
	i2c_smbus_write_byte_data(self->i2c, 0x09, 0x01);			
	DRM_DEV_INFO(bridge->dev->dev, "%s: PLL-Enable done", __func__);

	res = drm_panel_enable(self->panel);
	if (res) {
		DRM_ERROR("failed to enable my panel: %d\n", res);
		return;
	}
}
static void sn65dsi85_bridge_disable(struct drm_bridge* bridge)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);
	int res;
	DRM_DEV_DEBUG(bridge->dev->dev, "%s entry", __func__);

	DRM_DEV_INFO(bridge->dev->dev, "Time for hardcoded deinit");
	dsi85_hardcoded_deinit(self->i2c);
	DRM_DEV_INFO(bridge->dev->dev, "Done hardcoded deinit");

	res = drm_panel_disable(self->panel);
	if (res) {
		DRM_ERROR("failed to disable my panel: %d\n", res);
		return;
	}
}
static void sn65dsi85_bridge_post_disable(struct drm_bridge* bridge)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);
	int res;
	DRM_DEV_DEBUG(bridge->dev->dev, "%s entry", __func__);

	res = drm_panel_unprepare(self->panel);
	if (res) {
		DRM_ERROR("failed to unprepare my panel: %d\n", res);
		return;
	}
}

/*
 * Configure DSI link properties by creating a mipi_dsi_device
 */
int sn65dsi85_attach_dsi(struct sn65dsi85_device *self)
{
	struct device *dev = &self->i2c->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;
	const struct mipi_dsi_device_info info = { .type = "sn65dsi85_dsi_client",
						   .channel = 0,
						   .node = NULL,
						 };

	host = of_find_mipi_dsi_host_by_node(self->dsi_host_node);
	if (!host) {
		DRM_DEV_ERROR(dev, "%s failed to find dsi host by OF node %s\n", __func__,
			self->dsi_host_node? self->dsi_host_node->full_name : "null-OF-node");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		ret = PTR_ERR(dsi);
		DRM_DEV_ERROR(dev, "%s failed to create dsi device: %d\n", __func__, ret);
		goto err_dsi_device;
	}

	self->dsi = dsi;

	/* In the simple driver, this was part of the panel_desc_dsi instance: */
	dsi->lanes = self->config->lanes;
	dsi->format = MIPI_DSI_FMT_RGB888; /* maybe from DT later */
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO; /* maybe from DT later */

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "%s failed to attach dsi to host: %d\n", __func__, ret);
		goto err_dsi_attach;
	}

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}

void sn65dsi85_detach_dsi(struct sn65dsi85_device *self)
{
	mipi_dsi_detach(self->dsi);
	mipi_dsi_device_unregister(self->dsi);
}

/*
 * Main Bridge config callback.
 *
 * Create a DRM Connector which answers queries for supported modes
 * (by delegating to the Panel, in our case).
 * Create a DSI Device which holds the DSI protocol details.
 */
static int sn65dsi85_bridge_attach(struct drm_bridge* bridge)
{
	struct sn65dsi85_device* self = bridge_to_sn65dsi85(bridge);
	int ret = 0;

	DRM_DEV_DEBUG(bridge->dev->dev, "%s entry", __func__);

	if(!bridge->encoder) {
		DRM_ERROR("Bridge has no Encoder");
		return -ENODEV;
	}

	self->connector.polled = 0; // no hotplugging here
	ret = drm_connector_init(bridge->dev, &self->connector,
				 &sn65dsi85_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if(ret) {
		DRM_ERROR("Could not drm_connector_init: %d", ret);
		return ret;
	}
	drm_connector_helper_add(&self->connector, &sn65dsi85_connector_helper_funcs);

	/* Tempting to do right now, but not a good idea:
	 *   drm_connector_register(&self->connector);
	 * Wants to create kobject nodes, but the connector has no kobject parent as of yet. 
	 * The kobject parent will be assigned later, 
	 * but drm_connector_init has already added our connector
	 * to its internal list, and there will be a call to drm_connector_register_all later.
	 */ 

	ret = drm_mode_connector_attach_encoder(&self->connector, self->bridge.encoder);
	if(ret) {
		DRM_ERROR("Could not drm_mode_connector_attach_encoder: %d", ret);
		return ret;
	}

	/* In order to configure the DSI parameters, create a mipi_dsi_device
	 */
	ret = sn65dsi85_attach_dsi(self);
	if(ret) {
		DRM_ERROR("Could not sn65dsi85_attach_dsi: %d", ret);
		return ret;
	}
	

	if (self->panel)
		drm_panel_attach(self->panel, &self->connector);

	/* but no hotplug IRQ. Other drivers do that here. */


	return ret;
}

static inline void write_dsi85(struct i2c_client *i2c, u8 reg, u8 value)
{
	int res;
	res = i2c_smbus_write_byte_data(i2c, reg, value);
	DRM_DEV_DEBUG(&i2c->dev, "write reg 0x%X <- 0x%X (res=%d)", reg, value, res);
	if(res < 0) {
		DRM_DEV_ERROR(&i2c->dev, "could not write DSI85 register 0x%x, i2c error %d", reg, res);
	}
}

static inline u8 compute_dsi_clock_range_code_from_mode(struct drm_display_mode *mode, 
							int bytes_per_pixel)
{
	/*
	 * DSI85 wants to know the approximate DSI clock in 5MHz increments.
	 */

	int total_pixels_per_sec = mode->htotal * mode->vtotal * mode->vrefresh;
	int dsi_clk = total_pixels_per_sec * bytes_per_pixel;
	int dsi_clk_div5MHz = dsi_clk / 5000000; 
	
	if(8 <= dsi_clk_div5MHz && dsi_clk_div5MHz <= 0x64) {
		return (u8) dsi_clk_div5MHz;
	}
	else {
		DRM_ERROR("DSI clock out of range: %d pixels/s give clock %d, range %d out of range\n",
			  total_pixels_per_sec, dsi_clk, dsi_clk_div5MHz);
		return 0;
	}
}

/*
 * Translate config and mode attributes to a complete DSI85 configuration,
 * write them to registers.
 */
static void sn65dsi85_apply_mode(struct sn65dsi85_device *self,
				struct drm_display_mode *mode)
{
	struct i2c_client *i2c = self->i2c;
	int lvds_clock;
	bool lvds_clock_is_half_dsi_clock = (self->config->lvds_channels == 2);
	struct sn65dsi85_config* dt = self->config;

	struct sn65dsi85_regs regs = {
		.pll_en_stat = 0,
		.lvds_clk_range = 0,
		.hs_clk_src = 1, /* LVDS pixel clock derived from MIPI D-PHY channel A HS continuous clock */
		.dsi_clk_divider = lvds_clock_is_half_dsi_clock? 5: 2, /* divide by 6 or 3 for 24bpp */

		.refclk_multiplier = 0,

		.left_right_pixels = 0,
		.dsi_channel_mode = 1, /* single channel DSI receiver */
		.cha_dsi_lanes = 0,    /* four lanes A */
		.chb_dsi_lanes = 0x3,  /* one lane B (default) */
		.sot_err_tol_dis = 0,  /* tolerate single bit errors for SoT leader sequence */

		.cha_dsi_data_eq = 0,
		.chb_dsi_data_eq = 0,

		.cha_dsi_clk_eq = 0,
		.chb_dsi_clk_eq = 0,		
		.cha_dsi_clk_rng = compute_dsi_clock_range_code_from_mode(mode, 3),
		.chb_dsi_clk_rng = 0,
		
		.de_neg_polarity = !!dt->de_neg_polarity,
		.hs_neg_polarity = 0, /* from Panel, see below */
		.vs_neg_polarity = 0,
		.lvds_link_cfg = (dt->lvds_channels==2)?0:1,
		.cha_24bpp_mode = !!dt->force_24bpp_mode,
		.chb_24bpp_mode = !!dt->force_24bpp_mode,
		.cha_24bpp_format1 = !!dt->force_24bpp_format1,
		.chb_24bpp_format1 =  !!dt->force_24bpp_format1,

		.cha_lvds_vocm = !!(dt->lvds_vocm & 0x02),
		.chb_lvds_vocm = !!(dt->lvds_vocm & 0x01),
		.cha_lvds_vod_swing = dt->lvds_vod_swing_cha,
		.chb_lvds_vod_swing = dt->lvds_vod_swing_chb,

		.even_odd_swap = !!(dt->lvds_even_odd_swap),
		.cha_reverse_lvds = !!(dt->lvds_reverse & 0x02),
		.chb_reverse_lvds = !!(dt->lvds_reverse & 0x01),
		.cha_lvds_term = !!(dt->lvds_term & 0x02),
		.chb_lvds_term = !!(dt->lvds_term & 0x01),

		.cha_lvds_cm_adjust = dt->lvds_cm_adjust_cha,
		.chb_lvds_cm_adjust = dt->lvds_cm_adjust_chb,	

		.cha_active_line_length = mode->hdisplay,
		.chb_active_line_length = 0,
		
		.cha_sync_delay = (u16)dt->sync_delay,
		.chb_sync_delay = 0,
		
		.cha_hsync_pulse_width = mode->hsync_end - mode->hsync_start,
		.chb_hsync_pulse_width = 0,
		
		.cha_vsync_pulse_width = mode->vsync_end - mode->vsync_start,
		.chb_vsync_pulse_width = 0,

		.cha_horizontal_back_porch = mode->hsync_start - mode->hdisplay,
		.chb_horizontal_back_porch = 0,
	};

	/* ------- Decide about complicated register values... ------- */	

	/* LVDS panel may need half of everything horizontal if using two LVDS channels */
	if (lvds_clock_is_half_dsi_clock) {
		lvds_clock = mode->clock / 2;
		regs.cha_hsync_pulse_width /= 2;
		regs.cha_horizontal_back_porch /= 2;
	} else {
		lvds_clock = mode->clock;
	}

	/* Compute LVDS clock-range register; ranges from datasheet */
	if (lvds_clock <= 37500) {
		/* use 0 */
	}
	else if (lvds_clock <= 62500) {
		regs.lvds_clk_range = 0x01;
	}
	else if (lvds_clock <= 87500){
		regs.lvds_clk_range = 0x02;
	}
	else if (lvds_clock <= 112500)	{
		regs.lvds_clk_range = 0x03;
	}
	else if (lvds_clock <= 137500) {
		regs.lvds_clk_range = 0x04;
	}
	else {
		regs.lvds_clk_range = 0x05;
	}

	/* Sync polarities */
	if( (mode->flags & DRM_MODE_FLAG_NHSYNC) != 0 ) {
		regs.hs_neg_polarity = 1;
	}
	if( (mode->flags & DRM_MODE_FLAG_NVSYNC) != 0 ) {
		regs.vs_neg_polarity = 1;
	}

	/* Soft reset and disable PLL */
	write_dsi85(i2c,  DSI85_SOFT_RESET, 1 );
	write_dsi85(i2c,  DSI85_PLL_EN, 0 );

	/* ------- CORE_PLL register ------- */
	write_dsi85(i2c,  DSI85_CORE_PLL, 
			     ((regs.pll_en_stat & 0x01) << 7)
			     | ((regs.lvds_clk_range & 0x07) << 1)
			     | ((regs.hs_clk_src & 0x01) << 0 ));

	/* ------- PLL_DIV register ------- */
	write_dsi85(i2c,  DSI85_PLL_DIV, 
			     (regs.dsi_clk_divider << 3) 
			     | (regs.refclk_multiplier & 0x3) );

	/* ------- DSI ------- */
	write_dsi85(i2c,  DSI85_DSI_CFG, 
			     ((regs.left_right_pixels & 0x01) << 7)
			     | ((regs.dsi_channel_mode & 0x03) << 5)
			     | ((regs.cha_dsi_lanes & 0x03) << 3)
			     | ((regs.chb_dsi_lanes & 0x03) << 1)
			     | ((regs.sot_err_tol_dis) << 0));
	write_dsi85(i2c,  DSI85_DSI_EQ,
			     ((regs.cha_dsi_data_eq & 0x03) << 6)
			     | ((regs.chb_dsi_data_eq & 0x03) << 4)
			     | ((regs.cha_dsi_clk_eq & 0x03) << 2)
			     | ((regs.chb_dsi_clk_eq & 0x03) << 0));
	write_dsi85(i2c,  DSI85_CHA_DSI_CLK_RNG,
			     regs.cha_dsi_clk_rng);
	write_dsi85(i2c,  DSI85_CHB_DSI_CLK_RNG,
			     regs.chb_dsi_clk_rng);
	
	/* ------- LVDS ------- */

	write_dsi85(i2c,  DSI85_LVDS_MODE,
			     ((regs.de_neg_polarity & 0x01) << 7)
			     | ((regs.hs_neg_polarity & 0x01) << 6)
			     | ((regs.vs_neg_polarity & 0x01) << 5)
			     | ((regs.lvds_link_cfg   & 0x01) << 4)
			     | ((regs.cha_24bpp_mode  & 0x01) << 3)
			     | ((regs.chb_24bpp_mode  & 0x01) << 2)
			     | ((regs.cha_24bpp_format1  & 0x01) << 1)
			     | ((regs.chb_24bpp_format1  & 0x01) << 0));
	write_dsi85(i2c,  DSI85_LVDS_SIGN,
			     ((regs.cha_lvds_vocm & 0x01) << 6)
			     | ((regs.chb_lvds_vocm & 0x01) << 4)
			     | ((regs.cha_lvds_vod_swing & 0x03) << 2)
			     | ((regs.chb_lvds_vod_swing & 0x03) << 0));
	write_dsi85(i2c,  DSI85_LVDS_TERM,
			     ((regs.even_odd_swap & 0x01) << 6)
			     | ((regs.cha_reverse_lvds & 0x01) << 5)
			     | ((regs.chb_reverse_lvds & 0x01) << 4)
			     | ((regs.cha_lvds_term & 0x01) << 1)
			     | ((regs.chb_lvds_term & 0x01) << 0));
	write_dsi85(i2c,  DSI85_LVDS_ADJUST, 
		    	     ((regs.cha_lvds_cm_adjust & 0x03) << 4)
	 		    | ((regs.chb_lvds_cm_adjust & 0x03) << 0));
	
	/* ------- Dimensions ------- */
	write_dsi85(i2c,  DSI85_CHA_LINE_LEN_LO, 
			     (u8)(regs.cha_active_line_length & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHA_LINE_LEN_HI, 
			     (u8)((regs.cha_active_line_length & 0x0f00u) >> 8));
	write_dsi85(i2c,  DSI85_CHB_LINE_LEN_LO, 
			     (u8)(regs.chb_active_line_length & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHB_LINE_LEN_HI, 
			     (u8)((regs.chb_active_line_length & 0x0f00u) >> 8));

	write_dsi85(i2c,  DSI85_CHA_VERT_LINES_LO, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHA_VERT_LINES_HI, 0 );
	write_dsi85(i2c,  DSI85_CHB_VERT_LINES_LO, 0 );
	write_dsi85(i2c,  DSI85_CHB_VERT_LINES_HI, 0 );

	write_dsi85(i2c,  DSI85_CHA_SYNC_DELAY_LO, 
			     (u8)(regs.cha_sync_delay & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHA_SYNC_DELAY_HI, 
			     (u8)((regs.cha_sync_delay & 0x0f00u) >> 8));
	write_dsi85(i2c,  DSI85_CHB_SYNC_DELAY_LO, 
			     (u8)(regs.chb_sync_delay & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHB_SYNC_DELAY_HI, 
			     (u8)((regs.chb_sync_delay & 0x0f00u) >> 8));

	write_dsi85(i2c,  DSI85_CHA_HSYNC_WIDTH_LO, 
			     (u8)(regs.cha_hsync_pulse_width & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHA_HSYNC_WIDTH_HI, 
			     (u8)((regs.cha_hsync_pulse_width & 0x0f00u) >> 8));
	write_dsi85(i2c,  DSI85_CHB_HSYNC_WIDTH_LO, 
			     (u8)(regs.chb_hsync_pulse_width & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHB_HSYNC_WIDTH_HI, 
			     (u8)((regs.chb_hsync_pulse_width & 0x0f00u) >> 8));

	write_dsi85(i2c,  DSI85_CHA_VSYNC_WIDTH_LO, 
			     (u8)(regs.cha_vsync_pulse_width & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHA_VSYNC_WIDTH_HI, 
			     (u8)((regs.cha_vsync_pulse_width & 0x0f00u) >> 8));
	write_dsi85(i2c,  DSI85_CHB_VSYNC_WIDTH_LO, 
			     (u8)(regs.chb_vsync_pulse_width & 0x00ffu));
	write_dsi85(i2c,  DSI85_CHB_VSYNC_WIDTH_HI, 
			     (u8)((regs.chb_vsync_pulse_width & 0x0f00u) >> 8));

	write_dsi85(i2c,  DSI85_CHA_HORZ_BACKPORCH, regs.cha_horizontal_back_porch );
	write_dsi85(i2c,  DSI85_CHB_HORZ_BACKPORCH, regs.chb_horizontal_back_porch );

	write_dsi85(i2c,  DSI85_CHA_VERT_BACKPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHB_VERT_BACKPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHA_HORZ_FRONTPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHB_HORZ_FRONTPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHA_HORZ_FRONTPORCH, 0 ); /* TPG only */
	write_dsi85(i2c,  DSI85_CHB_VERT_FRONTPORCH, 0 ); /* TPG only */
	
}


static void sn65dsi85_bridge_mode_set(struct drm_bridge *bridge,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct sn65dsi85_device *self = bridge_to_sn65dsi85(bridge);

	DRM_DEV_DEBUG(bridge->dev->dev, "%s entry", __func__);
	sn65dsi85_apply_mode(self, mode);
	DRM_DEV_DEBUG(bridge->dev->dev, "%s exit", __func__);
}

static bool sn65dsi85_bridge_mode_fixup(struct drm_bridge *bridge,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	/* The panel may request negative vsync and hsync, but please don't
	 * tell the DSI host; we want the DSI host to transmit uniform polarity,
	 * and sn65dsi85_apply_mode will tell the DSI85 what polarity to produce. */
	if( mode->flags & (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) ) {
		adjusted_mode->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
		DRM_DEBUG("Clearing FLAG_NHSYNC and FLAG_NVSYNC for DSI");
	}

	return 1;
}

static const struct drm_bridge_funcs sn65dsi85_bridge_funcs = {
	.pre_enable   = sn65dsi85_bridge_pre_enable,
	.enable       = sn65dsi85_bridge_enable,
	.disable      = sn65dsi85_bridge_disable,
	.post_disable = sn65dsi85_bridge_post_disable,
	.attach       = sn65dsi85_bridge_attach,
	.mode_set     = sn65dsi85_bridge_mode_set,
	.mode_fixup   = sn65dsi85_bridge_mode_fixup,
};

/*
 * Probe an SN65DSI85 on an I2C bus.
 *
 * This installs a drm_bridge into the DRM system to do the actual mode-setting work.
 *
 * @c i2c_client pointer
 * @id i2c_device_id pointer
 */
static int sn65dsi85_probe(struct i2c_client* c, const struct i2c_device_id *id)
{
	struct sn65dsi85_config *pdata = NULL;
	struct sn65dsi85_device *ctx = NULL;
	struct device_node *endpoint, *panel_node, *host_node;
	int error_value = 0;

	DRM_DEV_INFO(&c->dev, "sn65dsi85_probe: welcome!\n");

	pdata = sn65dsi85_get_pdata(c);
	if(!pdata) {
		DRM_DEV_ERROR(&c->dev, "Cannot read configuration. Probe failed.\n");
		return -EINVAL;
	}

	ctx = devm_kzalloc(&c->dev, sizeof(struct sn65dsi85_device), GFP_KERNEL);
	if(!ctx) {
		DRM_DEV_ERROR(&c->dev, "Cannot allocate device state\n");
		error_value = -ENOMEM;
		goto fail_free_pdata;
	}
	ctx->config = pdata;
	ctx->i2c = c;
	ctx->i2c_id = id;
	i2c_set_clientdata(c, ctx);

	/* Find the associated DSI host. port@0 is input */
	endpoint = of_graph_get_endpoint_by_regs(c->dev.of_node, 0, -1);
	if (endpoint) {
		DRM_DEV_DEBUG(&c->dev, "%s my DSI-side endpoint: %s", __func__, endpoint->full_name);
		host_node = of_graph_get_remote_port_parent(endpoint);
		if (host_node) {
			DRM_DEV_DEBUG(&c->dev, "%s my DSI host node: %s", __func__, host_node->full_name);
			/* Store a pointer to the DT node because the DSI host may not be up yet */
			ctx->dsi_host_node = host_node;
		}
		else {
			DRM_DEV_ERROR(&c->dev, "%s cannot find DSI host node", __func__);
		}
	}
	else {
		DRM_DEV_ERROR(&c->dev, "%s cannot find endpoint 0 (towards DSI)", __func__);
		error_value = -ENOENT;
		goto fail_free_pdata;
	}

	/* Find the associated panel. port@1 is output */
	endpoint = of_graph_get_endpoint_by_regs(c->dev.of_node, 1, -1);
	if (endpoint) {
		DRM_DEV_DEBUG(&c->dev, "%s my panel-side endpoint: %s", __func__, endpoint->full_name);
		panel_node = of_graph_get_remote_port_parent(endpoint);
		if (panel_node) {
			ctx->panel = of_drm_find_panel(panel_node);
			if (!ctx->panel) {
				DRM_WARN("Cannot find panel by panel node name=%s", 
					 panel_node->full_name);
				return -EPROBE_DEFER;
			}
			of_node_put(panel_node);
		}
	}
	else {
		DRM_DEV_ERROR(&c->dev, "%s cannot find endpoint 1 (towards panel)", __func__);
		error_value = -ENOENT;
		goto fail_free_pdata;
	}

	ctx->bridge.funcs = &sn65dsi85_bridge_funcs;
	ctx->bridge.of_node = c->dev.of_node;
	
	error_value = drm_bridge_add(&ctx->bridge);
	(void)error_value; // "Unconditionally returns zero"

        DRM_DEV_INFO(&c->dev, "sn65dsi85_probe: done!\n");

	return 0;

fail_free_pdata:
	kfree(pdata);
        DRM_DEV_INFO(&c->dev, "sn65dsi85_probe: err=%d :(\n", error_value);
	return error_value;
	
}

/*
 * Remove SN65DSI85
 * @c: pointer to the i2c_client
 */
static int sn65dsi85_remove(struct i2c_client *c)
{
	struct sn65dsi85_device *ctx = i2c_get_clientdata(c);
	DRM_DEV_DEBUG(&c->dev, "%s entry", __func__);

	if(ctx->dsi) {
		sn65dsi85_detach_dsi(ctx);
	}

	drm_bridge_remove(&ctx->bridge);
	
	DRM_DEV_DEBUG(&c->dev, "%s exit", __func__);
	return 0;
}

/*
 * Device-Tree declaration 
 */
static const struct of_device_id sn65dsi85_of_match[] = {
	{.compatible = "ti,dsi85" },
	{}
};
MODULE_DEVICE_TABLE(of, sn65dsi85_of_match);


/*
 * This is an I2C client driver module instead of a general module
 */
static const struct i2c_device_id sn65dsi85_i2c_id[] = {
	{ "ti,dsi85", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn65dsi85_i2c_id);

static struct i2c_driver sn65dsi85_i2c_driver = {
	.driver = {
                .of_match_table = of_match_ptr(sn65dsi85_of_match),
		.owner = THIS_MODULE,
		.name = SN65DSI85_MODULE_NAME,
	},
	.probe = sn65dsi85_probe,
	.remove = sn65dsi85_remove,
	.id_table = sn65dsi85_i2c_id,
};

module_i2c_driver(sn65dsi85_i2c_driver);


MODULE_AUTHOR("Konrad Anton <konrad.anton@awinia.de>");
MODULE_DESCRIPTION("DRM Driver for SN65DSI85 MIPI-DSI-to-LVDS converter");
MODULE_LICENSE("GPL");

/* EOF */
